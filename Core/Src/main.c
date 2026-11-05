/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Giam sat dong bo: 5 LED ADC, Sai so Error, RPM 2 banh xe
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "rpr220.h"
#include "hx711.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- THÔNG SỐ ĐỘNG CƠ & ENCODER ---
#define ENCODER_PPR             11      // Xung/vòng của đĩa từ
#define GEAR_RATIO              35.5f   // Tỉ số truyền hộp số thực tế
#define TOTAL_PULSES_PER_REV    (ENCODER_PPR * 4.0f * GEAR_RATIO) // 11 * 4 * 35.5 = 1562.0 xung/vòng

// Biến đo vận tốc thực tế
volatile float rpm_L = 0.0f;
volatile float rpm_R = 0.0f;

// --- DÀN CẢM BIẾN RPR-220 & BÁM LINE ---
volatile uint16_t adc_sensor[NUM_SENSORS];   // Buffer nhận dữ liệu ADC DMA
volatile uint16_t sensor_norm[NUM_SENSORS];  // Mảng chuẩn hóa (0 -> 1000)
volatile int16_t  line_pos_error = 0;
volatile int16_t  prev_line_error = 0;
volatile uint8_t  calib_completed = 0;
volatile float current_weight_g = 0.0f;
uint32_t last_hx711_tick = 0;

// Hệ số bám line thử nghiệm
#define BASE_DUTY               30.0f   // Mức tốc độ nền có tải 2kg (%)
#define KP_LINE                 0.007f  // Hệ số P
#define KD_LINE                 0.0003f // Hệ số D
#define MAX_STEER               11.0f   // Kẹp biên lái vi sai an toàn (%)

// Giao tiếp UART lên MATLAB
volatile uint8_t tx_ready = 0;
char tx_buffer[128];

typedef enum {
    WEIGHT_STATE_IDLE = 0,    // Bàn cân trống
    WEIGHT_STATE_SETTLING,    // Đang chờ tay người buông ra ổn định
    WEIGHT_STATE_LOCKED       // Đã khóa cứng loại hàng
} WeightState_t;

typedef enum {
    PACKAGE_NONE = 0,
    PACKAGE_1KG  = 1,
    PACKAGE_2KG  = 2
} PackageType_t;

volatile WeightState_t weight_state = WEIGHT_STATE_IDLE;
volatile PackageType_t detected_package = PACKAGE_NONE;
volatile float locked_weight_g = 0.0f;
volatile uint8_t is_weight_locked = 0;

static uint32_t settle_start_tick = 0;

/* ========================================================================= */
/* CẤU HÌNH & BIẾN ĐIỀU KHIỂN RẼ NHÁNH NGÃ BA CHỮ Y                          */
/* ========================================================================= */
#define SENSOR_ACTIVE_THRESH   400   // Ngưỡng phát hiện vạch đen (0 - 1000)
#define SUM_WIDE_LINE_THRESH   1000  // Ngưỡng tổng năng lượng báo hiệu ngã ba
#define WIDE_LINE_DEBOUNCE_CNT 3     // Giữ liên tục 3 chu kỳ ngắt (6ms) để chống nhiễu
#define BRANCH_HOLD_TIME_MS    800   // Thời gian duy trì ép lái qua ngã ba (ms)

// Trạng thái của module rẽ nhánh
typedef enum {
    BRANCH_WAITING_FORK = 0, // Đang chờ tới ngã ba
    BRANCH_TURNING,          // Đang trong ngã ba, ép lái
    BRANCH_COMPLETED         // Đã qua ngã ba an toàn
} BranchStatus_t;

volatile BranchStatus_t branch_status = BRANCH_WAITING_FORK;
uint32_t branch_start_time = 0;       // Mốc thời gian bắt đầu rẽ (ms)
uint8_t  wide_line_counter = 0;       // Bộ đếm debounce chống nhiễu sàn

// Mảng giá trị cảm biến sau khi lọc qua mặt nạ
uint16_t sensor_filtered[NUM_SENSORS] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float u_steer;

/**
 * @brief  Kiểm tra ngã ba chữ Y bằng Tổng năng lượng và Cặp mắt kết hợp
 * @retval 1: Đã chạm ngã ba thật | 0: Line đơn hoặc nhiễu sàn
 */
uint8_t Check_Wide_Line(void)
{
    uint32_t total_sensor_sum = 0;
    uint8_t active_count = 0;

    // 1. Tính tổng năng lượng và đếm số mắt vượt ngưỡng 400
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        total_sensor_sum += sensor_norm[i];
        if (sensor_norm[i] >= SENSOR_ACTIVE_THRESH)
        {
            active_count++;
        }
    }

    uint8_t is_wide = 0;

    // 2. Điều kiện nhận diện ngã ba:
    if (total_sensor_sum >= SUM_WIDE_LINE_THRESH)
    {
        is_wide = 1;
    }
    else if (((sensor_norm[1] >= SENSOR_ACTIVE_THRESH && sensor_norm[2] >= SENSOR_ACTIVE_THRESH) ||
              (sensor_norm[2] >= SENSOR_ACTIVE_THRESH && sensor_norm[3] >= SENSOR_ACTIVE_THRESH)) 
              && (total_sensor_sum >= 1000))
    {
        is_wide = 1;
    }
    else if (active_count >= 3)
    {
        is_wide = 1;
    }

    // 3. Bộ lọc Debounce (6ms)
    if (is_wide)
    {
        wide_line_counter++;
        if (wide_line_counter >= WIDE_LINE_DEBOUNCE_CNT)
        {
            return 1;
        }
    }
    else
    {
        wide_line_counter = 0;
    }

    return 0;
}

/**
 * @brief  Ép giá trị cảm biến về 0 tùy theo hướng cần rẽ
 */
void Apply_Sensor_Mask(void)
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensor_filtered[i] = sensor_norm[i];
    }

    if (branch_status == BRANCH_TURNING)
    {
        if (detected_package == PACKAGE_1KG) 
        {
            // Tải 1kg: rẽ TRÁI -> xóa mắt bên phải
            sensor_filtered[3] = 0; // S4
            sensor_filtered[4] = 0; // S5
        }
        else if (detected_package == PACKAGE_2KG) 
        {
            // Tải 2kg: rẽ PHẢI -> xóa mắt bên trái
            sensor_filtered[0] = 0; // S1
            sensor_filtered[1] = 0; // S2
        }
    }
}

/**
 * @brief Tính sai số vị trí line dựa trên mảng cảm biến đã lọc qua mặt nạ
 */
int16_t Calculate_Error_From_Filtered(uint16_t *sens)
{
    const int32_t weights[5] = {-2000, -1000, 0, 1000, 2000};
    int64_t weighted_sum = 0;
    int32_t total_val = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        weighted_sum += (int64_t)sens[i] * weights[i];
        total_val += sens[i];
    }

    if (total_val < 150)
    {
        return line_pos_error;
    }

    return (int16_t)(weighted_sum / total_val);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // NGẮT ĐỊNH THỜI 2ms TRÊN TIM1
    if (htim->Instance == TIM1)
    {
        // -------------------------------------------------------------
        // 1. GIAI ĐOẠN CALIB: 7 GIÂY ĐẦU TIÊN
        // -------------------------------------------------------------
        if (!calib_completed)
        {
            calib_completed = RPR220_Calibrate_Process(adc_sensor);
            Motor_Brake(); // Khóa cứng bánh để xe đứng yên khi calib
            return;
        }

        // -------------------------------------------------------------
        // 2. CHUẨN HÓA DỮ LIỆU & QUẢN LÝ RẼ NHÁNH NGÃ BA CHỮ Y
        // -------------------------------------------------------------
        line_pos_error = RPR220_Get_Error(adc_sensor, sensor_norm);

        // Khai báo biến đạo hàm D dùng chung
        static uint8_t  d_tick = 0;
        static float    d_filt = 0.0f;
        static int16_t  err_d_prev = 0;

        switch (branch_status)
        {
            case BRANCH_WAITING_FORK:
                if (is_weight_locked && detected_package != PACKAGE_NONE)
                {
                    if (Check_Wide_Line())
                    {
                        branch_start_time = HAL_GetTick();
                        branch_status = BRANCH_TURNING; // Bắt đầu bẻ nhánh
                    }
                }
                break;

            case BRANCH_TURNING:
                Apply_Sensor_Mask();
                line_pos_error = Calculate_Error_From_Filtered(sensor_filtered);

                if (HAL_GetTick() - branch_start_time >= BRANCH_HOLD_TIME_MS)
                {
                    branch_status = BRANCH_COMPLETED; // Hoàn tất bẻ nhánh

                    // 1. Phục hồi mảng cảm biến về nguyên bản (mở lại đủ 5 mắt)
                    for (uint8_t i = 0; i < NUM_SENSORS; i++)
                    {
                        sensor_filtered[i] = sensor_norm[i];
                    }

                    // 2. Tính lại sai số thực tế ngay lập tức
                    line_pos_error = RPR220_Get_Error(adc_sensor, sensor_norm);

                    // 3. Triệt tiêu Derivative Kick để tránh giật bánh
                    err_d_prev = line_pos_error;
                    d_filt = 0.0f;
                    d_tick = 0;
                }
                break;

            case BRANCH_COMPLETED:
                break;
        }

        // -------------------------------------------------------------
        // 3. THUẬT TOÁN ĐIỀU KHIỂN ĐỘNG CƠ (ÉP LÁI VS BÁM PD)
        // -------------------------------------------------------------
        if (branch_status == BRANCH_TURNING)
        {
            // Ép thẳng góc lái cực đại để thắng lực quán tính của tải
            if (detected_package == PACKAGE_1KG)
            {
                u_steer = -26.0f; // Rẽ TRÁI
            }
            else if (detected_package == PACKAGE_2KG)
            {
                u_steer = 26.0f;  // Rẽ PHẢI
            }
        }
        else
        {
            // Bám line bình thường bằng PD
            float p_term = KP_LINE * (float)line_pos_error;

            if (++d_tick >= 5)   // 10 ms
            {
                d_tick = 0;
                float d_raw = (float)(line_pos_error - err_d_prev) / 0.010f;
                err_d_prev = line_pos_error;
                d_filt += 0.3f * (d_raw - d_filt);
            }

            float d_term = KD_LINE * d_filt;
            u_steer = p_term + d_term;

            if (u_steer >  MAX_STEER) u_steer =  MAX_STEER;
            if (u_steer < -MAX_STEER) u_steer = -MAX_STEER;
        }

        Motor_DriveLine(BASE_DUTY, u_steer);

        // -------------------------------------------------------------
        // 4. TÍNH VẬN TỐC RPM MỖI 50ms (GỬI TELEMETRY)
        // -------------------------------------------------------------
        static uint8_t tick_50ms = 0;
        if (++tick_50ms >= 25)
        {
            tick_50ms = 0;

            int16_t pulses_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
            int16_t pulses_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            __HAL_TIM_SET_COUNTER(&htim2, 0);

            rpm_L = ((float)pulses_L / TOTAL_PULSES_PER_REV) * 1200.0f;
            rpm_R = ((float)pulses_R / TOTAL_PULSES_PER_REV) * 1200.0f;
            if (rpm_L < 0) rpm_L = -rpm_L;
            if (rpm_R < 0) rpm_R = -rpm_R;

            tx_ready = 1;
        }
    }
}

void Process_Weight_Logic(float raw_weight)
{
    switch (weight_state)
    {
        case WEIGHT_STATE_IDLE:
            is_weight_locked  = 0;
            detected_package  = PACKAGE_NONE;
            locked_weight_g   = 0.0f;
            branch_status     = BRANCH_WAITING_FORK; // Sẵn sàng rẽ cho lượt mới
            wide_line_counter = 0;

            if (raw_weight > 500.0f)
            {
                settle_start_tick = HAL_GetTick();
                weight_state = WEIGHT_STATE_SETTLING;
            }
            break;

        case WEIGHT_STATE_SETTLING:
            if (raw_weight < 300.0f)
            {
                weight_state = WEIGHT_STATE_IDLE;
                break;
            }

            if (HAL_GetTick() - settle_start_tick >= 1200)
            {
                if (raw_weight >= 800.0f && raw_weight <= 1350.0f)
                {
                    locked_weight_g   = 1000.0f;
                    detected_package  = PACKAGE_1KG;
                    is_weight_locked  = 1;
                    weight_state      = WEIGHT_STATE_LOCKED;
                }
                else if (raw_weight >= 1650.0f && raw_weight <= 2400.0f)
                {
                    locked_weight_g   = 2000.0f;
                    detected_package  = PACKAGE_2KG;
                    is_weight_locked  = 1;
                    weight_state      = WEIGHT_STATE_LOCKED;
                }
                else
                {
                    settle_start_tick = HAL_GetTick() - 900;
                }
            }
            break;

        case WEIGHT_STATE_LOCKED:
            // Tự động mở khóa và reset cờ ngã ba khi dỡ hàng
            if (raw_weight < 200.0f)
            {
                weight_state      = WEIGHT_STATE_IDLE;
                branch_status     = BRANCH_WAITING_FORK;
                wide_line_counter = 0;
                branch_start_time = 0;
            }
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // 1. Khởi động driver TB6612 trên TIM4
  Motor_Init(&htim4);

  // 2. Kích hoạt chế độ đếm Encoder phần cứng trên TIM2 và TIM3
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // 3. Khởi tạo cảm biến RPR-220 và DMA
  RPR220_Init(15000);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_sensor, NUM_SENSORS);

  // 4. Khởi tạo HX711 và trừ bì ban đầu
  HX711_Init(420.0f);
  HX711_Tare(15);

  // 5. Bật ngắt định thời 2ms trên TIM1
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. Đọc khối lượng non-blocking mỗi 100ms
    if (HAL_GetTick() - last_hx711_tick >= 100)
    {
        last_hx711_tick = HAL_GetTick();

        if (HX711_IsReady())
        {
            float raw_val = HX711_GetGram(1);
            Process_Weight_Logic(raw_val);

            if (is_weight_locked)
            {
                current_weight_g = locked_weight_g;
            }
            else
            {
                current_weight_g = raw_val;
            }
        }
    }

    // 2. Gửi telemetry lên MATLAB mỗi 50ms
    if (tx_ready)
    {
        tx_ready = 0;

        int l_int  = (int)rpm_L;
        int l_frac = (int)((rpm_L - l_int) * 10.0f + 0.5f);
        if (l_frac >= 10) { l_int++; l_frac = 0; }

        int r_int  = (int)rpm_R;
        int r_frac = (int)((rpm_R - r_int) * 10.0f + 0.5f);
        if (r_frac >= 10) { r_int++; r_frac = 0; }

        int st_int  = (int)u_steer;
        int st_frac = (int)((u_steer - (float)st_int) * 10.0f);
        if (st_frac < 0) st_frac = -st_frac;

        int w_val = (int)current_weight_g;

        int len = sprintf(tx_buffer, "%u,%u,%u,%u,%u,%d,%d,%d,%d.%d,%d,%d.%d,%d.%d\n",
                          sensor_norm[0], sensor_norm[1], sensor_norm[2],
                          sensor_norm[3], sensor_norm[4],
                          line_pos_error,
                          detected_package,
                          branch_status,
                          st_int, st_frac,
                          w_val,
                          l_int, l_frac,
                          r_int, r_frac);

        HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 25);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */