#include "hx711.h"

static int32_t tare_offset = 0;
static float scale_factor = HX711_DEFAULT_SCALE;

// Hàm tr? micro-giây cho nh?p xung Clock
static void hx711_delay_us(void)
{
    for (volatile uint8_t i = 0; i < 10; i++)
    {
        __NOP();
    }
}

// Kh?i t?o tr?ng thái ban d?u cho module
void HX711_Init(float scale)
{
    if (scale != 0.0f)
    {
        scale_factor = scale;
    }

    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
}

// Ki?m tra chân DT dã kéo xu?ng m?c 0 (d? li?u s?n sàng) hay chua (Non-blocking)
uint8_t HX711_IsReady(void)
{
    return (HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN) == GPIO_PIN_RESET);
}

// Ð?c 1 m?u ADC thô 24-bit (Kênh A, Gain 128)
int32_t HX711_ReadRaw(void)
{
    uint32_t count = 0;
    uint32_t start_time = HAL_GetTick();

    // Ch? DT xu?ng 0 (t?i da 150ms d? ch?ng treo CPU khi l?ng dây)
    while (!HX711_IsReady())
    {
        if ((HAL_GetTick() - start_time) > 150)
        {
            return 0;
        }
    }

    // T?m khóa ng?t trong lúc kéo xung bit-banging (~40us)
    // Ngan TIM1 nh?y vào gi?a lúc SCK dang ? m?c HIGH (>60us s? làm HX711 sleep)
    __disable_irq();

    // Kéo 24 bit d? li?u
    for (uint8_t i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        hx711_delay_us();
        count = count << 1;

        if (HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN) == GPIO_PIN_SET)
        {
            count++;
        }

        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        hx711_delay_us();
    }

    // Xung th? 25 ch?t Kênh A, d? l?i Gain 128
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    hx711_delay_us();
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    hx711_delay_us();

    __enable_irq();

    // X? lý bù 2 s? âm 24-bit
    if (count & 0x800000)
    {
        count |= 0xFF000000;
    }

    return (int32_t)count;
}

// L?y trung bình nhi?u m?u
int32_t HX711_ReadAverage(uint8_t samples)
{
    if (samples == 0) samples = 1;

    int64_t sum = 0;
    uint8_t valid = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        int32_t val = HX711_ReadRaw();
        if (val != 0)
        {
            sum += val;
            valid++;
        }
        HAL_Delay(5);
    }

    if (valid == 0) return 0;
    return (int32_t)(sum / valid);
}

// T? d?ng tr? bì ban d?u (bàn cân ph?i tr?ng)
void HX711_Tare(uint8_t samples)
{
    tare_offset = HX711_ReadAverage(samples);
}

// Tr? v? kh?i lu?ng tính b?ng Gram
float HX711_GetGram(uint8_t samples)
{
    int32_t raw = HX711_ReadAverage(samples);
    if (raw == 0 && tare_offset != 0) return 0.0f;
    return (float)(raw - tare_offset) / scale_factor;
}

void HX711_SetScale(float scale)
{
    if (scale != 0.0f) scale_factor = scale;
}

void HX711_SetTare(int32_t offset)
{
    tare_offset = offset;
}

int32_t HX711_GetTare(void)
{
    return tare_offset;
}

void HX711_PowerDown(void)
{
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
}

void HX711_PowerUp(void)
{
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
}