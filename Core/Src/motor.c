#include "motor.h"

static TIM_HandleTypeDef *p_htim_motor = NULL;
static MotorInfo_t motor_info = {0.0f, 0.0f, MOTOR_STATE_BRAKE, MOTOR_STATE_BRAKE};

void Motor_Init(TIM_HandleTypeDef *htim)
{
    p_htim_motor = htim;

    // Kích ho?t PWM kênh 3 và 4 trên Timer 4
    HAL_TIM_PWM_Start(p_htim_motor, MOTOR_TIM_CH_L);
    HAL_TIM_PWM_Start(p_htim_motor, MOTOR_TIM_CH_R);

    // Kích ho?t chân STBY c?a TB6612
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);

    // Ban d?u khóa c?ng bánh xe an toàn
    Motor_Brake();
}

void Motor_SetDuty(float duty_L, float duty_R)
{
    if (p_htim_motor == NULL) return;

    // 1. K?p biên t?i da
    if (duty_L > MOTOR_MAX_DUTY)  duty_L = MOTOR_MAX_DUTY;
    if (duty_L < -MOTOR_MAX_DUTY) duty_L = -MOTOR_MAX_DUTY;
    if (duty_R > MOTOR_MAX_DUTY)  duty_R = MOTOR_MAX_DUTY;
    if (duty_R < -MOTOR_MAX_DUTY) duty_R = -MOTOR_MAX_DUTY;

    motor_info.duty_L = duty_L;
    motor_info.duty_R = duty_R;

    // Quy d?i % Duty sang giá tr? Compare Register (ARR = 799)
    uint16_t pulse_L = (uint16_t)((duty_L >= 0 ? duty_L : -duty_L) * (float)MOTOR_ARR / 100.0f);
    uint16_t pulse_R = (uint16_t)((duty_R >= 0 ? duty_R : -duty_R) * (float)MOTOR_ARR / 100.0f);

    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);

    // --- ÐI?U KHI?N BÁNH TRÁI (AIN1: PB12, AIN2: PB13) - ÐÃ Ð?O CHI?U TI?N V? PHÍA C?M BI?N ---
    if (duty_L > 0.0f)
    {
        motor_info.state_L = MOTOR_STATE_FORWARD;
        HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_L, pulse_L);
    }
    else if (duty_L < 0.0f)
    {
        motor_info.state_L = MOTOR_STATE_REVERSE;
        HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_L, pulse_L);
    }
    else
    {
        motor_info.state_L = MOTOR_STATE_BRAKE;
        HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_L, 0);
    }

    // --- ÐI?U KHI?N BÁNH PH?I (BIN1: PB14, BIN2: PB15) - ÐÃ Ð?O CHI?U TI?N V? PHÍA C?M BI?N ---
    if (duty_R > 0.0f)
    {
        motor_info.state_R = MOTOR_STATE_FORWARD;
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_R, pulse_R);
    }
    else if (duty_R < 0.0f)
    {
        motor_info.state_R = MOTOR_STATE_REVERSE;
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_R, pulse_R);
    }
    else
    {
        motor_info.state_R = MOTOR_STATE_BRAKE;
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_R, 0);
    }
}

void Motor_DriveLine(float base_duty, float steer)
{
    // Ð?ng h?c hu?ng m?i (Thanh c?m bi?n di tru?c):
    // - Nhìn theo hu?ng ti?n m?i: Bánh Ph?i v?t lý (Motor R) n?m ? bên TRÁI.
    // - Nhìn theo hu?ng ti?n m?i: Bánh Trái v?t lý (Motor L) n?m ? bên PH?I.
    //
    // steer > 0: L?ch Ph?i -> C?n b? Ph?i -> Tang bánh Trái m?i (Motor R), gi?m bánh Ph?i m?i (Motor L)
    // steer < 0: L?ch Trái  -> C?n b? Trái  -> Gi?m bánh Trái m?i (Motor R), tang bánh Ph?i m?i (Motor L)
    float target_L = base_duty - steer;
    float target_R = base_duty + steer;

    // Khóa sàn ch?ng k?t bánh: Bánh phía trong cua luôn gi? m?c t?i thi?u >= MOTOR_DEADBAND (15%)
    // Tri?t tiêu hi?n tu?ng su?ng bánh, quay compa ho?c ch?t d?ng d?ng co khi b? lái
    if (target_L < MOTOR_DEADBAND) target_L = MOTOR_DEADBAND;
    if (target_R < MOTOR_DEADBAND) target_R = MOTOR_DEADBAND;

    Motor_SetDuty(target_L, target_R);
}

void Motor_Brake(void)
{
    if (p_htim_motor == NULL) return;

    motor_info.duty_L = 0.0f;
    motor_info.duty_R = 0.0f;
    motor_info.state_L = MOTOR_STATE_BRAKE;
    motor_info.state_R = MOTOR_STATE_BRAKE;

    // AIN1=1, AIN2=1 & BIN1=1, BIN2=1 kích ho?t phanh di?n t? TB6612
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_L, 0);
    __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_R, 0);
}

void Motor_Coast(void)
{
    if (p_htim_motor == NULL) return;

    motor_info.duty_L = 0.0f;
    motor_info.duty_R = 0.0f;
    motor_info.state_L = MOTOR_STATE_COAST;
    motor_info.state_R = MOTOR_STATE_COAST;

    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_L, 0);
    __HAL_TIM_SET_COMPARE(p_htim_motor, MOTOR_TIM_CH_R, 0);
}

const MotorInfo_t* Motor_GetInfo(void)
{
    return &motor_info;
}