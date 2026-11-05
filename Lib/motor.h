#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/* --- C?U HÌNH TIMER & DRIVER TB6612 --- */
#define MOTOR_ARR               799             // ARR c?a TIM4 t?i 15kHz
#define MOTOR_TIM_CH_L          TIM_CHANNEL_3   // PB8 (PWMA - Bánh Trái)
#define MOTOR_TIM_CH_R          TIM_CHANNEL_4   // PB9 (PWMB - Bánh Ph?i)

/* --- C?U HÌNH CHÂN GPIO TB6612 --- */
#define MOTOR_STBY_PORT         GPIOA
#define MOTOR_STBY_PIN          GPIO_PIN_8

#define MOTOR_AIN1_PORT         GPIOB
#define MOTOR_AIN1_PIN          GPIO_PIN_12
#define MOTOR_AIN2_PORT         GPIOB
#define MOTOR_AIN2_PIN          GPIO_PIN_13

#define MOTOR_BIN1_PORT         GPIOB
#define MOTOR_BIN1_PIN          GPIO_PIN_14
#define MOTOR_BIN2_PORT         GPIOB
#define MOTOR_BIN2_PIN          GPIO_PIN_15

/* --- THÔNG S? Ð?C TUY?N XE CÓ T?I 2KG --- */
#define MOTOR_DEADBAND          15.0f   // Ngu?ng Duty t?i thi?u th?ng ma sát tinh (%)
#define MOTOR_MAX_DUTY          80.0f   // Gi?i h?n ga t?i da d? tránh tru?t t?i (%)

/* --- Ð?NH NGHIA TR?NG THÁI V?N HÀNH --- */
typedef enum {
    MOTOR_STATE_COAST = 0,  // Th? trôi t? do
    MOTOR_STATE_FORWARD,    // Quay ti?n
    MOTOR_STATE_REVERSE,    // Quay lùi
    MOTOR_STATE_BRAKE       // Khóa c?ng (Short Brake)
} MotorWheelState_t;

typedef struct {
    float duty_L;
    float duty_R;
    MotorWheelState_t state_L;
    MotorWheelState_t state_R;
} MotorInfo_t;

/* --- HÀM GIAO TI?P NGOÀI --- */
/**
 * @brief Kh?i t?o PWM và kích ho?t driver TB6612
 */
void Motor_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Ði?u khi?n d?c l?p Duty 2 bánh (-100.0% d?n +100.0%)
 */
void Motor_SetDuty(float duty_L, float duty_R);

/**
 * @brief Hàm chuyên bi?t cho Dò Line:
 *        T? phân ph?i vi sai, bù vùng ch?t Deadband cho bánh phía trong cua.
 * @param base_duty: T?c d? n?n di th?ng (khuyên dùng 24.0f -> 28.0f)
 * @param steer:     Giá tr? b? lái t? c?m bi?n/PID (steer > 0: r? ph?i, steer < 0: r? trái)
 */
void Motor_DriveLine(float base_duty, float steer);

/**
 * @brief Phanh c?ng bánh xe t?c thì (Short Brake) khi g?p v?ch dích / s? c?
 */
void Motor_Brake(void);

/**
 * @brief Th? trôi bánh xe t? do (Coast)
 */
void Motor_Coast(void);

/**
 * @brief L?y thông tin Duty và tr?ng thái hi?n t?i c?a 2 bánh
 */
const MotorInfo_t* Motor_GetInfo(void);

#endif /* INC_MOTOR_H_ */