#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

// --- THÔNG S? TIMER & PWM ---
#define MOTOR_ARR               799             // ARR c?a TIM4 t?i 15kHz
#define MOTOR_TIM_CH_L          TIM_CHANNEL_3   // PB8 (PWMA)
#define MOTOR_TIM_CH_R          TIM_CHANNEL_4   // PB9 (PWMB)

// --- Ð?NH NGHIA CHÂN GPIO ---
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

/**
 * @brief Kh?i t?o driver TB6612, b?t xung PWM và kéo chân STBY
 */
void Motor_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Ði?u khi?n t?c d? và hu?ng theo % Duty (-100.0% d?n +100.0%)
 *        duty = 0.0f: Th? trôi t? do (Coast)
 */
void Motor_SetDuty(float duty_L, float duty_R);

/**
 * @brief Phanh hãm c?ng (Short Brake): Khóa c?ng 2 bánh d?ng ngay t?i di?m
 */
void Motor_Brake(void);

#endif /* INC_MOTOR_H_ */