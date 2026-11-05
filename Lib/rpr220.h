#ifndef INC_RPR220_H_
#define INC_RPR220_H_

#include "main.h"

#define NUM_SENSORS             5
#define T_CALIB_DEFAULT_MS      15000    // Thoi gian Calib mac dinh T_calib = 7 giay
#define TIM2_STEP_MS            2       // Chu ky ngat TIM2 (2ms/tick)
#define NOISE_THRESHOLD         200     // Nguong cat nhieu phong
#define MIN_SPAN                200     // Do tuong phan toi thieu giua vach va nen

/**
 * @brief  Khoi tao cam bien va cai dat thoi gian T_calib
 * @param  t_calib_ms: Thoi gian lay mau calib (ms). Truyen 0 se lay mac dinh 7000ms.
 */
void RPR220_Init(uint32_t t_calib_ms);

/**
 * @brief  Thuc hien calib theo chu ky ngat 2ms
 * @param  raw_adc: Mang 5 gia tri ADC tho tu DMA
 * @return 0: Dang trong thoi gian T_calib
 *         1: Da du thoi gian T_calib, hoan tat qua trinh calib
 */
uint8_t RPR220_Calibrate_Process(volatile uint16_t *raw_adc);

/**
 * @brief  Chuan hoa tin hieu va tinh sai so vi tri Centroid (-2000 den +2000)
 * @param  raw_adc: Mang gia tri ADC tho tu DMA
 * @param  norm_out: Mang nhan gia tri da chuan hoa (0 -> 1000) de truyen UART len MATLAB
 * @return Sai so vi tri line_pos_error
 */
int16_t RPR220_Get_Error(volatile uint16_t *raw_adc, volatile uint16_t *norm_out);

#endif /* INC_RPR220_H_ */