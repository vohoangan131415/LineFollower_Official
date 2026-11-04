#ifndef __HX711_H
#define __HX711_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// --- C?U HÌNH PH?N C?NG (ÐÃ Ð?I THEO SO Ð? M?I) ---
#define HX711_DT_PORT           GPIOB
#define HX711_DT_PIN            GPIO_PIN_10  // DT: Input (5V-tolerant)

#define HX711_SCK_PORT          GPIOB
#define HX711_SCK_PIN           GPIO_PIN_11   // SCK: Output Push-Pull

#define HX711_DEFAULT_SCALE     420.0f       // H? s? t? l? gram m?c d?nh

// --- CÁC HÀM GIAO TI?P NGO?I VI ---
void    HX711_Init(float scale);
uint8_t HX711_IsReady(void);
int32_t HX711_ReadRaw(void);
int32_t HX711_ReadAverage(uint8_t samples);
void    HX711_Tare(uint8_t samples);
float   HX711_GetGram(uint8_t samples);
void    HX711_SetScale(float scale);
void    HX711_SetTare(int32_t offset);
int32_t HX711_GetTare(void);
void    HX711_PowerDown(void);
void    HX711_PowerUp(void);

#ifdef __cplusplus
}
#endif

#endif /* __HX711_H */