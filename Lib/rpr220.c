#include "rpr220.h"

static uint16_t sensor_min[NUM_SENSORS];
static uint16_t sensor_max[NUM_SENSORS];

// Ð?O M?NG TR?NG S? CHO HU?NG TI?N M?I (C?M BI?N D?N Ð?U):
// M?t 0 (Bên Ph?i m?i: +2000) -> M?t 2 (Gi?a: 0) -> M?t 4 (Bên Trái m?i: -2000)
static const int16_t weights[NUM_SENSORS] = {2000, 1000, 0, -1000, -2000};
static int16_t last_line_error = 0;

static uint32_t target_calib_ticks = 0;
static uint32_t current_calib_ticks = 0;
static uint8_t  is_calibrated = 0;

void RPR220_Init(uint32_t t_calib_ms)
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }
    last_line_error = 0;
    is_calibrated = 0;
    current_calib_ticks = 0;

    if (t_calib_ms == 0) {
        t_calib_ms = T_CALIB_DEFAULT_MS;
    }
    target_calib_ticks = t_calib_ms / TIM2_STEP_MS;
}

uint8_t RPR220_Calibrate_Process(volatile uint16_t *raw_adc)
{
    if (is_calibrated) return 1;

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (raw_adc[i] < sensor_min[i]) sensor_min[i] = raw_adc[i];
        if (raw_adc[i] > sensor_max[i]) sensor_max[i] = raw_adc[i];
    }

    current_calib_ticks++;
    if (current_calib_ticks >= target_calib_ticks)
    {
        is_calibrated = 1;
        return 1;	
    }

    return 0;
}

int16_t RPR220_Get_Error(volatile uint16_t *raw_adc, volatile uint16_t *norm_out)
{
    if (!is_calibrated) return 0;

    // 1. Chu?n hóa tuy?n tính t?ng m?t v? thang 0 -> 1000
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        uint16_t raw = raw_adc[i];
        if (raw < sensor_min[i]) raw = sensor_min[i];
        if (raw > sensor_max[i]) raw = sensor_max[i];

        uint16_t span = sensor_max[i] - sensor_min[i];
        if (span > 150) { 
            norm_out[i] = (uint16_t)(((uint32_t)(raw - sensor_min[i]) * 1000) / span);
        } else {
            norm_out[i] = 0;
        }
    }

    // 2. Tính Centroid: L?c nhi?u ngu?ng 60 d? b?t tín hi?u mu?t gi?a 2 m?t li?n k?
    int32_t weighted_sum = 0;
    int32_t sum_norm = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (norm_out[i] > 60)
        {
            weighted_sum += (int32_t)norm_out[i] * weights[i];
            sum_norm += norm_out[i];
        }
    }

    // 3. C?p nh?t v? trí sai s?
    if (sum_norm > 80)
    {
        last_line_error = (int16_t)(weighted_sum / sum_norm);
    }
    // Khi m?t v?ch (sum_norm <= 80): Gi? nguyên last_line_error tru?c dó,
    // không t? ý gán k?ch tr?n 2000/-2000 d? tránh s?c lái vi sai

    return last_line_error;
}