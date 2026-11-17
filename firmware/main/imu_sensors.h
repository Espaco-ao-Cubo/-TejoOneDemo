#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float acc[3];   /* g */
    float gyro[3];  /* deg/s, raw (no bias correction) */
    float temp;     /* deg C */
    uint8_t clipped;  /* bits 0-2: gyro x/y/z at full scale, bits 3-5: accel x/y/z at full scale */
} icm_sample_t;

/* Creates the I2C bus and configures both chips. Call from the task that will read them
 * (the I2C interrupt is bound to the core it was created on). */
void sensors_init(bool *icm_ok, bool *ak_ok);

/* ICM-20600 at 1 kHz internal rate: accel + temp + gyro in one 14-byte burst (~0.4 ms at 400 kHz). */
esp_err_t icm_read(icm_sample_t *s);

/* AK09918 in 100 Hz continuous mode. *fresh is true only when a new, non-overflowed sample was read. */
esp_err_t ak_read(float mag_ut[3], bool *fresh);
