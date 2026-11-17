#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Latest state of the on-board ESKF, for the admin page. */
typedef struct {
    bool     valid;             /* false until the first packet after start-up */
    uint16_t loop_hz;
    float    temp;              /* deg C */
    float    roll, pitch, yaw;  /* deg, ESKF, same Euler convention as the lab */
    float    sigma[3];          /* deg, 1-sigma attitude uncertainty */
    float    bias[3];           /* deg/s, estimated gyro bias */
    uint8_t  flags;             /* same bits as the packet: gravity used, heading used, ... */
    uint32_t clip_gyro, clip_accel;   /* samples at full scale since boot */
    bool     mag_calibrated;    /* a magnetometer calibration is stored */
} imu_live_t;

void imu_get_live(imu_live_t *out);
