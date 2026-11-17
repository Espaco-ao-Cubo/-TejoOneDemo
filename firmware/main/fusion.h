#pragma once
/*
 * Orientation filters. Conventions match the IMU Fusion Lab page:
 *   world frame NWU (x = magnetic north, y = west, z = up)
 *   q = [w, x, y, z] rotates sensor-frame vectors into the world frame
 *   gyro in rad/s, accel in any unit (normalised inside), mag in uT
 */
#include <stdbool.h>

/* ---------- Mahony: PI feedback on the gravity / field direction error ---------- */
typedef struct {
    float q[4];
    float ib[3];        /* integral term, rad/s; -ib is the gyro bias estimate */
    float kp, ki;
} mahony_t;

void mahony_init(mahony_t *f, const float q0[4], float kp, float ki);
/* mag may be NULL (6-DOF mode) */
void mahony_update(mahony_t *f, const float gyro[3], const float acc[3], const float *mag, float dt);

/* ---------- Error-state Kalman filter (ESKF) ----------
 * Nominal state: quaternion q and gyro bias bg.
 * Error state (6): small attitude error dtheta (body frame) and bias error dbg.
 * Predict with the gyro every sample. Correct roll/pitch with gravity (3-D update, gated
 * when |a| is far from 1 g) and yaw with a heading-only magnetometer update (gated when |m|
 * jumps), so a nearby magnet can never tilt the estimate.
 */
typedef struct {
    float q[4];
    float bg[3];            /* rad/s */
    float P[6][6];
    /* tuning */
    float gyro_noise;       /* rad/s/sqrt(Hz)  angle random walk */
    float bias_walk;        /* rad/s/sqrt(s)   bias random walk  */
    float acc_noise;        /* in g (on the normalised vector) */
    float heading_noise;    /* rad */
    float acc_gate;         /* reject accel update when | |a| - 1 g | exceeds this */
    float mag_gate;         /* reject mag update when |m| differs from its running mean by this fraction */
    float dip_gate;         /* reject mag update when the field-to-gravity angle moves this far (rad) */
    /* status */
    float mag_ref, dip_ref;
    bool acc_used, mag_used;
} eskf_t;

void eskf_init(eskf_t *f, const float q0[4]);
void eskf_predict(eskf_t *f, const float gyro[3], float dt);
void eskf_update_accel(eskf_t *f, const float acc_g[3]);
void eskf_update_mag(eskf_t *f, const float mag[3]);
void eskf_sigma_deg(const eskf_t *f, float out[3]);
/* The gyro hit full scale this sample: the rotation it reported is too small by an unknown amount.
 * Widen the attitude uncertainty so gravity and the magnetometer pull the estimate back quickly. */
void eskf_gyro_clipped(eskf_t *f);

/* Level-and-north quaternion from one accel (+ optional mag) reading. */
void quat_from_accel_mag(const float acc[3], const float *mag, float q[4]);
