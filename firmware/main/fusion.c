#include <math.h>
#include <string.h>
#include "fusion.h"

#define RAD2DEG 57.29577951f

/* ======================= quaternion helpers ======================= */
static void quat_mul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void quat_norm(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    for (int i = 0; i < 4; i++) q[i] /= n;
}

/* q <- q (x) Exp(v), v = rotation vector in rad */
static void quat_rotate_by(float q[4], const float v[3])
{
    float a = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    float d[4];
    if (a < 1e-9f) {
        d[0] = 1; d[1] = v[0]*0.5f; d[2] = v[1]*0.5f; d[3] = v[2]*0.5f;
    } else {
        float s = sinf(a*0.5f) / a;
        d[0] = cosf(a*0.5f); d[1] = v[0]*s; d[2] = v[1]*s; d[3] = v[2]*s;
    }
    float o[4];
    quat_mul(q, d, o);
    memcpy(q, o, sizeof(o));
    quat_norm(q);
}

static void quat_to_R(const float q[4], float R[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2*(y*y + z*z); R[0][1] = 2*(x*y - w*z);     R[0][2] = 2*(x*z + w*y);
    R[1][0] = 2*(x*y + w*z);     R[1][1] = 1 - 2*(x*x + z*z); R[1][2] = 2*(y*z - w*x);
    R[2][0] = 2*(x*z - w*y);     R[2][1] = 2*(y*z + w*x);     R[2][2] = 1 - 2*(x*x + y*y);
}

void quat_from_accel_mag(const float a[3], const float *m, float q[4])
{
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
    float yaw = 0;
    if (m) {
        float sr = sinf(roll), cr = cosf(roll), sp = sinf(pitch), cp = cosf(pitch);
        float hx = m[0]*cp + m[1]*sp*sr + m[2]*sp*cr;
        float hy = m[1]*cr - m[2]*sr;
        yaw = atan2f(-hy, hx);
    }
    float cr = cosf(roll/2), sr = sinf(roll/2), cp = cosf(pitch/2), sp = sinf(pitch/2), cy = cosf(yaw/2), sy = sinf(yaw/2);
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
}

/* ======================= Mahony ======================= */
void mahony_init(mahony_t *f, const float q0[4], float kp, float ki)
{
    memcpy(f->q, q0, sizeof(f->q));
    f->ib[0] = f->ib[1] = f->ib[2] = 0;
    f->kp = kp;
    f->ki = ki;
}

void mahony_update(mahony_t *f, const float g[3], const float a[3], const float *m, float dt)
{
    float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];
    float gx = g[0], gy = g[1], gz = g[2];
    float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (an > 1e-6f) {
        float ax = a[0]/an, ay = a[1]/an, az = a[2]/an;
        /* half of predicted gravity direction in sensor frame */
        float vx = q1*q3 - q0*q2, vy = q0*q1 + q2*q3, vz = q0*q0 - 0.5f + q3*q3;
        float ex = ay*vz - az*vy, ey = az*vx - ax*vz, ez = ax*vy - ay*vx;
        float mn = m ? sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]) : 0;
        if (mn > 1e-6f) {
            float mx = m[0]/mn, my = m[1]/mn, mz = m[2]/mn;
            float hx = 2*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
            float hy = 2*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
            float bx = sqrtf(hx*hx + hy*hy);
            float bz = 2*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));
            float wx = bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2);
            float wy = bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3);
            float wz = bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2);
            ex += my*wz - mz*wy; ey += mz*wx - mx*wz; ez += mx*wy - my*wx;
        }
        ex *= 2; ey *= 2; ez *= 2;
        if (f->ki > 0) {
            f->ib[0] += f->ki*ex*dt; f->ib[1] += f->ki*ey*dt; f->ib[2] += f->ki*ez*dt;
        } else {
            f->ib[0] = f->ib[1] = f->ib[2] = 0;
        }
        gx += f->ib[0] + f->kp*ex;
        gy += f->ib[1] + f->kp*ey;
        gz += f->ib[2] + f->kp*ez;
    }
    float v[3] = { gx*dt, gy*dt, gz*dt };
    quat_rotate_by(f->q, v);
}

/* ======================= ESKF ======================= */
#define N 6

static void eskf_inject(eskf_t *f, const float dx[N])
{
    quat_rotate_by(f->q, dx);       /* q <- q (x) Exp(dtheta) */
    f->bg[0] += dx[3]; f->bg[1] += dx[4]; f->bg[2] += dx[5];
}

static void sym(float P[N][N])
{
    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++) {
            float v = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = v;
        }
}

void eskf_init(eskf_t *f, const float q0[4])
{
    memset(f, 0, sizeof(*f));
    memcpy(f->q, q0, sizeof(f->q));
    const float att0 = 0.1f;                 /* ~6 deg initial attitude std */
    const float bias0 = 2.0f / RAD2DEG;      /* 2 deg/s initial bias std */
    for (int i = 0; i < 3; i++) { f->P[i][i] = att0*att0; f->P[i+3][i+3] = bias0*bias0; }
    /* From a 2 h Allan variance recording of this board (2026-11-15), still, at 26 C:
     * white noise N = 0.0030-0.0033 deg/s/sqrt(Hz), rate random walk K = 1.3e-4 deg/s/sqrt(s).
     * Used about 3-4x larger to leave room for vibration and scale-factor errors in real motion.
     * Noise densities don't depend on the loop rate, so these hold at 500 Hz and 1 kHz alike. */
    f->gyro_noise = 0.01f / RAD2DEG;         /* deg/s/sqrt(Hz) */
    f->bias_walk = 0.0005f / RAD2DEG;        /* deg/s/sqrt(s) */
    f->acc_noise = 0.03f;
    f->heading_noise = 3.0f / RAD2DEG;
    f->acc_gate = 0.15f;
    f->mag_gate = 0.15f;
    f->dip_gate = 6.0f / RAD2DEG;
}

void eskf_predict(eskf_t *f, const float g[3], float dt)
{
    float w[3] = { g[0] - f->bg[0], g[1] - f->bg[1], g[2] - f->bg[2] };
    float v[3] = { w[0]*dt, w[1]*dt, w[2]*dt };
    quat_rotate_by(f->q, v);

    /* Phi = [ I - [v]x   -I*dt ]
     *       [ 0           I    ]  */
    float F[N][N] = { 0 };
    for (int i = 0; i < N; i++) F[i][i] = 1;
    F[0][1] =  v[2]; F[0][2] = -v[1];
    F[1][0] = -v[2]; F[1][2] =  v[0];
    F[2][0] =  v[1]; F[2][1] = -v[0];
    F[0][3] = F[1][4] = F[2][5] = -dt;

    float FP[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0;
            for (int k = 0; k < N; k++) s += F[i][k] * f->P[k][j];
            FP[i][j] = s;
        }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0;
            for (int k = 0; k < N; k++) s += FP[i][k] * F[j][k];
            f->P[i][j] = s;
        }
    float qa = f->gyro_noise * f->gyro_noise * dt;
    float qb = f->bias_walk * f->bias_walk * dt;
    for (int i = 0; i < 3; i++) { f->P[i][i] += qa; f->P[i+3][i+3] += qb; }
    sym(f->P);
}

static bool inv3(const float A[3][3], float B[3][3])
{
    float c00 = A[1][1]*A[2][2] - A[1][2]*A[2][1];
    float c01 = A[1][2]*A[2][0] - A[1][0]*A[2][2];
    float c02 = A[1][0]*A[2][1] - A[1][1]*A[2][0];
    float det = A[0][0]*c00 + A[0][1]*c01 + A[0][2]*c02;
    if (fabsf(det) < 1e-20f) return false;
    float id = 1.0f / det;
    B[0][0] = c00*id; B[0][1] = (A[0][2]*A[2][1] - A[0][1]*A[2][2])*id; B[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1])*id;
    B[1][0] = c01*id; B[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0])*id; B[1][2] = (A[0][2]*A[1][0] - A[0][0]*A[1][2])*id;
    B[2][0] = c02*id; B[2][1] = (A[0][1]*A[2][0] - A[0][0]*A[2][1])*id; B[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0])*id;
    return true;
}

void eskf_update_accel(eskf_t *f, const float a[3])
{
    f->acc_used = false;
    float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (an < 1e-3f || fabsf(an - 1.0f) > f->acc_gate) return;     /* accelerating: don't trust gravity */
    float z[3] = { a[0]/an, a[1]/an, a[2]/an };

    float R[3][3];
    quat_to_R(f->q, R);
    float gh[3] = { R[2][0], R[2][1], R[2][2] };                     /* predicted "up" in sensor frame */
    float y[3] = { z[0] - gh[0], z[1] - gh[1], z[2] - gh[2] };

    /* H = [ [gh]x  0 ] */
    float H[3][3] = { { 0, -gh[2], gh[1] }, { gh[2], 0, -gh[0] }, { -gh[1], gh[0], 0 } };
    float PHt[N][3];
    for (int i = 0; i < N; i++)
        for (int r = 0; r < 3; r++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += f->P[i][k] * H[r][k];
            PHt[i][r] = s;
        }
    float S[3][3], Si[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += H[r][k] * PHt[k][c];
            S[r][c] = s + (r == c ? f->acc_noise * f->acc_noise : 0);
        }
    if (!inv3(S, Si)) return;
    float K[N][3], dx[N];
    for (int i = 0; i < N; i++) {
        dx[i] = 0;
        for (int c = 0; c < 3; c++) {
            float s = 0;
            for (int r = 0; r < 3; r++) s += PHt[i][r] * Si[r][c];
            K[i][c] = s;
            dx[i] += s * y[c];
        }
    }
    /* P = P - K (H P),  with H P = PHt^T because P is symmetric */
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0;
            for (int r = 0; r < 3; r++) s += K[i][r] * PHt[j][r];
            f->P[i][j] -= s;
        }
    sym(f->P);
    eskf_inject(f, dx);
    f->acc_used = true;
}

void eskf_update_mag(eskf_t *f, const float m[3])
{
    f->mag_used = false;
    float mn = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (mn < 1.0f) return;

    float R[3][3];
    quat_to_R(f->q, R);
    /* A magnet or iron nearby changes the field strength and/or its angle to gravity (dip).
     * Both references adapt slowly (~5 s) so a slowly approaching magnet can't drag them along. */
    float c = (R[2][0]*m[0] + R[2][1]*m[1] + R[2][2]*m[2]) / mn;
    float dip = acosf(fminf(1.0f, fmaxf(-1.0f, c)));
    if (f->mag_ref <= 0) { f->mag_ref = mn; f->dip_ref = dip; }
    if (fabsf(mn / f->mag_ref - 1.0f) > f->mag_gate || fabsf(dip - f->dip_ref) > f->dip_gate) return;
    f->mag_ref += 0.002f * (mn - f->mag_ref);
    f->dip_ref += 0.002f * (dip - f->dip_ref);

    float mw[3];
    for (int i = 0; i < 3; i++) mw[i] = R[i][0]*m[0] + R[i][1]*m[1] + R[i][2]*m[2];
    float hh = sqrtf(mw[0]*mw[0] + mw[1]*mw[1]);
    if (hh < 0.15f * mn) return;                                   /* field almost vertical: heading undefined */
    float z = atan2f(mw[1], mw[0]);                                /* heading of the field in world; 0 when yaw is right */

    /* h(dtheta) = -gh^T dtheta  -> H = [ -gh  0 ] (only the rotation about world-up) */
    float H[3] = { -R[2][0], -R[2][1], -R[2][2] };
    float PHt[N];
    for (int i = 0; i < N; i++) PHt[i] = f->P[i][0]*H[0] + f->P[i][1]*H[1] + f->P[i][2]*H[2];
    float S = H[0]*PHt[0] + H[1]*PHt[1] + H[2]*PHt[2] + f->heading_noise * f->heading_noise;
    float K[N], dx[N];
    for (int i = 0; i < N; i++) { K[i] = PHt[i] / S; dx[i] = K[i] * z; }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) f->P[i][j] -= K[i] * PHt[j];
    sym(f->P);
    eskf_inject(f, dx);
    f->mag_used = true;
}

void eskf_gyro_clipped(eskf_t *f)
{
    /* Adding to the diagonal keeps P a valid covariance. 2 deg per clipped 2 ms sample is about
     * 1000 deg/s of unknown extra rate; the cap keeps a long clip from making P absurd. */
    const float add = (2.0f / RAD2DEG) * (2.0f / RAD2DEG), cap = (30.0f / RAD2DEG) * (30.0f / RAD2DEG);
    for (int i = 0; i < 3; i++) {
        if (f->P[i][i] < cap) f->P[i][i] += add;
    }
}

void eskf_sigma_deg(const eskf_t *f, float out[3])
{
    for (int i = 0; i < 3; i++) out[i] = sqrtf(fmaxf(f->P[i][i], 0)) * RAD2DEG;
}
