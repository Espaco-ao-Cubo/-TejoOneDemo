/*
 * IMU Lab firmware
 *
 *   core 1, high priority : read ICM-20600 and run Mahony + ESKF at CONFIG_IMU_RATE_HZ
 *                           (500 Hz default, 1000 Hz with external I2C pull-ups),
 *                           read AK09918 at 100 Hz
 *   core 0                : every 10 ms send one 120-byte packet over WebSocket,
 *                           and one "$I" text line over USB (same as before)
 *
 * Wiring: IMU VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22
 * Wi-Fi:  idf.py menuconfig -> IMU Lab
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "imu_sensors.h"
#include "fusion.h"
#include "net.h"
#include "admin.h"
#include "imu_main.h"

static const char *TAG = "imu";

#define IMU_PERIOD_US   (1000000 / CONFIG_IMU_RATE_HZ)  /* sampling + fusion period (menuconfig -> IMU Lab) */
#define OUT_DIV         (CONFIG_IMU_RATE_HZ / 100)      /* output stays at 100 Hz */
#define D2R             0.0174532925f
#define R2D             57.2957795f

/* One packet every 10 ms. Little-endian, read by the lab with a DataView. */
typedef struct __attribute__((packed)) {
    char     magic[2];          /* "IQ" */
    uint8_t  version;           /* 1 */
    uint8_t  flags;             /* bit0 ESKF used accel, bit1 ESKF used mag, bit2 magnetometer present,
                                       bit3 gyro hit full scale, bit4 accel hit full scale (since the last packet) */
    uint32_t t_ms;
    uint16_t loop_hz;           /* measured fusion loop rate */
    uint16_t loop_us_max;       /* slowest loop iteration in the last second */
    float    acc[3];            /* g, mean of the last 10 samples */
    float    gyro[3];           /* deg/s, mean of the last 10 samples, raw */
    float    mag[3];            /* uT, raw chip axes */
    float    temp;              /* deg C */
    float    q_eskf[4];         /* w x y z */
    float    q_mahony[4];
    float    bias_eskf[3];      /* deg/s */
    float    bias_mahony[3];    /* deg/s */
    float    sigma_eskf[3];     /* deg, 1-sigma attitude uncertainty */
} imu_packet_t;
_Static_assert(sizeof(imu_packet_t) == 120, "packet layout must match the lab page");

/* Magnetometer calibration, sent from the lab's Calibrate tab and kept in flash. */
typedef struct {
    float off[3];
    float scale[3];
    int8_t src[3];      /* which raw axis feeds x, y, z */
    int8_t sign[3];
} mag_cal_t;

static mag_cal_t s_cal = { { 0, 0, 0 }, { 1, 1, 1 }, { 0, 1, 2 }, { 1, 1, 1 } };
static portMUX_TYPE s_cal_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_reset_req;

static const mag_cal_t CAL_NONE = { { 0, 0, 0 }, { 1, 1, 1 }, { 0, 1, 2 }, { 1, 1, 1 } };
static bool s_cal_stored;
static volatile uint32_t s_clip_g_total, s_clip_a_total;

static TaskHandle_t s_imu_task;
static TaskHandle_t s_out_task;
static imu_packet_t s_pkt;
static portMUX_TYPE s_pkt_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---------------- calibration storage & commands ---------------- */
static void cal_load(void)
{
    nvs_handle_t h;
    if (nvs_open("imu", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    mag_cal_t c;
    size_t len = sizeof(c);
    if (nvs_get_blob(h, "magcal", &c, &len) == ESP_OK && len == sizeof(c)) {
        s_cal = c;
        s_cal_stored = true;
        ESP_LOGI(TAG, "Loaded magnetometer calibration from flash");
    }
    nvs_close(h);
}

static void cal_save(const mag_cal_t *c)
{
    nvs_handle_t h;
    if (nvs_open("imu", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "magcal", c, sizeof(*c));
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool parse_axis(const char *t, int8_t *src, int8_t *sign)
{
    if (!t || strlen(t) != 2 || (t[0] != '+' && t[0] != '-') || t[1] < 'x' || t[1] > 'z') {
        return false;
    }
    *sign = t[0] == '-' ? -1 : 1;
    *src = t[1] - 'x';
    return true;
}

/* Commands from the lab (WebSocket text frames):
 *   CAL ox oy oz sx sy sz +x +y +z    magnetometer offset (uT), scale, axis map
 *   RESET                             re-initialise both filters from accel + mag */
static void on_command(const char *cmd)
{
    if (strncmp(cmd, "CAL ", 4) == 0) {
        char buf[200];
        strlcpy(buf, cmd + 4, sizeof(buf));
        char *save = NULL, *tok[9];
        int n = 0;
        for (char *t = strtok_r(buf, " ", &save); t && n < 9; t = strtok_r(NULL, " ", &save)) {
            tok[n++] = t;
        }
        mag_cal_t c;
        bool ok = n == 9;
        for (int i = 0; ok && i < 3; i++) {
            c.off[i] = strtof(tok[i], NULL);
            c.scale[i] = strtof(tok[3 + i], NULL);
            ok = parse_axis(tok[6 + i], &c.src[i], &c.sign[i]) && c.scale[i] > 0.2f && c.scale[i] < 5.0f;
        }
        if (!ok) {
            ESP_LOGW(TAG, "Ignored malformed calibration: %s", cmd);
            return;
        }
        taskENTER_CRITICAL(&s_cal_mux);
        s_cal = c;
        taskEXIT_CRITICAL(&s_cal_mux);
        cal_save(&c);
        s_cal_stored = true;
        s_reset_req = true;
        ESP_LOGI(TAG, "New magnetometer calibration saved: offset %.1f %.1f %.1f uT", c.off[0], c.off[1], c.off[2]);
    } else if (strcmp(cmd, "RESET") == 0) {
        s_reset_req = true;
        ESP_LOGI(TAG, "Filters reset");
    } else if (strcmp(cmd, "CALCLEAR") == 0) {
        taskENTER_CRITICAL(&s_cal_mux);
        s_cal = CAL_NONE;
        taskEXIT_CRITICAL(&s_cal_mux);
        nvs_handle_t h;
        if (nvs_open("imu", NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_key(h, "magcal");
            nvs_commit(h);
            nvs_close(h);
        }
        s_cal_stored = false;
        s_reset_req = true;
        ESP_LOGI(TAG, "Magnetometer calibration cleared; using raw magnetometer readings");
    }
}

void imu_get_live(imu_live_t *o)
{
    imu_packet_t p;
    taskENTER_CRITICAL(&s_pkt_mux);
    p = s_pkt;
    taskEXIT_CRITICAL(&s_pkt_mux);
    memset(o, 0, sizeof(*o));
    o->valid = p.magic[0] == 'I';
    o->loop_hz = p.loop_hz;
    o->temp = p.temp;
    float w = p.q_eskf[0], x = p.q_eskf[1], y = p.q_eskf[2], z = p.q_eskf[3];
    o->roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * R2D;
    o->pitch = asinf(fmaxf(-1, fminf(1, 2 * (w * y - z * x)))) * R2D;
    o->yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * R2D;
    memcpy(o->sigma, p.sigma_eskf, sizeof(o->sigma));
    memcpy(o->bias, p.bias_eskf, sizeof(o->bias));
    o->flags = p.flags;
    o->clip_gyro = s_clip_g_total;
    o->clip_accel = s_clip_a_total;
    o->mag_calibrated = s_cal_stored;
}

static void apply_mag_cal(const float raw[3], float out[3])
{
    mag_cal_t c;
    taskENTER_CRITICAL(&s_cal_mux);
    c = s_cal;
    taskEXIT_CRITICAL(&s_cal_mux);
    for (int i = 0; i < 3; i++) {
        out[i] = (c.sign[i] * raw[c.src[i]] - c.off[i]) * c.scale[i];
    }
}

/* ---------------- 1 kHz sampling + fusion (core 1) ---------------- */
static void tick_cb(void *arg)
{
    xTaskNotifyGive(s_imu_task);
}

static void imu_task(void *arg)
{
    s_imu_task = xTaskGetCurrentTaskHandle();
    bool icm_ok, ak_ok;
    sensors_init(&icm_ok, &ak_ok);
    if (!icm_ok) {
        ESP_LOGE(TAG, "No ICM-20600: nothing to fuse. Check wiring and reset.");
        vTaskDelete(NULL);
    }

    esp_timer_handle_t tick;
    const esp_timer_create_args_t targs = { .callback = tick_cb, .name = "imu_tick" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &tick));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick, IMU_PERIOD_US));

    /* Warm-up: average 300 ms of accel and grab a mag sample, so both filters start on the right answer. */
    icm_sample_t s = { 0 };
    float acc0[3] = { 0 }, mag_raw[3] = { 0 }, mag[3] = { 0 };
    bool have_mag = false;
    int got = 0;
    for (int i = 0; i < 300; i++) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (icm_read(&s) == ESP_OK) {
            for (int k = 0; k < 3; k++) acc0[k] += s.acc[k];
            got++;
        }
        bool fresh = false;
        if (ak_ok && i % 10 == 5 && ak_read(mag_raw, &fresh) == ESP_OK && fresh) {
            apply_mag_cal(mag_raw, mag);
            have_mag = true;
        }
    }
    for (int k = 0; k < 3; k++) acc0[k] /= (got ? got : 1);

    float q0[4];
    quat_from_accel_mag(acc0, have_mag ? mag : NULL, q0);
    static mahony_t mah;
    static eskf_t ekf;
    mahony_init(&mah, q0, 1.0f, 0.05f);
    eskf_init(&ekf, q0);
    ESP_LOGI(TAG, "Fusion running at %d Hz (Mahony + ESKF)%s", 1000000 / IMU_PERIOD_US, have_mag ? "" : " without magnetometer");

    float acc_sum[3] = { 0 }, gyro_sum[3] = { 0 };
    int nsum = 0;
    uint32_t n = 0, loops = 0, i2c_errors = 0;
    uint32_t clip_g = 0, clip_a = 0;      /* samples at full scale, reported every 5 s */
    uint8_t clip_flags = 0;               /* clipped axes since the last packet */
    uint32_t us_max = 0, us_max_report = 0;
    uint16_t hz_report = 0;
    int64_t last = esp_timer_get_time(), sec_start = last, prev_end = last;
    struct { int64_t sum, max; uint32_t n; } tm_icm = { 0 }, tm_mag = { 0 }, tm_fuse = { 0 }, tm_wait = { 0 };
    uint32_t report_count = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t t0 = esp_timer_get_time();
        float dt = (t0 - last) * 1e-6f;
        last = t0;
        if (dt < 0.0002f) dt = 0.0002f;
        if (dt > 0.02f) dt = 0.02f;

        esp_err_t rerr = icm_read(&s);
        int64_t t1 = esp_timer_get_time();
        tm_icm.sum += t1 - t0; tm_icm.n++; if (t1 - t0 > tm_icm.max) tm_icm.max = t1 - t0;
        if (rerr != ESP_OK) {
            if (++i2c_errors % 1000 == 1) ESP_LOGW(TAG, "I2C read errors: %lu", (unsigned long)i2c_errors);
            continue;
        }
        n++;
        loops++;

        bool mag_fresh = false;
        if (ak_ok && n % OUT_DIV == OUT_DIV / 2) {     /* mag read half-way between outputs */
            if (ak_read(mag_raw, &mag_fresh) == ESP_OK && mag_fresh) {
                apply_mag_cal(mag_raw, mag);
                have_mag = true;
            }
            int64_t t2 = esp_timer_get_time();
            tm_mag.sum += t2 - t1; tm_mag.n++; if (t2 - t1 > tm_mag.max) tm_mag.max = t2 - t1;
        }
        int64_t tf = esp_timer_get_time();

        if (s_reset_req) {
            s_reset_req = false;
            quat_from_accel_mag(s.acc, have_mag ? mag : NULL, q0);
            mahony_init(&mah, q0, mah.kp, mah.ki);
            eskf_init(&ekf, q0);
        }

        float g[3] = { s.gyro[0] * D2R, s.gyro[1] * D2R, s.gyro[2] * D2R };
        mahony_update(&mah, g, s.acc, have_mag ? mag : NULL, dt);
        eskf_predict(&ekf, g, dt);
        if (s.clipped & 0x07) {
            eskf_gyro_clipped(&ekf);
            clip_g++;
            s_clip_g_total++;
        }
        if (s.clipped & 0x38) {
            clip_a++;
            s_clip_a_total++;
        }
        clip_flags |= s.clipped;
        if (mag_fresh) {
            eskf_update_mag(&ekf, mag);
        }
        for (int k = 0; k < 3; k++) { acc_sum[k] += s.acc[k]; gyro_sum[k] += s.gyro[k]; }
        nsum++;

        if (n % OUT_DIV == 0) {
            float acc_mean[3];
            for (int k = 0; k < 3; k++) acc_mean[k] = acc_sum[k] / nsum;
            eskf_update_accel(&ekf, acc_mean);     /* gravity update at 100 Hz on the 10-sample mean */

            imu_packet_t p = { .magic = { 'I', 'Q' }, .version = 1 };
            p.flags = (ekf.acc_used ? 1 : 0) | (ekf.mag_used ? 2 : 0) | (have_mag ? 4 : 0)
                    | ((clip_flags & 0x07) ? 8 : 0) | ((clip_flags & 0x38) ? 16 : 0);
            clip_flags = 0;
            p.t_ms = (uint32_t)(t0 / 1000);
            p.loop_hz = hz_report;
            p.loop_us_max = (uint16_t)(us_max_report > 65535 ? 65535 : us_max_report);
            for (int k = 0; k < 3; k++) {
                p.acc[k] = acc_mean[k];
                p.gyro[k] = gyro_sum[k] / nsum;
                p.mag[k] = mag_raw[k];
                p.bias_eskf[k] = ekf.bg[k] * R2D;
                p.bias_mahony[k] = -mah.ib[k] * R2D;
            }
            p.temp = s.temp;
            memcpy(p.q_eskf, ekf.q, sizeof(p.q_eskf));
            memcpy(p.q_mahony, mah.q, sizeof(p.q_mahony));
            float sig[3];
            eskf_sigma_deg(&ekf, sig);
            memcpy(p.sigma_eskf, sig, sizeof(sig));

            taskENTER_CRITICAL(&s_pkt_mux);
            s_pkt = p;
            taskEXIT_CRITICAL(&s_pkt_mux);
            xTaskNotifyGive(s_out_task);

            memset(acc_sum, 0, sizeof(acc_sum));
            memset(gyro_sum, 0, sizeof(gyro_sum));
            nsum = 0;
        }

        int64_t te = esp_timer_get_time();
        tm_fuse.sum += te - tf; tm_fuse.n++; if (te - tf > tm_fuse.max) tm_fuse.max = te - tf;
        int64_t wake = t0 - prev_end;   /* time spent waiting for the next tick */
        prev_end = te;
        tm_wait.sum += wake; tm_wait.n++; if (wake > tm_wait.max) tm_wait.max = wake;
        uint32_t used = (uint32_t)(te - t0);
        if (used > us_max) us_max = used;
        if (t0 - sec_start >= 1000000) {
            hz_report = (uint16_t)loops;
            us_max_report = us_max;
            if (++report_count % 5 == 0) {
                ESP_LOGI(TAG, "timing us (avg/max): icm read %lld/%lld  mag read %lld/%lld  fusion %lld/%lld  idle %lld/%lld  | %u Hz",
                         tm_icm.n ? tm_icm.sum / tm_icm.n : 0, tm_icm.max, tm_mag.n ? tm_mag.sum / tm_mag.n : 0, tm_mag.max,
                         tm_fuse.n ? tm_fuse.sum / tm_fuse.n : 0, tm_fuse.max, tm_wait.n ? tm_wait.sum / tm_wait.n : 0, tm_wait.max,
                         (unsigned)loops);
                if (clip_g || clip_a) {
                    ESP_LOGW(TAG, "full scale reached in the last 5 s: gyro %lu samples, accel %lu samples",
                             (unsigned long)clip_g, (unsigned long)clip_a);
                    clip_g = clip_a = 0;
                }
            }
            memset(&tm_icm, 0, sizeof(tm_icm)); memset(&tm_mag, 0, sizeof(tm_mag));
            memset(&tm_fuse, 0, sizeof(tm_fuse)); memset(&tm_wait, 0, sizeof(tm_wait));
            loops = 0;
            us_max = 0;
            sec_start = t0;
        }
    }
}

/* ---------------- output (core 0) ---------------- */
static void out_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_packet_t p;
        taskENTER_CRITICAL(&s_pkt_mux);
        p = s_pkt;
        taskEXIT_CRITICAL(&s_pkt_mux);

        net_publish(&p, sizeof(p));

        /* Same USB line as before, so the lab's USB mode keeps working. */
        printf("$I,%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\n",
               (unsigned long)p.t_ms, p.acc[0], p.acc[1], p.acc[2], p.gyro[0], p.gyro[1], p.gyro[2],
               p.mag[0], p.mag[1], p.mag[2], p.temp);
    }
}

void app_main(void)
{
    admin_log_init();       /* keep the last few KB of log for the admin page (no USB needed) */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    cal_load();

    /* Interrupt-driven console: printf returns immediately instead of waiting ~7 ms for the UART. */
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 4096, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    xTaskCreatePinnedToCore(out_task, "imu_out", 4096, NULL, 5, &s_out_task, 0);
    xTaskCreatePinnedToCore(imu_task, "imu_fusion", 6144, NULL, configMAX_PRIORITIES - 3, NULL, 1);
    net_start(on_command);
}
