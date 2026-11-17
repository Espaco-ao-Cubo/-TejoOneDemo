/*
 * ICM-20600 (accel + gyro) and AK09918 (magnetometer) on one I2C bus.
 *
 * Wiring (ESP32 DevKit):
 *   IMU VCC -> 3V3,  GND -> GND,  SDA -> GPIO21,  SCL -> GPIO22
 */
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "imu_sensors.h"

static const char *TAG = "sensors";

#define I2C_SDA_GPIO        21
#define I2C_SCL_GPIO        22
#define I2C_FREQ_HZ         400000
#define I2C_TIMEOUT_MS      20

/* ICM-20600 */
#define ICM_ADDR_LOW        0x68
#define ICM_ADDR_HIGH       0x69
#define ICM_WHO_AM_I_VAL    0x11
#define ICM_SMPLRT_DIV      0x19
#define ICM_CONFIG          0x1A
#define ICM_GYRO_CONFIG     0x1B
#define ICM_ACCEL_CONFIG    0x1C
#define ICM_ACCEL_CONFIG2   0x1D
#define ICM_ACCEL_XOUT_H    0x3B
#define ICM_PWR_MGMT_1      0x6B
#define ICM_PWR_MGMT_2      0x6C
#define ICM_WHO_AM_I        0x75
#define ACCEL_LSB_PER_G     8192.0f     /* +-4 g    */
#define GYRO_LSB_PER_DPS    16.4f       /* +-2000 dps: a quick flick of the wrist passes 500 dps */
#define CLIP_RAW            32000       /* |raw| at or above this counts as clipped (full scale is 32767) */

/* AK09918 */
#define AK_ADDR             0x0C
#define AK_WIA1             0x00
#define AK_ST1              0x10
#define AK_CNTL2            0x31
#define AK_CNTL3            0x32
#define AK_MODE_CONT_100HZ  0x08
#define AK_UT_PER_LSB       0.15f

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_icm;
static i2c_master_dev_handle_t s_ak;

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t add_device(uint8_t addr, i2c_master_dev_handle_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, out);
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=%d, SCL=%d)...", I2C_SDA_GPIO, I2C_SCL_GPIO);
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  found device at 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGE(TAG, "  no devices found - check wiring/power");
    }
}

static bool icm_init(void)
{
    uint8_t addr;
    if (i2c_master_probe(s_bus, ICM_ADDR_HIGH, 50) == ESP_OK) {
        addr = ICM_ADDR_HIGH;
    } else if (i2c_master_probe(s_bus, ICM_ADDR_LOW, 50) == ESP_OK) {
        addr = ICM_ADDR_LOW;
    } else {
        ESP_LOGE(TAG, "ICM-20600 not found at 0x68 or 0x69");
        return false;
    }
    if (add_device(addr, &s_icm) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    reg_read(s_icm, ICM_WHO_AM_I, &who, 1);
    if (who == ICM_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "ICM-20600 at 0x%02X, WHO_AM_I=0x%02X", addr, who);
    } else {
        ESP_LOGW(TAG, "Device at 0x%02X WHO_AM_I=0x%02X (expected 0x11), continuing anyway", addr, who);
    }

    esp_err_t err = reg_write(s_icm, ICM_PWR_MGMT_1, 0x80);    /* device reset */
    vTaskDelay(pdMS_TO_TICKS(100));
    err |= reg_write(s_icm, ICM_PWR_MGMT_1, 0x01);             /* wake, auto-select best clock */
    err |= reg_write(s_icm, ICM_PWR_MGMT_2, 0x00);             /* all accel + gyro axes on */
    err |= reg_write(s_icm, ICM_CONFIG, 0x02);                 /* gyro DLPF 92 Hz, 1 kHz internal rate */
    err |= reg_write(s_icm, ICM_SMPLRT_DIV, 0);                /* 1 kHz / (1+0) = 1 kHz output */
    err |= reg_write(s_icm, ICM_GYRO_CONFIG, 0x18);            /* +-2000 dps */
    err |= reg_write(s_icm, ICM_ACCEL_CONFIG, 0x08);           /* +-4 g */
    err |= reg_write(s_icm, ICM_ACCEL_CONFIG2, 0x02);          /* accel DLPF ~99 Hz, 1 kHz */
    vTaskDelay(pdMS_TO_TICKS(50));
    return err == ESP_OK;
}

static bool ak_init(void)
{
    if (i2c_master_probe(s_bus, AK_ADDR, 50) != ESP_OK) {
        ESP_LOGE(TAG, "AK09918 not found at 0x0C - running without magnetometer");
        return false;
    }
    if (add_device(AK_ADDR, &s_ak) != ESP_OK) {
        return false;
    }
    uint8_t wia[2] = { 0 };
    reg_read(s_ak, AK_WIA1, wia, 2);
    if (wia[0] == 0x48 && wia[1] == 0x0C) {
        ESP_LOGI(TAG, "AK09918 at 0x0C, WIA=0x%02X 0x%02X", wia[0], wia[1]);
    } else {
        ESP_LOGW(TAG, "AK09918 WIA=0x%02X 0x%02X (expected 0x48 0x0C), continuing anyway", wia[0], wia[1]);
    }
    esp_err_t err = reg_write(s_ak, AK_CNTL3, 0x01);            /* soft reset */
    vTaskDelay(pdMS_TO_TICKS(10));
    err |= reg_write(s_ak, AK_CNTL2, AK_MODE_CONT_100HZ);       /* 100 Hz is the chip's maximum */
    vTaskDelay(pdMS_TO_TICKS(20));
    return err == ESP_OK;
}

void sensors_init(bool *icm_ok, bool *ak_ok)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
    i2c_scan();
    *icm_ok = icm_init();
    *ak_ok = ak_init();
}

static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static inline int16_t le16(const uint8_t *p) { return (int16_t)((p[1] << 8) | p[0]); }

esp_err_t icm_read(icm_sample_t *s)
{
    uint8_t d[14];
    esp_err_t err = reg_read(s_icm, ICM_ACCEL_XOUT_H, d, sizeof(d));
    if (err != ESP_OK) {
        return err;
    }
    s->acc[0] = be16(&d[0]) / ACCEL_LSB_PER_G;
    s->acc[1] = be16(&d[2]) / ACCEL_LSB_PER_G;
    s->acc[2] = be16(&d[4]) / ACCEL_LSB_PER_G;
    s->temp   = be16(&d[6]) / 326.8f + 25.0f;
    s->clipped = 0;
    for (int i = 0; i < 3; i++) {
        int16_t g = be16(&d[8 + 2 * i]), a = be16(&d[2 * i]);
        s->gyro[i] = g / GYRO_LSB_PER_DPS;
        if (g >= CLIP_RAW || g <= -CLIP_RAW) s->clipped |= 1 << i;
        if (a >= CLIP_RAW || a <= -CLIP_RAW) s->clipped |= 1 << (3 + i);
    }
    return ESP_OK;
}

esp_err_t ak_read(float mag_ut[3], bool *fresh)
{
    uint8_t d[9];   /* ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2 (reading ST2 releases the data) */
    *fresh = false;
    esp_err_t err = reg_read(s_ak, AK_ST1, d, sizeof(d));
    if (err != ESP_OK) {
        return err;
    }
    bool ready = d[0] & 0x01;
    bool overflow = d[8] & 0x08;
    if (ready && !overflow) {
        mag_ut[0] = le16(&d[1]) * AK_UT_PER_LSB;
        mag_ut[1] = le16(&d[3]) * AK_UT_PER_LSB;
        mag_ut[2] = le16(&d[5]) * AK_UT_PER_LSB;
        *fresh = true;
    }
    return ESP_OK;
}
