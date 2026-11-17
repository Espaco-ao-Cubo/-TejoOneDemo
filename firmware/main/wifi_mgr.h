#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi_types.h"

/* Station side of Wi-Fi: joins one of up to WM_MAX_KNOWN saved networks (the strongest one in
 * range), keeps looking while none is in range, and keeps a history of what happened in flash,
 * so you can find out later which network the board joined and which IP it got.
 * The IMU-Lab access point is separate and always on. */

#define WM_MAX_KNOWN 5
#define WM_HIST      12

typedef enum { WM_OFF, WM_SEARCHING, WM_CONNECTING, WM_CONNECTED, WM_NOT_FOUND } wm_state_t;
typedef enum { WM_EV_CONNECTED = 1, WM_EV_FAILED, WM_EV_LOST } wm_event_kind_t;

typedef struct {
    uint32_t boot;          /* boot number (counts up at every power-up or restart) */
    uint32_t uptime_s;      /* seconds since that boot */
    int64_t  unix_s;        /* wall-clock time if it was known (internet time), else 0 */
    char     ssid[33];
    uint32_t ip, gw;        /* lwIP byte order */
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  kind;          /* wm_event_kind_t */
    uint8_t  reason;        /* Wi-Fi reason code for FAILED / LOST */
    uint16_t count;         /* identical events in a row are merged */
} wm_event_t;

typedef struct {
    wm_state_t state;
    char     ssid[33];      /* network joined or being joined */
    uint32_t ip, gw;
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  reason;        /* last failure reason, 0 = none */
} wm_status_t;

/* Call after esp_wifi_init() and before esp_wifi_start(). */
void wm_init(void);

void wm_get_status(wm_status_t *s);
int wm_known(char ssids[][33], int max);
int wm_history(wm_event_t *out, int max);       /* newest first */
uint32_t wm_boot_count(void);
bool wm_time_known(void);

esp_err_t wm_join(const char *ssid, const char *pass);   /* saves it and joins now */
esp_err_t wm_forget(const char *ssid);
void wm_search_now(void);
void wm_clear_history(void);
esp_err_t wm_scan(wifi_ap_record_t *out, uint16_t *n);   /* blocking, ~2 s */
