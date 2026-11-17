#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "wifi_mgr.h"

static const char *TAG = "wifi";

#define RETRIES_PER_NETWORK 3
#define RESCAN_S            30      /* look again this often while nobody is on the access point */
/* Searching hops the radio across channels, which silences the IMU-Lab access point for ~2 s
 * each time; phones and laptops with a weak signal then drop off it. So automatic searches
 * pause while any device is connected to IMU-Lab, and resume when the last one leaves. */

typedef struct { char ssid[33]; char pass[65]; } known_t;
typedef struct { uint8_t n; known_t k[WM_MAX_KNOWN]; } known_store_t;
typedef struct { uint8_t head, n; wm_event_t ev[WM_HIST]; } hist_store_t;

static known_store_t s_known;
static hist_store_t s_hist;
static uint32_t s_boot;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static char s_target[33], s_target_pass[65];
static volatile wm_state_t s_state = WM_OFF;
static volatile uint8_t s_reason;
static volatile uint32_t s_ip, s_gw;
static volatile uint8_t s_channel;
static int s_retries;
static bool s_auto_scan, s_logged_none, s_logged_pause, s_sntp_started, s_manual;
static esp_timer_handle_t s_timer;

/* ---------------- flash ---------------- */
static void save_blob(const char *key, const void *p, size_t n)
{
    nvs_handle_t h;
    if (nvs_open("imu", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, key, p, n);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void load_all(void)
{
    nvs_handle_t h;
    bool have_known = false;
    if (nvs_open("imu", NVS_READWRITE, &h) == ESP_OK) {
        size_t n = sizeof(s_known);
        have_known = nvs_get_blob(h, "wifi_known", &s_known, &n) == ESP_OK && n == sizeof(s_known);
        n = sizeof(s_hist);
        if (nvs_get_blob(h, "wifi_hist", &s_hist, &n) != ESP_OK || n != sizeof(s_hist)) memset(&s_hist, 0, sizeof(s_hist));
        if (nvs_get_u32(h, "boots", &s_boot) != ESP_OK) s_boot = 0;
        s_boot++;
        nvs_set_u32(h, "boots", s_boot);
        nvs_commit(h);
        nvs_close(h);
    }
    if (!have_known) {
        memset(&s_known, 0, sizeof(s_known));
        if (strlen(CONFIG_IMU_WIFI_SSID)) {            /* first start: take the menuconfig network */
            strlcpy(s_known.k[0].ssid, CONFIG_IMU_WIFI_SSID, sizeof(s_known.k[0].ssid));
            strlcpy(s_known.k[0].pass, CONFIG_IMU_WIFI_PASSWORD, sizeof(s_known.k[0].pass));
            s_known.n = 1;
        }
    }
    if (s_known.n > WM_MAX_KNOWN) s_known.n = 0;
    if (s_hist.n > WM_HIST || s_hist.head >= WM_HIST) memset(&s_hist, 0, sizeof(s_hist));
}

/* ---------------- history ---------------- */
static uint32_t uptime_s(void) { return xTaskGetTickCount() / configTICK_RATE_HZ; }
bool wm_time_known(void) { return time(NULL) > 1700000000; }

static void hist_add(wm_event_kind_t kind, const char *ssid, uint8_t reason, int8_t rssi)
{
    int64_t now = wm_time_known() ? (int64_t)time(NULL) : 0;
    taskENTER_CRITICAL(&s_mux);
    wm_event_t *last = s_hist.n ? &s_hist.ev[(s_hist.head + WM_HIST - 1) % WM_HIST] : NULL;
    if (last && last->kind == kind && kind != WM_EV_CONNECTED && last->boot == s_boot && strcmp(last->ssid, ssid) == 0) {
        last->count++;                          /* same failure again: just count it */
        last->uptime_s = uptime_s();
        last->unix_s = now;
        last->reason = reason;
    } else {
        wm_event_t *e = &s_hist.ev[s_hist.head];
        memset(e, 0, sizeof(*e));
        e->boot = s_boot;
        e->uptime_s = uptime_s();
        e->unix_s = now;
        strlcpy(e->ssid, ssid, sizeof(e->ssid));
        e->ip = s_ip;
        e->gw = s_gw;
        e->rssi = rssi;
        e->channel = s_channel;
        e->kind = kind;
        e->reason = reason;
        e->count = 1;
        s_hist.head = (s_hist.head + 1) % WM_HIST;
        if (s_hist.n < WM_HIST) s_hist.n++;
    }
    taskEXIT_CRITICAL(&s_mux);
    save_blob("wifi_hist", &s_hist, sizeof(s_hist));
}

int wm_history(wm_event_t *out, int max)
{
    taskENTER_CRITICAL(&s_mux);
    int n = s_hist.n < max ? s_hist.n : max;
    for (int i = 0; i < n; i++) out[i] = s_hist.ev[(s_hist.head + WM_HIST - 1 - i) % WM_HIST];
    taskEXIT_CRITICAL(&s_mux);
    return n;
}

uint32_t wm_boot_count(void) { return s_boot; }

/* ---------------- connecting ---------------- */
static void schedule(int seconds)
{
    esp_timer_stop(s_timer);
    esp_timer_start_once(s_timer, (int64_t)seconds * 1000000);
}

static bool ap_in_use(void)
{
    wifi_sta_list_t l;
    return esp_wifi_ap_get_sta_list(&l) == ESP_OK && l.num > 0;
}

/* Look again later, unless someone is on IMU-Lab: then wait until they leave. */
static void search_later(int seconds)
{
    if (ap_in_use()) {
        if (!s_logged_pause) {
            ESP_LOGI(TAG, "Not searching while devices are on IMU-Lab (it would interrupt them). "
                          "Use \"Look for saved networks now\" on the admin page.");
            s_logged_pause = true;
        }
        return;
    }
    schedule(seconds);
}

static void connect_to(const char *ssid, const char *pass, uint8_t channel)
{
    strlcpy(s_target, ssid, sizeof(s_target));
    strlcpy(s_target_pass, pass, sizeof(s_target_pass));
    s_retries = 0;
    s_ip = s_gw = 0;
    s_state = WM_CONNECTING;
    wifi_config_t st = { 0 };
    strlcpy((char *)st.sta.ssid, s_target, sizeof(st.sta.ssid));
    strlcpy((char *)st.sta.password, s_target_pass, sizeof(st.sta.password));
    st.sta.channel = channel;               /* known from the scan: no need to sweep every channel */
    esp_wifi_set_config(WIFI_IF_STA, &st);
    ESP_LOGI(TAG, "Joining \"%s\"", s_target);
    esp_wifi_connect();
}

static void search(void)
{
    if (s_state == WM_CONNECTED || s_state == WM_CONNECTING) return;
    if (s_known.n == 0) {
        s_state = WM_OFF;
        return;
    }
    s_state = WM_SEARCHING;
    s_auto_scan = true;
    wifi_scan_config_t sc = { .show_hidden = false };
    if (esp_wifi_scan_start(&sc, false) != ESP_OK) {
        s_auto_scan = false;
        schedule(5);
    }
}

static void timer_cb(void *arg)
{
    if (!s_manual && ap_in_use()) return;      /* someone joined IMU-Lab meanwhile: wait for them to leave */
    s_manual = false;
    search();
}

static void pick_from_scan(void)
{
    uint16_t n = 24;
    wifi_ap_record_t *r = calloc(n, sizeof(*r));
    if (!r) { esp_wifi_clear_ap_list(); schedule(10); return; }
    if (esp_wifi_scan_get_ap_records(&n, r) != ESP_OK) n = 0;
    int best = -1, best_rssi = -1000;
    uint8_t best_ch = 0;
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < s_known.n; k++) {
            if (strcmp((const char *)r[i].ssid, s_known.k[k].ssid) == 0 && r[i].rssi > best_rssi) {
                best = k;
                best_rssi = r[i].rssi;
                best_ch = r[i].primary;
            }
        }
    }
    free(r);
    if (best >= 0) {
        s_logged_none = s_logged_pause = false;
        ESP_LOGI(TAG, "Saved network \"%s\" in range (%d dBm, channel %d)", s_known.k[best].ssid, best_rssi, best_ch);
        connect_to(s_known.k[best].ssid, s_known.k[best].pass, best_ch);
    } else {
        s_state = WM_NOT_FOUND;
        if (!s_logged_none) {
            ESP_LOGI(TAG, "None of the %d saved networks is in range. The IMU-Lab access point stays on: "
                          "http://192.168.4.1/", s_known.n);
            s_logged_none = true;
        }
        search_later(RESCAN_S);
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        search();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        if ((s_state == WM_NOT_FOUND || s_state == WM_SEARCHING) && !ap_in_use()) {
            s_logged_pause = false;
            schedule(5);                            /* last device left IMU-Lab: resume searching */
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE) {
        if (s_auto_scan) {                          /* the admin page's scans read their own results */
            s_auto_scan = false;
            pick_from_scan();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        if (!s_target[0]) return;
        /* Leaving a previous network after a switch: the new one is already being joined. */
        if (e->ssid_len && (e->ssid_len != strlen(s_target) || memcmp(e->ssid, s_target, e->ssid_len) != 0)) return;
        bool was_connected = s_state == WM_CONNECTED;
        s_reason = e->reason;
        if (was_connected) {
            ESP_LOGW(TAG, "Lost \"%s\" (reason %d)", s_target, e->reason);
            hist_add(WM_EV_LOST, s_target, e->reason, e->rssi);
            s_retries = 0;
        }
        s_ip = s_gw = 0;
        if (++s_retries <= RETRIES_PER_NETWORK) {
            s_state = WM_CONNECTING;
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "Could not join \"%s\" (reason %d).", s_target, e->reason);
            hist_add(WM_EV_FAILED, s_target, e->reason, e->rssi);
            s_state = WM_NOT_FOUND;
            search_later(RESCAN_S);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        wifi_ap_record_t ap = { 0 };
        esp_wifi_sta_get_ap_info(&ap);
        s_ip = e->ip_info.ip.addr;
        s_gw = e->ip_info.gw.addr;
        s_channel = ap.primary;
        s_reason = 0;
        s_retries = 0;
        s_state = WM_CONNECTED;
        ESP_LOGI(TAG, "Joined \"%s\": IP " IPSTR ", gateway " IPSTR ", %d dBm, channel %d. "
                      "Admin page: http://" IPSTR "/ or http://%s.local/",
                 s_target, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw), ap.rssi, ap.primary,
                 IP2STR(&e->ip_info.ip), CONFIG_IMU_HOSTNAME);
        hist_add(WM_EV_CONNECTED, s_target, 0, ap.rssi);
        if (!s_sntp_started) {                      /* internet time, for dated history entries */
            esp_sntp_config_t c = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
            s_sntp_started = esp_netif_sntp_init(&c) == ESP_OK;
        }
    }
}

void wm_init(void)
{
    load_all();
    const esp_timer_create_args_t a = { .callback = timer_cb, .name = "wifi_search" };
    ESP_ERROR_CHECK(esp_timer_create(&a, &s_timer));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));
    ESP_LOGI(TAG, "Boot #%lu. %d saved network(s).", (unsigned long)s_boot, s_known.n);
    for (int i = 0; i < s_known.n; i++) ESP_LOGI(TAG, "  saved: \"%s\"", s_known.k[i].ssid);
    wm_event_t last[1];
    if (wm_history(last, 1)) {
        esp_ip4_addr_t ip = { .addr = last[0].ip };
        ESP_LOGI(TAG, "Last Wi-Fi event (boot #%lu): %s \"%s\"%s" IPSTR,
                 (unsigned long)last[0].boot,
                 last[0].kind == WM_EV_CONNECTED ? "joined" : last[0].kind == WM_EV_LOST ? "lost" : "failed to join",
                 last[0].ssid, last[0].kind == WM_EV_CONNECTED ? " with IP " : " ", IP2STR(&ip));
    }
}

/* ---------------- API for the admin page ---------------- */
void wm_get_status(wm_status_t *s)
{
    memset(s, 0, sizeof(*s));
    s->state = s_state;
    strlcpy(s->ssid, s_target, sizeof(s->ssid));
    s->ip = s_ip;
    s->gw = s_gw;
    s->channel = s_channel;
    s->reason = s_reason;
    wifi_ap_record_t ap;
    if (s_state == WM_CONNECTED && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) s->rssi = ap.rssi;
}

int wm_known(char ssids[][33], int max)
{
    int n = s_known.n < max ? s_known.n : max;
    for (int i = 0; i < n; i++) strlcpy(ssids[i], s_known.k[i].ssid, 33);
    return n;
}

esp_err_t wm_join(const char *ssid, const char *pass)
{
    if (!ssid || !ssid[0] || strlen(ssid) > 32 || strlen(pass) > 64) return ESP_ERR_INVALID_ARG;
    /* Move (or add) it to the front of the saved list; the oldest one drops off when full. */
    known_t k = { 0 };
    strlcpy(k.ssid, ssid, sizeof(k.ssid));
    strlcpy(k.pass, pass, sizeof(k.pass));
    int at = -1;
    for (int i = 0; i < s_known.n; i++) if (strcmp(s_known.k[i].ssid, ssid) == 0) at = i;
    int last = at >= 0 ? at : (s_known.n < WM_MAX_KNOWN ? s_known.n++ : WM_MAX_KNOWN - 1);
    memmove(&s_known.k[1], &s_known.k[0], last * sizeof(known_t));
    s_known.k[0] = k;
    save_blob("wifi_known", &s_known, sizeof(s_known));

    esp_timer_stop(s_timer);
    s_auto_scan = false;
    esp_wifi_scan_stop();
    s_state = WM_CONNECTING;
    esp_wifi_disconnect();
    connect_to(ssid, pass, 0);
    return ESP_OK;
}

esp_err_t wm_forget(const char *ssid)
{
    int at = -1;
    for (int i = 0; i < s_known.n; i++) if (strcmp(s_known.k[i].ssid, ssid) == 0) at = i;
    if (at < 0) return ESP_ERR_NOT_FOUND;
    memmove(&s_known.k[at], &s_known.k[at + 1], (s_known.n - at - 1) * sizeof(known_t));
    s_known.n--;
    memset(&s_known.k[s_known.n], 0, sizeof(known_t));
    save_blob("wifi_known", &s_known, sizeof(s_known));
    ESP_LOGI(TAG, "Forgot \"%s\"", ssid);
    if (strcmp(s_target, ssid) == 0) {
        s_target[0] = 0;
        s_ip = s_gw = 0;
        s_state = WM_OFF;
        esp_wifi_disconnect();
        schedule(2);                                /* look for the other saved networks */
    }
    return ESP_OK;
}

void wm_search_now(void)
{
    if (s_state == WM_SEARCHING || s_state == WM_NOT_FOUND || s_state == WM_OFF) {
        s_state = WM_OFF;                           /* lets search() run */
        s_manual = true;
        esp_timer_stop(s_timer);
        esp_timer_start_once(s_timer, 10 * 1000);
    }
}

void wm_clear_history(void)
{
    taskENTER_CRITICAL(&s_mux);
    memset(&s_hist, 0, sizeof(s_hist));
    taskEXIT_CRITICAL(&s_mux);
    save_blob("wifi_hist", &s_hist, sizeof(s_hist));
}

esp_err_t wm_scan(wifi_ap_record_t *out, uint16_t *n)
{
    /* Scanning hops the radio across channels, so streaming pauses for ~2 s. */
    wifi_scan_config_t sc = { .show_hidden = false };
    esp_err_t err = esp_wifi_scan_start(&sc, true);
    if (err != ESP_OK) {
        *n = 0;
        return err;
    }
    return esp_wifi_scan_get_ap_records(n, out);
}
