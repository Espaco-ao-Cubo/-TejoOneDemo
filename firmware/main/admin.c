/* Admin page served by the ESP32 itself: open http://192.168.4.1/ (or http://imu-lab.local/
 * on your home network) in any browser.
 *
 *   GET  /                 the page (admin.html, compiled into the firmware)
 *   GET  /api/status       firmware, IMU, Wi-Fi, connected devices (JSON)
 *   GET  /api/scan         nearby Wi-Fi networks (JSON, takes ~2 s)
 *   POST /api/wifi         ssid=...&pass=...   join a network and remember it
 *   GET  /api/wifi         saved networks + connection history (which network, which IP)
 *   POST /api/wifi/forget  ssid=...   forget a saved network
 *   POST /api/wifi/search  look for saved networks now
 *   POST /api/cmd          cmd=reset | restart | clearcal
 *   GET  /api/log          the last few KB of the ESP32 log, so you don't need USB
 *
 * POST requests need the admin password (menuconfig -> IMU Lab) if one is set.
 */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "admin.h"
#include "ota.h"
#include "net.h"
#include "imu_main.h"
#include "wifi_mgr.h"
#include <time.h>

static const char *TAG = "admin";
extern const char admin_html_start[] asm("_binary_admin_html_start");
extern const char admin_html_end[] asm("_binary_admin_html_end");
static net_command_cb_t s_cmd;

/* ---------------- log ring buffer ---------------- */
#define LOG_RING 6144
static char s_ring[LOG_RING];
static size_t s_head;           /* next write position */
static bool s_wrapped;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_orig_vprintf;

static int log_vprintf(const char *fmt, va_list ap)
{
    char line[200];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(line, sizeof(line), fmt, ap2);
    va_end(ap2);
    if (n > 0) {
        if (n >= (int)sizeof(line)) {
            n = sizeof(line) - 1;
            line[n - 1] = '\n';
        }
        taskENTER_CRITICAL(&s_log_mux);
        for (int i = 0; i < n; i++) {
            s_ring[s_head++] = line[i];
            if (s_head == LOG_RING) { s_head = 0; s_wrapped = true; }
        }
        taskEXIT_CRITICAL(&s_log_mux);
    }
    return s_orig_vprintf ? s_orig_vprintf(fmt, ap) : vprintf(fmt, ap);
}

void admin_log_init(void)
{
    s_orig_vprintf = esp_log_set_vprintf(log_vprintf);
}

/* ---------------- helpers ---------------- */
static esp_err_t send_json(httpd_req_t *req, const char *body)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t send_err(httpd_req_t *req, const char *status, const char *msg)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, msg);
}

/* Append s to a JSON string, escaped. */
static size_t json_str(char *out, size_t cap, const char *s)
{
    size_t o = 0;
    if (o < cap) out[o++] = '"';
    for (; *s && o + 7 < cap; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
        else if (c < 0x20) o += snprintf(out + o, cap - o, "\\u%04x", c);
        else out[o++] = c;
    }
    if (o < cap) out[o++] = '"';
    if (o < cap) out[o] = 0;
    return o;
}

/* Read a small form body (application/x-www-form-urlencoded). */
static bool read_body(httpd_req_t *req, char *buf, size_t cap)
{
    if (req->content_len >= cap) return false;
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r <= 0) return false;
        got += r;
    }
    buf[got] = 0;
    return true;
}

static void url_decode(char *s)
{
    char *o = s;
    for (; *s; s++) {
        if (*s == '+') *o++ = ' ';
        else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
            char hex[3] = { s[1], s[2], 0 };
            *o++ = (char)strtol(hex, NULL, 16);
            s += 2;
        } else *o++ = *s;
    }
    *o = 0;
}

static bool form_get(const char *body, const char *key, char *out, size_t cap)
{
    if (httpd_query_key_value(body, key, out, cap) != ESP_OK) return false;
    url_decode(out);
    return true;
}

static void restart_cb(void *arg) { esp_restart(); }

static void restart_soon(void)
{
    static esp_timer_handle_t t;
    const esp_timer_create_args_t a = { .callback = restart_cb, .name = "admin_restart" };
    if (!t) esp_timer_create(&a, &t);
    esp_timer_start_once(t, 700 * 1000);
}

/* ---------------- handlers ---------------- */
static esp_err_t favicon_get(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");     /* browsers ask for it on every visit */
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, admin_html_start, admin_html_end - admin_html_start - 1);
}

static const char *sta_state_name(wm_state_t s)
{
    switch (s) {
    case WM_SEARCHING: return "searching";
    case WM_CONNECTING: return "connecting";
    case WM_CONNECTED: return "connected";
    case WM_NOT_FOUND: return "not_found";
    default: return "off";
    }
}

static void ip_str(uint32_t a, char out[16])
{
    out[0] = 0;
    if (a) {
        esp_ip4_addr_t x = { .addr = a };
        snprintf(out, 16, IPSTR, IP2STR(&x));
    }
}

static esp_err_t status_get(httpd_req_t *req)
{
    const size_t cap = 2048;
    char *b = malloc(cap);
    if (!b) return send_err(req, "500 Internal Server Error", "out of memory");
    const esp_app_desc_t *d = esp_app_get_description();
    imu_live_t im;
    imu_get_live(&im);
    net_status_t ns;
    net_get_status(&ns);
    wm_status_t ws;
    wm_get_status(&ws);

    size_t o = 0;
    o += snprintf(b + o, cap - o,
                  "{\"fw\":{\"version\":\"%s\",\"build\":\"%02x%02x%02x%02x%02x%02x%02x%02x\",\"slot\":\"%s\",\"state\":\"%s\",\"idf\":\"%s\"},"
                  "\"uptime_s\":%lu,\"heap_free\":%u,\"heap_min\":%u,",
                  d->version, d->app_elf_sha256[0], d->app_elf_sha256[1], d->app_elf_sha256[2], d->app_elf_sha256[3],
                  d->app_elf_sha256[4], d->app_elf_sha256[5], d->app_elf_sha256[6], d->app_elf_sha256[7],
                  esp_ota_get_running_partition()->label, ota_state_str(), d->idf_ver,
                  (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ),
                  (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    o += snprintf(b + o, cap - o,
                  "\"imu\":{\"valid\":%s,\"hz\":%u,\"temp\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
                  "\"sigma\":[%.3f,%.3f,%.3f],\"bias\":[%.4f,%.4f,%.4f],\"gravity_used\":%s,\"heading_used\":%s,"
                  "\"mag_present\":%s,\"mag_calibrated\":%s,\"clip_gyro\":%lu,\"clip_accel\":%lu},",
                  im.valid ? "true" : "false", im.loop_hz, im.temp, im.roll, im.pitch, im.yaw,
                  im.sigma[0], im.sigma[1], im.sigma[2], im.bias[0], im.bias[1], im.bias[2],
                  (im.flags & 1) ? "true" : "false", (im.flags & 2) ? "true" : "false",
                  (im.flags & 4) ? "true" : "false", im.mag_calibrated ? "true" : "false",
                  (unsigned long)im.clip_gyro, (unsigned long)im.clip_accel);
    o += snprintf(b + o, cap - o, "\"ap\":{\"ssid\":");
    o += json_str(b + o, cap - o, CONFIG_IMU_AP_SSID);
    o += snprintf(b + o, cap - o, ",\"ip\":\"192.168.4.1\",\"channel\":%u,\"streams\":%u,\"clients\":[",
                  ns.channel, ns.ws_clients);
    for (int i = 0; i < ns.ap_clients; i++) {
        const uint8_t *m = ns.ap_mac[i];
        o += snprintf(b + o, cap - o, "%s{\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"rssi\":%d}",
                      i ? "," : "", m[0], m[1], m[2], m[3], m[4], m[5], ns.ap_rssi[i]);
    }
    o += snprintf(b + o, cap - o, "]},\"sta\":{\"state\":\"%s\",\"ssid\":", sta_state_name(ws.state));
    o += json_str(b + o, cap - o, ws.ssid);
    char ip[16], gw[16];
    ip_str(ws.ip, ip);
    ip_str(ws.gw, gw);
    o += snprintf(b + o, cap - o, ",\"ip\":\"%s\",\"gateway\":\"%s\",\"rssi\":%d,\"channel\":%u,\"reason\":%u,"
                  "\"hostname\":\"%s.local\"},\"admin_password\":%s}",
                  ip, gw, ws.rssi, ws.channel, ws.reason, CONFIG_IMU_HOSTNAME,
                  strlen(CONFIG_IMU_OTA_PASSWORD) ? "true" : "false");
    esp_err_t err = send_json(req, b);
    free(b);
    return err;
}

static esp_err_t scan_get(httpd_req_t *req)
{
    uint16_t n = 20;
    wifi_ap_record_t *recs = calloc(n, sizeof(*recs));
    char *b = malloc(2048);
    if (!recs || !b) {
        free(recs);
        free(b);
        return send_err(req, "500 Internal Server Error", "out of memory");
    }
    esp_err_t err = wm_scan(recs, &n);
    if (err != ESP_OK) {
        free(recs);
        free(b);
        return send_err(req, "503 Service Unavailable",
                        "Scan not possible right now (the ESP32 is busy joining a network). Try again in a few seconds.");
    }
    size_t o = 0, cap = 2048;
    o += snprintf(b, cap, "[");
    for (int i = 0; i < n && o + 120 < cap; i++) {
        if (!recs[i].ssid[0]) continue;
        o += snprintf(b + o, cap - o, "%s{\"ssid\":", o > 1 ? "," : "");
        o += json_str(b + o, cap - o, (const char *)recs[i].ssid);
        o += snprintf(b + o, cap - o, ",\"rssi\":%d,\"channel\":%u,\"secure\":%s}",
                      recs[i].rssi, recs[i].primary, recs[i].authmode == WIFI_AUTH_OPEN ? "false" : "true");
    }
    snprintf(b + o, cap - o, "]");
    err = send_json(req, b);
    free(recs);
    free(b);
    return err;
}

static esp_err_t wifi_post(httpd_req_t *req)
{
    if (!ota_auth_ok(req)) return send_err(req, "403 Forbidden", "Wrong or missing admin password");
    char body[256], ssid[64] = "", pass[128] = "";
    if (!read_body(req, body, sizeof(body)) || !form_get(body, "ssid", ssid, sizeof(ssid))) {
        return send_err(req, "400 Bad Request", "Missing network name");
    }
    form_get(body, "pass", pass, sizeof(pass));
    size_t pl = strlen(pass);
    if (!ssid[0] || strlen(ssid) > 32) return send_err(req, "400 Bad Request", "Network name must be 1-32 characters");
    if (pl && (pl < 8 || pl > 63)) return send_err(req, "400 Bad Request", "Wi-Fi passwords are 8-63 characters (or empty for an open network)");
    send_json(req, "{\"ok\":true}");     /* answer first: joining may move the access point's channel */
    wm_join(ssid, pass);
    return ESP_OK;
}

static esp_err_t forget_post(httpd_req_t *req)
{
    if (!ota_auth_ok(req)) return send_err(req, "403 Forbidden", "Wrong or missing admin password");
    char body[128], ssid[64] = "";
    if (!read_body(req, body, sizeof(body)) || !form_get(body, "ssid", ssid, sizeof(ssid))) {
        return send_err(req, "400 Bad Request", "Missing network name");
    }
    if (wm_forget(ssid) != ESP_OK) return send_err(req, "404 Not Found", "That network isn't saved");
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t search_post(httpd_req_t *req)
{
    if (!ota_auth_ok(req)) return send_err(req, "403 Forbidden", "Wrong or missing admin password");
    wm_search_now();
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t clearhist_post(httpd_req_t *req)
{
    if (!ota_auth_ok(req)) return send_err(req, "403 Forbidden", "Wrong or missing admin password");
    wm_clear_history();
    return send_json(req, "{\"ok\":true}");
}

/* Saved networks and the connection history (kept in flash across power-offs). */
static esp_err_t wifi_get(httpd_req_t *req)
{
    const size_t cap = 4096;
    char *b = malloc(cap);
    wm_event_t *ev = calloc(WM_HIST, sizeof(wm_event_t));
    char (*known)[33] = calloc(WM_MAX_KNOWN, 33);
    if (!b || !ev || !known) {
        free(b); free(ev); free(known);
        return send_err(req, "500 Internal Server Error", "out of memory");
    }
    int nk = wm_known(known, WM_MAX_KNOWN), ne = wm_history(ev, WM_HIST);
    size_t o = snprintf(b, cap, "{\"boot\":%lu,\"uptime_s\":%lu,\"now\":%lld,\"known\":[",
                        (unsigned long)wm_boot_count(), (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ),
                        wm_time_known() ? (long long)time(NULL) : 0LL);
    for (int i = 0; i < nk; i++) {
        if (i) b[o++] = ',';
        o += json_str(b + o, cap - o, known[i]);
    }
    o += snprintf(b + o, cap - o, "],\"history\":[");
    for (int i = 0; i < ne && o + 200 < cap; i++) {
        static const char *kinds[] = { "?", "connected", "failed", "lost" };
        char ip[16], gw[16];
        ip_str(ev[i].ip, ip);
        ip_str(ev[i].gw, gw);
        o += snprintf(b + o, cap - o, "%s{\"boot\":%lu,\"uptime_s\":%lu,\"time\":%lld,\"kind\":\"%s\",\"ssid\":",
                      i ? "," : "", (unsigned long)ev[i].boot, (unsigned long)ev[i].uptime_s,
                      (long long)ev[i].unix_s, kinds[ev[i].kind <= 3 ? ev[i].kind : 0]);
        o += json_str(b + o, cap - o, ev[i].ssid);
        o += snprintf(b + o, cap - o, ",\"ip\":\"%s\",\"gateway\":\"%s\",\"rssi\":%d,\"channel\":%u,\"reason\":%u,\"count\":%u}",
                      ip, gw, ev[i].rssi, ev[i].channel, ev[i].reason, ev[i].count);
    }
    snprintf(b + o, cap - o, "]}");
    esp_err_t err = send_json(req, b);
    free(b); free(ev); free(known);
    return err;
}

static esp_err_t cmd_post(httpd_req_t *req)
{
    if (!ota_auth_ok(req)) return send_err(req, "403 Forbidden", "Wrong or missing admin password");
    char body[64], cmd[24] = "";
    if (!read_body(req, body, sizeof(body)) || !form_get(body, "cmd", cmd, sizeof(cmd))) {
        return send_err(req, "400 Bad Request", "Missing cmd");
    }
    ESP_LOGI(TAG, "Command from the admin page: %s", cmd);
    if (strcmp(cmd, "reset") == 0) {
        if (s_cmd) s_cmd("RESET");
    } else if (strcmp(cmd, "clearcal") == 0) {
        if (s_cmd) s_cmd("CALCLEAR");
    } else if (strcmp(cmd, "restart") == 0) {
        send_json(req, "{\"ok\":true}");
        restart_soon();
        return ESP_OK;
    } else {
        return send_err(req, "400 Bad Request", "Unknown command");
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t log_get(httpd_req_t *req)
{
    char *b = malloc(LOG_RING + 1);
    if (!b) return send_err(req, "500 Internal Server Error", "out of memory");
    size_t n;
    taskENTER_CRITICAL(&s_log_mux);
    if (s_wrapped) {
        size_t tail = LOG_RING - s_head;
        memcpy(b, s_ring + s_head, tail);
        memcpy(b + tail, s_ring, s_head);
        n = LOG_RING;
    } else {
        memcpy(b, s_ring, s_head);
        n = s_head;
    }
    taskEXIT_CRITICAL(&s_log_mux);
    size_t start = 0;
    if (s_wrapped) {                         /* drop the cut-off first line */
        while (start < n && b[start] != '\n') start++;
        if (start < n) start++;
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, b + start, n - start);
    free(b);
    return err;
}

void admin_register(httpd_handle_t server, net_command_cb_t on_command)
{
    s_cmd = on_command;
    static const httpd_uri_t h[] = {
        { .uri = "/", .method = HTTP_GET, .handler = page_get },
        { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get },
        { .uri = "/api/status", .method = HTTP_GET, .handler = status_get },
        { .uri = "/api/scan", .method = HTTP_GET, .handler = scan_get },
        { .uri = "/api/wifi", .method = HTTP_POST, .handler = wifi_post },
        { .uri = "/api/wifi", .method = HTTP_GET, .handler = wifi_get },
        { .uri = "/api/wifi/forget", .method = HTTP_POST, .handler = forget_post },
        { .uri = "/api/wifi/search", .method = HTTP_POST, .handler = search_post },
        { .uri = "/api/wifi/clearhist", .method = HTTP_POST, .handler = clearhist_post },
        { .uri = "/api/cmd", .method = HTTP_POST, .handler = cmd_post },
        { .uri = "/api/log", .method = HTTP_GET, .handler = log_get },
    };
    for (size_t i = 0; i < sizeof(h) / sizeof(h[0]); i++) {
        if (httpd_register_uri_handler(server, &h[i]) != ESP_OK) {
            ESP_LOGE(TAG, "Could not register %s", h[i].uri);
        }
    }
}
