#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "net.h"
#include "ota.h"
#include "admin.h"
#include "wifi_mgr.h"

static const char *TAG = "net";

#define MAX_FRAME 256
#define MAX_CLIENTS 4

static httpd_handle_t s_server;
static net_command_cb_t s_on_command;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* WebSocket clients we stream to. Each frame goes out as ONE non-blocking send() so a slow
 * Wi-Fi link drops stale frames instead of stalling. A partly sent frame is finished first,
 * so the WebSocket stream never gets corrupted. */
typedef struct {
    int fd;                         /* -1 = free */
    uint8_t pend[MAX_FRAME + 4];
    size_t pend_len, pend_off;
    uint32_t sent, dropped;
} ws_client_t;
static ws_client_t s_clients[MAX_CLIENTS];

/* ---------------- Wi-Fi ---------------- */
/* The station side (saved networks, history) lives in wifi_mgr.c. Here: the access point. */
static void ap_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "Device " MACSTR " joined the %s access point. Admin page: http://192.168.4.1/",
                 MAC2STR(e->mac), CONFIG_IMU_AP_SSID);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "Device " MACSTR " left the %s access point", MAC2STR(e->mac), CONFIG_IMU_AP_SSID);
    }
}

void net_get_status(net_status_t *s)
{
    memset(s, 0, sizeof(*s));
    uint8_t ch = 0;
    wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    s->channel = ch;
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
        s->ap_clients = list.num > NET_MAX_STATIONS ? NET_MAX_STATIONS : list.num;
        for (int i = 0; i < s->ap_clients; i++) {
            memcpy(s->ap_mac[i], list.sta[i].mac, 6);
            s->ap_rssi[i] = list.sta[i].rssi;
        }
    }
    for (int i = 0; i < MAX_CLIENTS; i++) s->ws_clients += s_clients[i].fd >= 0;
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, CONFIG_IMU_HOSTNAME);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, ap_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, ap_event, NULL));
    wm_init();

    /* Always access point + station. The access point is the way in that always works,
     * wherever the board is; the station joins a saved network when one is in range. */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, CONFIG_IMU_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(CONFIG_IMU_AP_SSID);
    strlcpy((char *)ap.ap.password, CONFIG_IMU_AP_PASSWORD, sizeof(ap.ap.password));
    ap.ap.authmode = strlen(CONFIG_IMU_AP_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap.ap.max_connection = MAX_CLIENTS;
    ap.ap.channel = 6;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    ESP_ERROR_CHECK(esp_wifi_start());
    /* The single most important latency setting: keep the radio awake. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* The password is not logged: the log can be read from the admin page. */
    ESP_LOGI(TAG, "Access point \"%s\" up (%s). Admin page: http://192.168.4.1/",
             CONFIG_IMU_AP_SSID, strlen(CONFIG_IMU_AP_PASSWORD) >= 8 ? "WPA2" : "open");
}

static void mdns_start(void)
{
    if (mdns_init() != ESP_OK) {
        ESP_LOGW(TAG, "mDNS failed to start; use the IP address instead");
        return;
    }
    mdns_hostname_set(CONFIG_IMU_HOSTNAME);
    mdns_instance_name_set("IMU Lab");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

/* ---------------- HTTP + WebSocket ---------------- */
static void client_add(int fd)
{
    taskENTER_CRITICAL(&s_mux);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].fd == fd) { taskEXIT_CRITICAL(&s_mux); return; }
    }
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].fd < 0) {
            s_clients[i].fd = fd;
            s_clients[i].pend_len = s_clients[i].pend_off = 0;
            s_clients[i].sent = s_clients[i].dropped = 0;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_mux);
}

static void client_remove(int fd)
{
    taskENTER_CRITICAL(&s_mux);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].fd == fd) s_clients[i].fd = -1;
    }
    taskEXIT_CRITICAL(&s_mux);
}

static void on_close(httpd_handle_t hd, int fd)
{
    client_remove(fd);
    close(fd);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "Lab connected over WebSocket (socket %d)", fd);
        client_add(fd);
        return ESP_OK;
    }
    httpd_ws_frame_t f = { .type = HTTPD_WS_TYPE_TEXT };
    esp_err_t err = httpd_ws_recv_frame(req, &f, 0);
    if (err != ESP_OK || f.len == 0 || f.len > 200) {
        return err;
    }
    char *buf = calloc(1, f.len + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    f.payload = (uint8_t *)buf;
    err = httpd_ws_recv_frame(req, &f, f.len);
    if (err == ESP_OK && f.type == HTTPD_WS_TYPE_TEXT && s_on_command) {
        s_on_command(buf);
    }
    free(buf);
    return err;
}

/* Every new socket: disable Nagle so small 120-byte frames go out immediately. */
static esp_err_t on_open(httpd_handle_t hd, int fd)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return ESP_OK;
}

/* Returns false if the socket is broken. */
static bool flush_pending(ws_client_t *c)
{
    while (c->pend_off < c->pend_len) {
        int r = send(c->fd, c->pend + c->pend_off, c->pend_len - c->pend_off, MSG_DONTWAIT);
        if (r > 0) {
            c->pend_off += r;
        } else if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return true;    /* link busy: try again next frame */
        } else {
            return false;
        }
    }
    c->pend_len = c->pend_off = 0;
    return true;
}

/* Track WebSocket clients by asking the server for its open sessions. This doesn't rely on the
 * URI handler being called at handshake time, which differs between ESP-IDF versions. */
static void sync_clients(void)
{
    int fds[MAX_CLIENTS + 3];
    size_t n = sizeof(fds) / sizeof(fds[0]);
    if (httpd_get_client_list(s_server, &n, fds) != ESP_OK) {
        return;
    }
    for (size_t i = 0; i < n; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            bool known = false;
            for (int k = 0; k < MAX_CLIENTS; k++) known |= s_clients[k].fd == fds[i];
            if (!known) {
                ESP_LOGI(TAG, "Streaming to WebSocket client on socket %d", fds[i]);
                client_add(fds[i]);
            }
        }
    }
    for (int k = 0; k < MAX_CLIENTS; k++) {
        int fd = s_clients[k].fd;
        if (fd < 0) continue;
        bool still = false;
        for (size_t i = 0; i < n; i++) still |= fds[i] == fd;
        if (!still) client_remove(fd);
    }
}

void net_publish(const void *data, size_t len)
{
    if (!s_server || len > MAX_FRAME || len > 125) {
        return;
    }
    sync_clients();
    static uint32_t s_last_report;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        ws_client_t *c = &s_clients[i];
        int fd = c->fd;
        if (fd < 0) continue;
        bool ok = flush_pending(c);
        if (ok && c->pend_len == 0) {
            /* Unmasked binary frame, payload < 126 bytes: 2-byte header. */
            c->pend[0] = 0x82;
            c->pend[1] = (uint8_t)len;
            memcpy(c->pend + 2, data, len);
            c->pend_len = len + 2;
            c->pend_off = 0;
            ok = flush_pending(c);
            c->sent++;
        } else if (ok) {
            c->dropped++;   /* previous frame still going out: skip this one, never queue */
        }
        if (!ok) {
            ESP_LOGW(TAG, "WebSocket client on socket %d went away", fd);
            client_remove(fd);
            httpd_sess_trigger_close(s_server, fd);
        }
    }
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000);
    if (now - s_last_report >= 5) {
        s_last_report = now;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            ws_client_t *c = &s_clients[i];
            if (c->fd >= 0 && (c->sent + c->dropped) > 0) {
                ESP_LOGI(TAG, "Wi-Fi socket %d: %lu frames sent, %lu skipped because the link was busy (last 5 s)",
                         c->fd, (unsigned long)c->sent, (unsigned long)c->dropped);
                c->sent = c->dropped = 0;
            }
        }
    }
}

static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 7;         /* lwIP has 10 sockets and the server keeps 3 for itself */
    cfg.max_uri_handlers = 20;
    cfg.stack_size = 6144;            /* the admin API builds JSON and runs Wi-Fi scans here */
    cfg.lru_purge_enable = true;
    cfg.open_fn = on_open;
    cfg.close_fn = on_close;
    cfg.send_wait_timeout = 1;
    cfg.recv_wait_timeout = 10;     /* firmware uploads over a weak link can pause for a while */
    cfg.core_id = 0;
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed to start");
        s_server = NULL;
        return;
    }
    static const httpd_uri_t ws = { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true };
    httpd_register_uri_handler(s_server, &ws);
    ota_register(s_server);
    admin_register(s_server, s_on_command);
}

void net_start(net_command_cb_t on_command)
{
    s_on_command = on_command;
    for (int i = 0; i < MAX_CLIENTS; i++) s_clients[i].fd = -1;
    wifi_start();
    mdns_start();
    http_start();
}
