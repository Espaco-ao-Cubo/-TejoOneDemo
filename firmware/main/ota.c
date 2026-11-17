/* Firmware update over Wi-Fi (OTA).
 *
 * The flash holds two app slots, ota_0 and ota_1. The running firmware receives the new image
 * over HTTP, writes it into the OTHER slot, checks it, and tells the bootloader to start from
 * that slot on the next boot. The old firmware stays untouched until the next update.
 *
 * Rollback: a freshly installed image boots as "pending verify". If it crashes or resets before
 * it has run for VALID_AFTER_S seconds with the web server up, the bootloader goes back to the
 * previous firmware. So a broken update can't lock you out of updating again.
 *
 *   POST /update     body = build/blink.bin            (tools/ota.py does this)
 *   GET  /version    JSON: version, build id, slot, state
 */
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "ota.h"

static const char *TAG = "ota";
#define VALID_AFTER_S 10
#define CHUNK 4096

static volatile bool s_busy;

static void cors(httpd_req_t *req)
{
    /* The lab page is opened from a file, so the browser asks permission first (CORS). */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, X-OTA-Password");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
}

bool ota_auth_ok(httpd_req_t *req)
{
    if (strlen(CONFIG_IMU_OTA_PASSWORD) == 0) return true;
    char pw[64] = { 0 };
    return httpd_req_get_hdr_value_str(req, "X-OTA-Password", pw, sizeof(pw)) == ESP_OK &&
           strcmp(pw, CONFIG_IMU_OTA_PASSWORD) == 0;
}

static esp_err_t fail(httpd_req_t *req, const char *status, const char *msg)
{
    ESP_LOGW(TAG, "Update refused: %s", msg);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, msg);
}

static void sha_hex(char out[17])
{
    const esp_app_desc_t *d = esp_app_get_description();
    for (int i = 0; i < 8; i++) sprintf(out + 2 * i, "%02x", d->app_elf_sha256[i]);
}

const char *ota_state_str(void)
{
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) != ESP_OK) return "factory";
    switch (st) {
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending verify";
    case ESP_OTA_IMG_NEW: return "new";
    default: return "other";
    }
}

static esp_err_t version_get(httpd_req_t *req)
{
    cors(req);
    const esp_app_desc_t *d = esp_app_get_description();
    char sha[17], body[320];
    sha_hex(sha);
    snprintf(body, sizeof(body),
             "{\"project\":\"%s\",\"version\":\"%s\",\"build\":\"%s\",\"date\":\"%s %s\",\"idf\":\"%s\","
             "\"slot\":\"%s\",\"state\":\"%s\",\"uptime_s\":%lld}",
             d->project_name, d->version, sha, d->date, d->time, d->idf_ver,
             esp_ota_get_running_partition()->label, ota_state_str(),
             (long long)(xTaskGetTickCount() / configTICK_RATE_HZ));   /* ticks restart at every boot */
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t options(httpd_req_t *req)
{
    cors(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static void restart_cb(void *arg) { esp_restart(); }

static esp_err_t update_post(httpd_req_t *req)
{
    cors(req);
    if (!ota_auth_ok(req)) {
        return fail(req, "403 Forbidden", "Wrong or missing admin password (X-OTA-Password)");
    }
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) return fail(req, "500 Internal Server Error", "No OTA slot: flash the OTA partition table over USB once");
    if (req->content_len < 1024 || req->content_len > part->size) {
        return fail(req, "400 Bad Request", "Image size doesn't fit the update slot");
    }
    if (s_busy) return fail(req, "409 Conflict", "Another update is running");
    s_busy = true;

    ESP_LOGI(TAG, "Receiving %u bytes into %s", (unsigned)req->content_len, part->label);
    char *buf = malloc(CHUNK);
    if (!buf) { s_busy = false; return fail(req, "500 Internal Server Error", "Out of memory"); }
    esp_ota_handle_t h = 0;
    bool begun = false;
    esp_err_t err = ESP_OK;
    size_t left = req->content_len, got = 0;
    int timeouts = 0, last_pct = -10;
    const char *msg = NULL;

    while (left > 0) {
        int r = httpd_req_recv(req, buf, left < CHUNK ? left : CHUNK);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) continue;
        if (r <= 0) { msg = "Connection lost during upload"; break; }
        timeouts = 0;
        if (!begun) {
            /* The app description sits right after the image and first segment headers. Refuse
             * images from another project before erasing anything. */
            size_t off = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
            if ((size_t)r < off + sizeof(esp_app_desc_t) || (uint8_t)buf[0] != ESP_IMAGE_HEADER_MAGIC) {
                msg = "Not an ESP32 firmware image (expected build/blink.bin)"; break;
            }
            const esp_app_desc_t *nd = (const esp_app_desc_t *)(buf + off);
            if (nd->magic_word != ESP_APP_DESC_MAGIC_WORD ||
                strncmp(nd->project_name, esp_app_get_description()->project_name, sizeof(nd->project_name)) != 0) {
                msg = "Image belongs to a different project"; break;
            }
            /* Sequential writes erase as they go, so the server never stalls for a full-slot erase. */
            err = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &h);
            if (err != ESP_OK) { msg = "Could not start writing the update slot"; break; }
            begun = true;
        }
        err = esp_ota_write(h, buf, r);
        if (err != ESP_OK) { msg = "Flash write failed"; break; }
        left -= r;
        got += r;
        int pct = (int)(100.0 * got / req->content_len);
        if (pct >= last_pct + 10) { ESP_LOGI(TAG, "%3d %%", pct); last_pct = pct; }
    }
    free(buf);

    if (!msg && begun) {
        err = esp_ota_end(h);           /* checks the whole image: format, chip, SHA-256 */
        begun = false;
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) msg = "Image failed verification (corrupt or wrong chip)";
        else if (err != ESP_OK) msg = "Could not finish the update";
    }
    if (begun) esp_ota_abort(h);
    if (!msg && esp_ota_set_boot_partition(part) != ESP_OK) msg = "Could not select the new firmware";
    s_busy = false;
    if (msg) return fail(req, "500 Internal Server Error", msg);

    ESP_LOGI(TAG, "Update written to %s and verified. Restarting into it.", part->label);
    char ok[96];
    snprintf(ok, sizeof(ok), "OK: %u bytes written to %s. Restarting.", (unsigned)got, part->label);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, ok);

    /* Give the reply a moment to leave before rebooting. */
    static esp_timer_handle_t t;
    const esp_timer_create_args_t a = { .callback = restart_cb, .name = "ota_restart" };
    if (!t) esp_timer_create(&a, &t);
    esp_timer_start_once(t, 700 * 1000);
    return ESP_OK;
}

static void mark_valid_cb(void *arg)
{
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "New firmware ran %d s with the web server up: marked as good, no rollback", VALID_AFTER_S);
    }
}

void ota_register(httpd_handle_t server)
{
    static const httpd_uri_t up = { .uri = "/update", .method = HTTP_POST, .handler = update_post };
    static const httpd_uri_t up_opt = { .uri = "/update", .method = HTTP_OPTIONS, .handler = options };
    static const httpd_uri_t ver = { .uri = "/version", .method = HTTP_GET, .handler = version_get };
    httpd_register_uri_handler(server, &up);
    httpd_register_uri_handler(server, &up_opt);
    httpd_register_uri_handler(server, &ver);

    const esp_app_desc_t *d = esp_app_get_description();
    char sha[17];
    sha_hex(sha);
    ESP_LOGI(TAG, "Firmware %s build %s running from %s (%s). Update over Wi-Fi: tools/ota.py",
             d->version, sha, esp_ota_get_running_partition()->label, ota_state_str());

    static esp_timer_handle_t t;
    const esp_timer_create_args_t a = { .callback = mark_valid_cb, .name = "ota_valid" };
    if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_once(t, VALID_AFTER_S * 1000000LL);
}
