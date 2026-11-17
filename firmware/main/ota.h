#pragma once
#include <stdbool.h>
#include "esp_http_server.h"

/* Adds POST /update and GET /version to the web server, and confirms a freshly
 * updated firmware as good once it has run for a few seconds. */
void ota_register(httpd_handle_t server);

/* "valid", "pending verify", ... for the running firmware slot. */
const char *ota_state_str(void);

/* True if no admin password is set in menuconfig, or the request carries the right one
 * in the X-OTA-Password header. Used for every request that changes something. */
bool ota_auth_ok(httpd_req_t *req);
