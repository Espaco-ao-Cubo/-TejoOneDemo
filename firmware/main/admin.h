#pragma once
#include "esp_http_server.h"
#include "net.h"

/* Capture ESP log output in a ring buffer for GET /api/log. Call first thing in app_main. */
void admin_log_init(void);

/* Serve the admin page at / and its API under /api/. */
void admin_register(httpd_handle_t server, net_command_cb_t on_command);
