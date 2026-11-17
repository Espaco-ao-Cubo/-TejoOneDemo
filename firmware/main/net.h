#pragma once
#include <stddef.h>
#include <stdint.h>

typedef void (*net_command_cb_t)(const char *cmd);

/* Starts Wi-Fi (the IMU-Lab access point, always on, plus a saved network when one is in
 * range, see wifi_mgr.h), mDNS (<hostname>.local) and an HTTP server with the admin page at /
 * and a WebSocket at /ws. Wi-Fi power saving is turned off for low latency. */
void net_start(net_command_cb_t on_command);

/* Send a binary frame to every connected WebSocket client.
 * Never blocks and never queues a backlog: if the previous frame has not gone out yet,
 * it is replaced by this newer one. */
void net_publish(const void *data, size_t len);

/* Access point side, for the admin page. */
#define NET_MAX_STATIONS 4
typedef struct {
    uint8_t channel;            /* the radio's channel (follows the joined network, if any) */
    uint8_t ap_clients;
    uint8_t ap_mac[NET_MAX_STATIONS][6];
    int8_t  ap_rssi[NET_MAX_STATIONS];
    uint8_t ws_clients;         /* WebSocket streams (the lab) */
} net_status_t;

void net_get_status(net_status_t *s);
