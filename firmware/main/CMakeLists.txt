idf_component_register(SRCS "imu_main.c" "imu_sensors.c" "fusion.c" "net.c" "ota.c" "admin.c" "wifi_mgr.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "admin.html"
                       PRIV_REQUIRES esp_driver_i2c esp_driver_uart esp_timer esp_wifi esp_netif
                                     esp_event nvs_flash esp_http_server app_update esp_app_format
                                     esp_partition)
