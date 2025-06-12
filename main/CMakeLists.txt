idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "esp_lcd" "lvgl" "esp_wifi" "esp_netif" "nvs_flash" "esp_event" "bme280")
