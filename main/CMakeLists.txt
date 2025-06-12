idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "esp_lcd" "lvgl" "bme280")

# Add BME280 component
set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS} "managed_components/bme280")
