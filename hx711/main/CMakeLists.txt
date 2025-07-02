idf_component_register(SRCS "main.c" "hx711.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos esp_common esp_system nvs_flash)