# main/CMakeLists.txt

idf_component_register(
    SRCS 
        "main.c"
        "vl53l0x.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver
        esp_timer
        nvs_flash
)