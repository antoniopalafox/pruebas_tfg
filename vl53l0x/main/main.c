/**
 * @file main.c
 * @brief Programa simplificado para sensor VL53L0X - Solo muestra distancia
 * 
 * Conexiones:
 * VL53L0X VCC  -> ESP32 3.3V
 * VL53L0X GND  -> ESP32 GND
 * VL53L0X SCL  -> ESP32 GPIO 22
 * VL53L0X SDA  -> ESP32 GPIO 21
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "vl53l0x.h"

// Configuración I2C
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000

// Configuración sensor
#define VL53L0X_ADDRESS             0x29

// Variable del sensor
vl53l0x_sensor_t laser_sensor;

// Inicializar I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Tarea principal del sensor
void sensor_task(void *pvParameters) {
    // Esperar estabilización
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Inicializar sensor
    if (!vl53l0x_init(&laser_sensor, I2C_MASTER_NUM, VL53L0X_ADDRESS)) {
        printf("Error: No se puede inicializar el sensor VL53L0X\n");
        printf("Verificar conexiones:\n");
        printf("VCC->3.3V, GND->GND, SCL->GPIO22, SDA->GPIO21\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Configurar para buena precisión
    vl53l0x_set_accuracy(&laser_sensor, VL53L0X_ACCURACY_GOOD);
    
    // Bucle principal - solo mostrar distancia
    while (1) {
        uint16_t distance = vl53l0x_read_distance(&laser_sensor);
        
        if (distance > 0) {
            printf("%u mm\n", distance);
        } else {
            printf("Error\n");
        }
        
        vTaskDelay(500 / portTICK_PERIOD_MS); // Medición cada 0.5 segundos
    }
}

void app_main(void) {
    // Inicializar NVS
    nvs_flash_init();
    
    // Inicializar I2C
    if (i2c_master_init() != ESP_OK) {
        printf("Error inicializando I2C\n");
        return;
    }
    
    // Crear tarea del sensor
    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);
}