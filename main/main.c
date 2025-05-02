// main.c - Programa de prueba para sensores HC-SR04P, HX711 y VL53L0X en ESP32
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

// Incluir drivers de sensores
#include "hcsr04p.h"
#include "hx711.h"
#include "vl53l0x.h"

// Etiqueta para logs
static const char *TAG = "SENSOR_TEST";

// Configuración de pines y conexiones
#define I2C_MASTER_SCL_IO           22      // Pin SCL para I2C
#define I2C_MASTER_SDA_IO           21      // Pin SDA para I2C
#define I2C_MASTER_NUM              0       // Puerto I2C número 0
#define I2C_MASTER_FREQ_HZ          400000  // Frecuencia I2C: 400 KHz

// Configuración HC-SR04P
#define HCSR04P_TRIGGER_PIN         12
#define HCSR04P_ECHO_PIN            13
#define HCSR04P_CAL_FACTOR          1.02f   // Factor de calibración ejemplo

// Configuración HX711
#define HX711_DOUT_PIN              26
#define HX711_SCK_PIN               27
#define HX711_KNOWN_WEIGHT_G        500     // Peso conocido en gramos para calibración

// Configuración VL53L0X
#define VL53L0X_ADDRESS             0x29
#define VL53L0X_CAL_FACTOR          1.05f   // Factor de calibración ejemplo

// Estructuras para los sensores
hcsr04p_sensor_t ultrasonic_sensor;
hx711_sensor_t scale_sensor;
vl53l0x_sensor_t laser_sensor;

// Inicializar el bus I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Tarea de lectura del sensor HC-SR04P
void ultrasonic_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando tarea de sensor ultrasónico HC-SR04P");
    
    // Inicializar el sensor
    if (!hcsr04p_init(&ultrasonic_sensor, HCSR04P_TRIGGER_PIN, HCSR04P_ECHO_PIN)) {
        ESP_LOGE(TAG, "Error al inicializar el sensor HC-SR04P");
        vTaskDelete(NULL);
        return;
    }
    
    // Configurar calibración
    hcsr04p_set_calibration(&ultrasonic_sensor, HCSR04P_CAL_FACTOR);
    ESP_LOGI(TAG, "Sensor HC-SR04P inicializado con factor de calibración: %.2f", HCSR04P_CAL_FACTOR);
    
    while (1) {
        // Leer distancia
        float distance = hcsr04p_read_distance(&ultrasonic_sensor);
        
        if (distance >= 0) {
            ESP_LOGI(TAG, "HC-SR04P: Distancia: %.2f cm", distance);
        } else {
            ESP_LOGW(TAG, "HC-SR04P: Error en lectura de distancia");
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Lectura cada segundo
    }
}

// Tarea de lectura del sensor HX711
void scale_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando tarea de sensor de peso HX711");
    
    // Esperar un momento para asegurar que el sensor esté listo
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Inicializar el sensor
    if (!hx711_init(&scale_sensor, HX711_DOUT_PIN, HX711_SCK_PIN, HX711_GAIN_128)) {
        ESP_LOGE(TAG, "Error al inicializar el sensor HX711");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Realizando tara del sensor de peso...");
    hx711_tare(&scale_sensor, 10); // 10 lecturas para tara
    
    ESP_LOGI(TAG, "Calibrando sensor HX711...");
    ESP_LOGI(TAG, "Por favor, coloque un peso conocido de %d gramos en la balanza", HX711_KNOWN_WEIGHT_G);
    ESP_LOGI(TAG, "Tiene 5 segundos para colocar el peso...");
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    // Calibrar con peso conocido
    hx711_calibrate(&scale_sensor, HX711_KNOWN_WEIGHT_G, 10); // 10 lecturas para calibración
    
    ESP_LOGI(TAG, "Calibración completada. Puede retirar el peso.");
    ESP_LOGI(TAG, "Factor de escala: %.2f", scale_sensor.scale);
    
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    while (1) {
        // Leer peso
        float weight = hx711_read_units(&scale_sensor);
        
        ESP_LOGI(TAG, "HX711: Peso: %.2f g", weight);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Lectura cada segundo
    }
}

// Tarea de lectura del sensor VL53L0X
void laser_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando tarea de sensor láser VL53L0X");
    
    // Inicializar el sensor
    if (!vl53l0x_init(&laser_sensor, I2C_MASTER_NUM, VL53L0X_ADDRESS)) {
        ESP_LOGE(TAG, "Error al inicializar el sensor VL53L0X");
        vTaskDelete(NULL);
        return;
    }
    
    // Configurar el sensor para mejor precisión
    vl53l0x_set_accuracy(&laser_sensor, VL53L0X_ACCURACY_BETTER);
    
    // Configurar calibración
    vl53l0x_set_calibration(&laser_sensor, VL53L0X_CAL_FACTOR);
    
    ESP_LOGI(TAG, "Sensor VL53L0X inicializado con factor de calibración: %.2f", VL53L0X_CAL_FACTOR);
    
    while (1) {
        // Leer distancia
        uint16_t distance = vl53l0x_read_distance(&laser_sensor);
        
        ESP_LOGI(TAG, "VL53L0X: Distancia: %u mm", distance);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Lectura cada segundo
    }
}

void app_main(void) {
    esp_err_t ret;
    
    // Inicializar NVS (para almacenar calibraciones si es necesario)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Iniciando programa de prueba de sensores");
    
    // Inicializar I2C (para el sensor VL53L0X)
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "Bus I2C inicializado");
    
    // Crear tareas para cada sensor
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, NULL);
    xTaskCreate(scale_task, "scale_task", 4096, NULL, 5, NULL);
    xTaskCreate(laser_task, "laser_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Tareas de sensores creadas");
}