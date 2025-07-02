#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "hx711.h"

// Etiqueta para logs
static const char *TAG = "HX711_MAIN";

// Configuración de pines para HX711
#define HX711_DOUT_PIN              26
#define HX711_SCK_PIN               27
#define HX711_KNOWN_WEIGHT_G        500.0f    // Peso conocido para calibración

// Estructura del sensor
hx711_sensor_t scale_sensor;

// Tarea principal del sensor HX711
void hx711_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando tarea del sensor HX711");
    
    // Esperar estabilización inicial
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Inicializar el sensor
    if (!hx711_init(&scale_sensor, HX711_DOUT_PIN, HX711_SCK_PIN, HX711_GAIN_128)) {
        ESP_LOGE(TAG, "Error al inicializar el sensor HX711");
        vTaskDelete(NULL);
        return;
    }
    
    // Mostrar información de debug inicial
    hx711_debug_info(&scale_sensor);
    
    // Mostrar lecturas RAW sin calibrar
    ESP_LOGI(TAG, "\n=== LECTURAS RAW SIN CALIBRAR ===");
    for (int i = 0; i < 10; i++) {
        int32_t raw_reading = hx711_read_raw(&scale_sensor);
        ESP_LOGI(TAG, "Lectura RAW #%d: %ld", i+1, raw_reading);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    // Proceso de tara
    ESP_LOGI(TAG, "\n=== PROCESO DE TARA ===");
    ESP_LOGI(TAG, "IMPORTANTE: Retire todo peso de la balanza");
    ESP_LOGI(TAG, "La tara comenzará en 5 segundos...");
    
    for (int i = 5; i > 0; i--) {
        ESP_LOGI(TAG, "Tara en %d segundos...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    hx711_tare(&scale_sensor, 15);
    
    // Verificar tara
    ESP_LOGI(TAG, "\n=== VERIFICACIÓN DE TARA ===");
    for (int i = 0; i < 5; i++) {
        int32_t raw_reading = hx711_read_raw(&scale_sensor);
        float units = hx711_read_units(&scale_sensor);
        ESP_LOGI(TAG, "Verificación %d - RAW: %ld, Unidades: %.2f", i+1, raw_reading, units);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    // Proceso de calibración
    ESP_LOGI(TAG, "\n=== PROCESO DE CALIBRACIÓN ===");
    ESP_LOGI(TAG, "Coloque un peso conocido de %.0f gramos en la balanza", HX711_KNOWN_WEIGHT_G);
    ESP_LOGI(TAG, "La calibración comenzará en 10 segundos...");
    
    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "Calibración en %d segundos...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    hx711_calibrate(&scale_sensor, HX711_KNOWN_WEIGHT_G, 15);
    
    ESP_LOGI(TAG, "Puede retirar el peso de calibración");
    ESP_LOGI(TAG, "Comenzando lecturas normales en 3 segundos...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Mostrar información final de calibración
    hx711_debug_info(&scale_sensor);
    
    ESP_LOGI(TAG, "\n=== LECTURAS CONTINUAS ===");
    ESP_LOGI(TAG, "Formato: [Tiempo] RAW: valor | Peso: valor g");
    
    // Bucle principal de lecturas
    uint32_t reading_count = 0;
    while (1) {
        reading_count++;
        
        // Leer valores
        int32_t raw_reading = hx711_read_raw(&scale_sensor);
        float weight = hx711_read_units(&scale_sensor);
        
        // Mostrar lecturas
        ESP_LOGI(TAG, "[%lu] RAW: %ld | Peso: %.2f g", 
                 reading_count, raw_reading, weight);
        
        // Cada 30 lecturas mostrar información de debug
        if (reading_count % 30 == 0) {
            ESP_LOGI(TAG, "\n=== INFORMACIÓN PERIÓDICA DE DEBUG ===");
            hx711_debug_info(&scale_sensor);
            ESP_LOGI(TAG, "=== CONTINUANDO LECTURAS ===\n");
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    esp_err_t ret;
    
    // Inicializar NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "=== PROYECTO ESP32 HX711 INICIADO ===");
    ESP_LOGI(TAG, "Configuración:");
    ESP_LOGI(TAG, "  - DOUT Pin: %d", HX711_DOUT_PIN);
    ESP_LOGI(TAG, "  - SCK Pin: %d", HX711_SCK_PIN);
    ESP_LOGI(TAG, "  - Peso de calibración: %.0f g", HX711_KNOWN_WEIGHT_G);
    ESP_LOGI(TAG, "  - Ganancia: 128 (Canal A)");
    
    // Crear tarea del sensor HX711
    xTaskCreate(hx711_task, "hx711_task", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Tarea HX711 creada correctamente");
}