#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "hx711.h"

static const char *TAG = "HX711_MAIN";

// Configuración del HX711
#define HX711_DOUT_PIN      GPIO_NUM_26
#define HX711_SCK_PIN       GPIO_NUM_27
#define CALIBRATION_WEIGHT  500.0f  // Peso conocido en gramos
#define TARE_SAMPLES        15      // Muestras para tara
#define CALIBRATION_SAMPLES 15      // Muestras para calibración

// Variable global del sensor
static hx711_t hx711_sensor;

// Función para verificar conexiones
static void check_connections(void)
{
    ESP_LOGI(TAG, "=== VERIFICACIÓN DE CONEXIONES ===");
    ESP_LOGI(TAG, "Pin DOUT (%d): %s", HX711_DOUT_PIN, 
             gpio_get_level(HX711_DOUT_PIN) ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "Pin SCK (%d): %s", HX711_SCK_PIN,
             gpio_get_level(HX711_SCK_PIN) ? "HIGH" : "LOW");
    
    // Verificar si hay actividad en DOUT
    int high_count = 0, low_count = 0;
    for (int i = 0; i < 100; i++) {
        if (gpio_get_level(HX711_DOUT_PIN)) {
            high_count++;
        } else {
            low_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Actividad DOUT - HIGH: %d, LOW: %d", high_count, low_count);
    
    if (low_count == 0) {
        ESP_LOGW(TAG, "⚠️  DOUT siempre está en HIGH - Posible problema de conexión");
    } else if (high_count == 0) {
        ESP_LOGI(TAG, "✅ DOUT está en LOW - Sensor probablemente listo");
    } else {
        ESP_LOGI(TAG, "ℹ️  DOUT cambia de estado - Sensor activo");
    }
}

// Tarea principal del HX711
static void hx711_task(void *pvParameters)
{
    ESP_LOGI(TAG, "=== PROYECTO ESP32 HX711 INICIADO ===");
    ESP_LOGI(TAG, "Configuración:");
    ESP_LOGI(TAG, "  - DOUT Pin: %d", HX711_DOUT_PIN);
    ESP_LOGI(TAG, "  - SCK Pin: %d", HX711_SCK_PIN);
    ESP_LOGI(TAG, "  - Peso de calibración: %.0f g", CALIBRATION_WEIGHT);
    ESP_LOGI(TAG, "  - Ganancia: 128 (Canal A)");
    
    // Esperar un poco antes de comenzar
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Verificar conexiones
    check_connections();
    
    // Configurar HX711
    hx711_config_t config = {
        .dout_pin = HX711_DOUT_PIN,
        .sck_pin = HX711_SCK_PIN,
        .gain = HX711_GAIN_128  // ← CORREGIDO: Era 1, ahora es 128
    };
    
    // Inicializar sensor
    esp_err_t ret = hx711_init(&hx711_sensor, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando HX711: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Esperar estabilización adicional
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Mostrar información de debug
    hx711_debug_info(&hx711_sensor);
    
    // Lecturas RAW iniciales
    ESP_LOGI(TAG, "\n=== LECTURAS RAW SIN CALIBRAR ===");
    for (int i = 1; i <= 10; i++) {
        int32_t raw_value;
        ret = hx711_read_raw(&hx711_sensor, &raw_value);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Lectura RAW #%d: %ld", i, (long)raw_value);
        } else {
            ESP_LOGW(TAG, "Error leyendo RAW #%d: %s", i, esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Proceso de tara
    ESP_LOGI(TAG, "\n=== PROCESO DE TARA ===");
    ESP_LOGI(TAG, "IMPORTANTE: Retire todo peso de la balanza");
    ESP_LOGI(TAG, "La tara comenzará en 5 segundos...");
    
    for (int i = 5; i > 0; i--) {
        ESP_LOGI(TAG, "Tara en %d segundos...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ret = hx711_tare(&hx711_sensor, TARE_SAMPLES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error durante la tara: %s", esp_err_to_name(ret));
    }
    
    // Verificar tara
    ESP_LOGI(TAG, "\n=== VERIFICACIÓN DE TARA ===");
    for (int i = 1; i <= 5; i++) {
        int32_t raw_value;
        float units;
        
        ret = hx711_read_raw(&hx711_sensor, &raw_value);
        if (ret == ESP_OK) {
            hx711_read_units(&hx711_sensor, &units);
            ESP_LOGI(TAG, "Verificación %d - RAW: %ld, Unidades: %.2f", 
                     i, (long)raw_value, units);
        } else {
            ESP_LOGW(TAG, "Error en verificación %d: %s", i, esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Proceso de calibración
    ESP_LOGI(TAG, "\n=== PROCESO DE CALIBRACIÓN ===");
    ESP_LOGI(TAG, "Coloque un peso conocido de %.0f gramos en la balanza", CALIBRATION_WEIGHT);
    ESP_LOGI(TAG, "La calibración comenzará en 10 segundos...");
    
    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "Calibración en %d segundos...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ret = hx711_calibrate(&hx711_sensor, CALIBRATION_WEIGHT, CALIBRATION_SAMPLES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error durante la calibración: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Puede retirar el peso de calibración");
    ESP_LOGI(TAG, "Comenzando lecturas normales en 3 segundos...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Mostrar información final
    hx711_debug_info(&hx711_sensor);
    
    // Lecturas continuas
    ESP_LOGI(TAG, "\n=== LECTURAS CONTINUAS ===");
    ESP_LOGI(TAG, "Formato: [Tiempo] RAW: valor | Peso: valor g");
    
    int reading_count = 1;
    while (1) {
        int32_t raw_value;
        float weight;
        
        ret = hx711_read_raw(&hx711_sensor, &raw_value);
        if (ret == ESP_OK) {
            hx711_read_units(&hx711_sensor, &weight);
            ESP_LOGI(TAG, "[%d] RAW: %ld | Peso: %.2f g", 
                     reading_count, (long)raw_value, weight);
        } else {
            ESP_LOGW(TAG, "[%d] Error leyendo sensor: %s", 
                     reading_count, esp_err_to_name(ret));
        }
        
        reading_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Cada 10 lecturas, mostrar información de debug
        if (reading_count % 10 == 0) {
            ESP_LOGI(TAG, "\n--- Estado del sensor ---");
            ESP_LOGI(TAG, "Sensor listo: %s", hx711_is_ready(&hx711_sensor) ? "SÍ" : "NO");
            ESP_LOGI(TAG, "Offset: %ld", (long)hx711_sensor.offset);
            ESP_LOGI(TAG, "Escala: %.2f", hx711_sensor.scale);
        }
    }
}

void app_main(void)
{
    // Inicializar NVS (si es necesario)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Iniciando tarea del sensor HX711");
    
    // Crear tarea del HX711
    BaseType_t task_created = xTaskCreate(
        hx711_task,           // Función de la tarea
        "hx711_task",         // Nombre de la tarea
        4096,                 // Tamaño del stack
        NULL,                 // Parámetros
        5,                    // Prioridad
        NULL                  // Handle de la tarea
    );
    
    if (task_created == pdPASS) {
        ESP_LOGI(TAG, "Tarea HX711 creada correctamente");
    } else {
        ESP_LOGE(TAG, "Error creando tarea HX711");
    }
}
/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define DOUT_PIN GPIO_NUM_26
#define SCK_PIN  GPIO_NUM_27

static const char *TAG = "HX711_TEST";

// Función simple para leer un valor RAW
uint32_t read_hx711_simple(void) {
    // Esperar hasta que DOUT esté en LOW
    int timeout = 1000;
    while (gpio_get_level(DOUT_PIN) == 1 && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (timeout <= 0) {
        ESP_LOGW(TAG, "Timeout esperando DOUT LOW");
        return 0xFFFFFFFF;
    }
    
    uint32_t data = 0;
    
    // Leer 24 bits
    for (int i = 0; i < 24; i++) {
        gpio_set_level(SCK_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        
        data <<= 1;
        if (gpio_get_level(DOUT_PIN)) {
            data |= 1;
        }
        
        gpio_set_level(SCK_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Pulso extra para ganancia 128
    gpio_set_level(SCK_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(SCK_PIN, 0);
    
    return data;
}

void app_main(void) {
    ESP_LOGI(TAG, "=== DIAGNÓSTICO SIMPLE HX711 ===");
    
    // Configurar pines
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << SCK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);
    
    config.pin_bit_mask = (1ULL << DOUT_PIN);
    config.mode = GPIO_MODE_INPUT;
    gpio_config(&config);
    
    gpio_set_level(SCK_PIN, 0);
    
    ESP_LOGI(TAG, "Configuración completada");
    ESP_LOGI(TAG, "DOUT Pin: %d, SCK Pin: %d", DOUT_PIN, SCK_PIN);
    
    // Verificar estado inicial
    ESP_LOGI(TAG, "Estado inicial - DOUT: %s, SCK: %s", 
             gpio_get_level(DOUT_PIN) ? "HIGH" : "LOW",
             gpio_get_level(SCK_PIN) ? "HIGH" : "LOW");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Lecturas continuas muy simples
    for (int i = 1; i <= 20; i++) {
        uint32_t raw = read_hx711_simple();
        
        ESP_LOGI(TAG, "Lectura %d: 0x%08lX (%lu decimal)", 
                 i, (unsigned long)raw, (unsigned long)raw);
        
        // Verificar estados
        if (i % 5 == 0) {
            ESP_LOGI(TAG, "  Estado - DOUT: %s, SCK: %s", 
                     gpio_get_level(DOUT_PIN) ? "HIGH" : "LOW",
                     gpio_get_level(SCK_PIN) ? "HIGH" : "LOW");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "Test completado");
}*/