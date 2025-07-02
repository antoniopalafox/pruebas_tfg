/**
 * @file main.c
 * @brief Programa de prueba para sensor VL53L0X en ESP32
 * @author Tu nombre
 * @date 2025
 * 
 * Conexiones requeridas:
 * VL53L0X VCC  -> ESP32 3.3V
 * VL53L0X GND  -> ESP32 GND
 * VL53L0X SCL  -> ESP32 GPIO 22
 * VL53L0X SDA  -> ESP32 GPIO 21
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_timer.h"

// Incluir driver del sensor
#include "vl53l0x.h"

// Etiqueta para logs
static const char *TAG = "VL53L0X_TEST";

// Configuración de pines I2C
#define I2C_MASTER_SCL_IO           22      // Pin SCL para I2C
#define I2C_MASTER_SDA_IO           21      // Pin SDA para I2C
#define I2C_MASTER_NUM              0       // Puerto I2C número 0
#define I2C_MASTER_FREQ_HZ          400000  // Frecuencia I2C: 400 KHz

// Configuración del sensor VL53L0X
#define VL53L0X_ADDRESS             0x29    // Dirección I2C por defecto
#define VL53L0X_CAL_FACTOR          1.0f    // Factor de calibración inicial

// Variable global del sensor
vl53l0x_sensor_t laser_sensor;

/**
 * @brief Inicializar el bus I2C
 */
static esp_err_t i2c_master_init(void) {
    ESP_LOGI(TAG, "Inicializando bus I2C...");
    
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
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✓ Bus I2C inicializado correctamente");
    ESP_LOGI(TAG, "  SCL: GPIO %d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "  SDA: GPIO %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  Frecuencia: %d Hz", I2C_MASTER_FREQ_HZ);
    
    return ESP_OK;
}

/**
 * @brief Mostrar información de conexiones
 */
static void show_connection_info(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         CONEXIONES REQUERIDAS        ║");
    ESP_LOGI(TAG, "╠══════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ VL53L0X VCC  ->  ESP32 3.3V          ║");
    ESP_LOGI(TAG, "║ VL53L0X GND  ->  ESP32 GND           ║");
    ESP_LOGI(TAG, "║ VL53L0X SCL  ->  ESP32 GPIO 22       ║");
    ESP_LOGI(TAG, "║ VL53L0X SDA  ->  ESP32 GPIO 21       ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "⚠️  IMPORTANTE: Usar 3.3V, NO 5V");
    ESP_LOGI(TAG, "");
}

/**
 * @brief Tarea principal del sensor VL53L0X
 */
void vl53l0x_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando tarea del sensor VL53L0X");
    
    // Mostrar información de conexiones
    show_connection_info();
    
    // Esperar para que el sistema se estabilice
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Escanear bus I2C para diagnóstico
    ESP_LOGI(TAG, "Ejecutando escaneo I2C para diagnóstico...");
    vl53l0x_i2c_scanner(I2C_MASTER_NUM);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Probar conexión específica con el sensor
    if (!vl53l0x_test_i2c_connection(I2C_MASTER_NUM, VL53L0X_ADDRESS)) {
        ESP_LOGE(TAG, "❌ No se puede conectar con VL53L0X en dirección 0x%02X", VL53L0X_ADDRESS);
        ESP_LOGE(TAG, "Verificar conexiones y volver a intentar");
        vTaskDelete(NULL);
        return;
    }
    
    // Inicializar el sensor
    ESP_LOGI(TAG, "Inicializando sensor VL53L0X...");
    if (!vl53l0x_init(&laser_sensor, I2C_MASTER_NUM, VL53L0X_ADDRESS)) {
        ESP_LOGE(TAG, "❌ Error al inicializar el sensor VL53L0X");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "✓ Sensor VL53L0X inicializado correctamente");
    
    // Configurar el sensor
    ESP_LOGI(TAG, "Configurando sensor...");
    
    // Probar diferentes niveles de precisión
    vl53l0x_accuracy_t accuracy_levels[] = {
        VL53L0X_ACCURACY_FAST,
        VL53L0X_ACCURACY_GOOD,
        VL53L0X_ACCURACY_BETTER
    };
    
    const char* accuracy_names[] = {
        "RÁPIDO (~20ms)",
        "BUENO (~30ms)", 
        "MEJOR (~70ms)"
    };
    
    int current_accuracy = 1; // Empezar con GOOD
    
    if (!vl53l0x_set_accuracy(&laser_sensor, accuracy_levels[current_accuracy])) {
        ESP_LOGW(TAG, "⚠️  Error configurando precisión, usando valores por defecto");
    } else {
        ESP_LOGI(TAG, "✓ Precisión configurada: %s", accuracy_names[current_accuracy]);
    }
    
    // Configurar modo de medición
    if (!vl53l0x_set_mode(&laser_sensor, VL53L0X_MODE_SINGLE)) {
        ESP_LOGW(TAG, "⚠️  Error configurando modo, usando valores por defecto");
    } else {
        ESP_LOGI(TAG, "✓ Modo configurado: MEDICIÓN ÚNICA");
    }
    
    // Configurar calibración
    vl53l0x_set_calibration(&laser_sensor, VL53L0X_CAL_FACTOR);
    ESP_LOGI(TAG, "✓ Factor de calibración: %.3f", VL53L0X_CAL_FACTOR);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🚀 Iniciando mediciones...");
    ESP_LOGI(TAG, "Presiona BOOT para cambiar precisión");
    ESP_LOGI(TAG, "");
    
    // Variables para estadísticas
    uint32_t measurement_count = 0;
    uint32_t error_count = 0;
    uint16_t min_distance = 65535;
    uint16_t max_distance = 0;
    uint32_t distance_sum = 0;
    
    // Tiempo de inicio para FPS
    int64_t start_time = esp_timer_get_time();
    int64_t last_stats_time = start_time;
    
    while (1) {
        // Medir tiempo de lectura
        int64_t read_start = esp_timer_get_time();
        
        // Leer distancia del sensor
        uint16_t distance = vl53l0x_read_distance(&laser_sensor);
        
        int64_t read_end = esp_timer_get_time();
        uint32_t read_time_ms = (read_end - read_start) / 1000;
        
        measurement_count++;
        
        if (distance > 0) {
            // Actualizar estadísticas
            if (distance < min_distance) min_distance = distance;
            if (distance > max_distance) max_distance = distance;
            distance_sum += distance;
            
            ESP_LOGI(TAG, "📏 Distancia: %4u mm | Tiempo: %2u ms | Medición: %u", 
                     distance, read_time_ms, measurement_count);
        } else {
            error_count++;
            ESP_LOGW(TAG, "❌ Error en lectura #%u (errores: %u)", measurement_count, error_count);
        }
        
        // Mostrar estadísticas cada 30 segundos
        int64_t current_time = esp_timer_get_time();
        if ((current_time - last_stats_time) >= 30000000) { // 30 segundos
            if (measurement_count > error_count) {
                uint16_t avg_distance = distance_sum / (measurement_count - error_count);
                float success_rate = ((float)(measurement_count - error_count) / measurement_count) * 100.0f;
                float fps = (float)measurement_count / ((current_time - start_time) / 1000000.0f);
                
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "📊 ESTADÍSTICAS (últimos 30s):");
                ESP_LOGI(TAG, "   Mediciones: %u | Errores: %u | Éxito: %.1f%%", 
                         measurement_count, error_count, success_rate);
                ESP_LOGI(TAG, "   Distancia - Min: %u mm | Max: %u mm | Promedio: %u mm", 
                         min_distance, max_distance, avg_distance);
                ESP_LOGI(TAG, "   Velocidad: %.1f mediciones/segundo", fps);
                ESP_LOGI(TAG, "   Precisión actual: %s", accuracy_names[current_accuracy]);
                ESP_LOGI(TAG, "");
            }
            
            last_stats_time = current_time;
        }
        
        // Esperar antes de la siguiente medición
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarea para manejar cambio de precisión con botón BOOT
 */
void button_task(void *pvParameters) {
    // Esta es una implementación simplificada
    // En una implementación completa, configuraríamos GPIO0 como input
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    while (1) {
        // Simular cambio de precisión cada 60 segundos
        vTaskDelay(60000 / portTICK_PERIOD_MS);
        
        static int accuracy_index = 1;
        accuracy_index = (accuracy_index + 1) % 3;
        
        vl53l0x_accuracy_t accuracy_levels[] = {
            VL53L0X_ACCURACY_FAST,
            VL53L0X_ACCURACY_GOOD,
            VL53L0X_ACCURACY_BETTER
        };
        
        const char* accuracy_names[] = {
            "RÁPIDO (~20ms)",
            "BUENO (~30ms)", 
            "MEJOR (~70ms)"
        };
        
        if (vl53l0x_set_accuracy(&laser_sensor, accuracy_levels[accuracy_index])) {
            ESP_LOGI(TAG, "🔧 Precisión cambiada a: %s", accuracy_names[accuracy_index]);
        }
    }
}

/**
 * @brief Función principal de la aplicación
 */
void app_main(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🔬 PROYECTO VL53L0X - ESP32");
    ESP_LOGI(TAG, "   Sensor de distancia láser");
    ESP_LOGI(TAG, "   Versión: 1.0");
    ESP_LOGI(TAG, "");
    
    // Inicializar NVS (necesario para WiFi, pero lo incluimos por completitud)
    ESP_LOGI(TAG, "Inicializando NVS...");
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS inicializado");
    
    // Inicializar I2C
    ESP_ERROR_CHECK(i2c_master_init());
    
    // Esperar un momento para que el I2C se estabilice
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Crear tareas
    ESP_LOGI(TAG, "Creando tareas...");
    
    BaseType_t task_result = xTaskCreate(
        vl53l0x_task,           // Función de la tarea
        "vl53l0x_task",         // Nombre de la tarea
        8192,                   // Tamaño del stack (8KB)
        NULL,                   // Parámetros
        5,                      // Prioridad
        NULL                    // Handle de la tarea
    );
    
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "❌ Error creando tarea principal");
        return;
    }
    
    xTaskCreate(
        button_task,            // Función de la tarea
        "button_task",          // Nombre de la tarea
        2048,                   // Tamaño del stack (2KB)
        NULL,                   // Parámetros
        3,                      // Prioridad
        NULL                    // Handle de la tarea
    );
    
    ESP_LOGI(TAG, "✓ Tareas creadas correctamente");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Sistema iniciado. Monitoreando sensor...");
}