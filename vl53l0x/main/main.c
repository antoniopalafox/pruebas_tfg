#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_system.h>

static const char *TAG = "VL53L0X_TEST";

// Configuración I2C
#define I2C_MASTER_SCL_IO           22    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0     /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// Configuración VL53L0X
#define VL53L0X_I2C_ADDR            0x29
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xC2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Escribir un byte al registro del VL53L0X
 */
static esp_err_t vl53l0x_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Leer un byte del registro del VL53L0X
 */
static esp_err_t vl53l0x_read_byte(uint8_t reg_addr, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Leer múltiples bytes del VL53L0X
 */
static esp_err_t vl53l0x_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Inicializar el sensor VL53L0X
 */
static esp_err_t vl53l0x_init(void)
{
    uint8_t model_id, revision_id;
    
    // Leer ID del modelo
    esp_err_t ret = vl53l0x_read_byte(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo Model ID");
        return ret;
    }
    
    // Leer ID de revisión
    ret = vl53l0x_read_byte(VL53L0X_REG_IDENTIFICATION_REVISION_ID, &revision_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo Revision ID");
        return ret;
    }
    
    ESP_LOGI(TAG, "VL53L0X detectado - Model ID: 0x%02X, Revision ID: 0x%02X", model_id, revision_id);
    
    if (model_id != 0xEE) {
        ESP_LOGE(TAG, "Model ID incorrecto. Esperado: 0xEE, Recibido: 0x%02X", model_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    return ESP_OK;
}

/**
 * @brief Realizar una medición de distancia
 */
static esp_err_t vl53l0x_read_range(uint16_t *range_mm)
{
    esp_err_t ret;
    uint8_t val = 0;
    uint8_t range_data[12];
    
    // Iniciar medición
    ret = vl53l0x_write_byte(VL53L0X_REG_SYSRANGE_START, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando medición");
        return ret;
    }
    
    // Esperar a que la medición esté lista
    int timeout = 1000; // 1 segundo timeout
    do {
        ret = vl53l0x_read_byte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, &val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error leyendo status de interrupción");
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    } while ((val & 0x07) == 0 && timeout > 0);
    
    if (timeout <= 0) {
        ESP_LOGE(TAG, "Timeout esperando medición");
        return ESP_ERR_TIMEOUT;
    }
    
    // Leer datos del rango
    ret = vl53l0x_read_bytes(VL53L0X_REG_RESULT_RANGE_STATUS, range_data, 12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo datos de rango");
        return ret;
    }
    
    // Extraer la distancia (bytes 10 y 11)
    *range_mm = (range_data[10] << 8) | range_data[11];
    
    // Limpiar interrupt
    ret = vl53l0x_write_byte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, 0x01);
    
    return ret;
}

/**
 * @brief Mostrar información del sensor
 */
static void mostrar_info_sensor(void)
{
    ESP_LOGI(TAG, "=== INFORMACIÓN DEL SENSOR ===");
    ESP_LOGI(TAG, "Modelo: VL53L0X");
    ESP_LOGI(TAG, "Tipo: Sensor láser ToF (Time of Flight)");
    ESP_LOGI(TAG, "Longitud de onda: 940 nm");
    ESP_LOGI(TAG, "Rango de medición: 40mm - 4000mm");
    ESP_LOGI(TAG, "Resolución: ±1mm");
    ESP_LOGI(TAG, "Campo de visión: 15° - 27°");
    ESP_LOGI(TAG, "Dirección I2C: 0x%02X", VL53L0X_I2C_ADDR);
    ESP_LOGI(TAG, "===============================");
}

/**
 * @brief Tarea principal del sensor
 */
static void vl53l0x_task(void *arg)
{
    uint16_t range_mm;
    int contador = 0;
    
    while (1) {
        esp_err_t ret = vl53l0x_read_range(&range_mm);
        
        if (ret == ESP_OK) {
            float range_cm = range_mm / 10.0;
            
            printf("Distancia: %d mm (%.1f cm)", range_mm, range_cm);
            
            // Indicador visual de proximidad
            if (range_mm < 100) {
                printf(" [MUY CERCA]");
            } else if (range_mm < 300) {
                printf(" [CERCA]");
            } else if (range_mm < 1000) {
                printf(" [MEDIO]");
            } else {
                printf(" [LEJOS]");
            }
            printf("\n");
            
        } else {
            ESP_LOGW(TAG, "Error en medición o fuera de rango");
        }
        
        // Mostrar información cada 20 mediciones
        contador++;
        if (contador >= 20) {
            contador = 0;
            mostrar_info_sensor();
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Esperar 500ms
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== PRUEBA SENSOR VL53L0X ===");
    
    // Inicializar I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado correctamente");
    
    // Esperar un poco para estabilización
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Inicializar sensor VL53L0X
    esp_err_t ret = vl53l0x_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando VL53L0X");
        ESP_LOGE(TAG, "Verificar conexiones:");
        ESP_LOGE(TAG, "VCC -> 3.3V");
        ESP_LOGE(TAG, "GND -> GND");
        ESP_LOGE(TAG, "SDA -> GPIO21");
        ESP_LOGE(TAG, "SCL -> GPIO22");
        return;
    }
    
    ESP_LOGI(TAG, "Sensor VL53L0X inicializado correctamente!");
    mostrar_info_sensor();
    
    // Crear tarea para lectura continua
    xTaskCreate(vl53l0x_task, "vl53l0x_task", 4096, NULL, 5, NULL);
}