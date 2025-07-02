/**
 * @file vl53l0x.c
 * @brief Implementación del driver para sensor VL53L0X
 */

#include "vl53l0x.h"

// Etiqueta para logs
static const char *TAG = "VL53L0X";

// Registros importantes del VL53L0X
#define REG_IDENTIFICATION_MODEL_ID                    0xC0
#define REG_IDENTIFICATION_REVISION_ID                 0xC2
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV          0x89
#define REG_MSRC_CONFIG_CONTROL                        0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_SYSTEM_SEQUENCE_CONFIG                     0x01
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET          0x4F
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD       0x4E
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT         0xB6
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO              0x0A
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                   0x84
#define REG_SYSTEM_INTERRUPT_CLEAR                    0x0B
#define REG_RESULT_INTERRUPT_STATUS                   0x13
#define REG_SYSRANGE_START                            0x00
#define REG_RESULT_RANGE_STATUS                       0x14
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW        0x47
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH       0x48

// Constantes
#define VL53L0X_EXPECTED_DEVICE_ID                    0xEE
#define VL53L0X_I2C_TIMEOUT_MS                        1000
#define VL53L0X_DEFAULT_ADDRESS                       0x29

// Funciones auxiliares para comunicación I2C
static esp_err_t vl53l0x_write_reg(vl53l0x_sensor_t *sensor, uint8_t reg, uint8_t data) {
    if (!sensor || !sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t write_buf[2] = {reg, data};
    esp_err_t ret = i2c_master_write_to_device(
        sensor->i2c_port, 
        sensor->address, 
        write_buf, 
        2, 
        sensor->timeout_ms / portTICK_PERIOD_MS
    );
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error escribiendo registro 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t vl53l0x_write_reg16(vl53l0x_sensor_t *sensor, uint8_t reg, uint16_t data) {
    if (!sensor || !sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t write_buf[3] = {reg, (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    esp_err_t ret = i2c_master_write_to_device(
        sensor->i2c_port, 
        sensor->address, 
        write_buf, 
        3, 
        sensor->timeout_ms / portTICK_PERIOD_MS
    );
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error escribiendo registro16 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t vl53l0x_read_reg(vl53l0x_sensor_t *sensor, uint8_t reg, uint8_t *data) {
    if (!sensor || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = i2c_master_write_read_device(
        sensor->i2c_port, 
        sensor->address, 
        &reg, 
        1, 
        data, 
        1, 
        sensor->timeout_ms / portTICK_PERIOD_MS
    );
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo registro 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t vl53l0x_read_reg16(vl53l0x_sensor_t *sensor, uint8_t reg, uint16_t *data) {
    if (!sensor || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t read_buf[2];
    esp_err_t ret = i2c_master_write_read_device(
        sensor->i2c_port, 
        sensor->address, 
        &reg, 
        1, 
        read_buf, 
        2, 
        sensor->timeout_ms / portTICK_PERIOD_MS
    );
    
    if (ret == ESP_OK) {
        *data = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    } else {
        ESP_LOGW(TAG, "Error leyendo registro16 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

// Implementación de funciones públicas
bool vl53l0x_test_i2c_connection(i2c_port_t i2c_port, uint8_t address) {
    ESP_LOGI(TAG, "Probando conexión I2C en dirección 0x%02X", address);
    
    uint8_t test_reg = REG_IDENTIFICATION_MODEL_ID;
    uint8_t test_data;
    
    esp_err_t ret = i2c_master_write_read_device(
        i2c_port, 
        address, 
        &test_reg, 
        1, 
        &test_data, 
        1, 
        VL53L0X_I2C_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Conexión I2C exitosa. Device ID: 0x%02X", test_data);
        return true;
    } else {
        ESP_LOGE(TAG, "✗ Error en conexión I2C: %s", esp_err_to_name(ret));
        return false;
    }
}

void vl53l0x_i2c_scanner(i2c_port_t i2c_port) {
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    bool device_found = false;
    
    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Dispositivo encontrado en: 0x%02X", address);
            device_found = true;
        }
    }
    
    if (!device_found) {
        ESP_LOGW(TAG, "  No se encontraron dispositivos I2C");
    }
    
    ESP_LOGI(TAG, "Escaneo completado");
}

bool vl53l0x_init(vl53l0x_sensor_t *sensor, i2c_port_t i2c_port, uint8_t address) {
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Puntero de sensor es NULL");
        return false;
    }
    
    // Inicializar estructura
    sensor->i2c_port = i2c_port;
    sensor->address = address;
    sensor->timeout_ms = VL53L0X_I2C_TIMEOUT_MS;
    sensor->calibration_factor = 1.0f;
    sensor->mode = VL53L0X_MODE_SINGLE;
    sensor->accuracy = VL53L0X_ACCURACY_GOOD;
    sensor->initialized = false;
    
    ESP_LOGI(TAG, "Inicializando sensor VL53L0X en dirección 0x%02X", address);
    
    // Esperar a que el sensor esté listo
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Probar conexión I2C primero
    if (!vl53l0x_test_i2c_connection(i2c_port, address)) {
        ESP_LOGE(TAG, "Error: No se puede conectar con el sensor");
        ESP_LOGI(TAG, "Ejecutando escaneo I2C para diagnóstico:");
        vl53l0x_i2c_scanner(i2c_port);
        return false;
    }
    
    // Marcar como inicializado temporalmente para las funciones auxiliares
    sensor->initialized = true;
    
    // Verificar ID del dispositivo
    uint8_t device_id;
    if (vl53l0x_read_reg(sensor, REG_IDENTIFICATION_MODEL_ID, &device_id) != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo Device ID");
        sensor->initialized = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Device ID leído: 0x%02X (esperado: 0x%02X)", device_id, VL53L0X_EXPECTED_DEVICE_ID);
    
    if (device_id != VL53L0X_EXPECTED_DEVICE_ID) {
        ESP_LOGE(TAG, "Device ID incorrecto");
        sensor->initialized = false;
        return false;
    }
    
    // Leer revision ID para información adicional
    uint8_t revision_id;
    if (vl53l0x_read_reg(sensor, REG_IDENTIFICATION_REVISION_ID, &revision_id) == ESP_OK) {
        ESP_LOGI(TAG, "Revision ID: 0x%02X", revision_id);
    }
    
    // Secuencia de inicialización simplificada
    ESP_LOGI(TAG, "Ejecutando secuencia de inicialización...");
    
    // Reset básico y configuración inicial
    if (vl53l0x_write_reg(sensor, 0x88, 0x00) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0x80, 0x01) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0xFF, 0x01) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0x00, 0x00) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0xFF, 0x00) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0x09, 0x00) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0x10, 0x00) != ESP_OK ||
        vl53l0x_write_reg(sensor, 0x11, 0x00) != ESP_OK) {
        
        ESP_LOGE(TAG, "Error en secuencia de inicialización básica");
        sensor->initialized = false;
        return false;
    }
    
    // Configurar interrupciones
    if (vl53l0x_write_reg(sensor, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) != ESP_OK ||
        vl53l0x_write_reg(sensor, REG_GPIO_HV_MUX_ACTIVE_HIGH, 0x01) != ESP_OK ||
        vl53l0x_write_reg(sensor, REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != ESP_OK) {
        
        ESP_LOGE(TAG, "Error configurando interrupciones");
        sensor->initialized = false;
        return false;
    }
    
    // Configurar precisión por defecto
    if (!vl53l0x_set_accuracy(sensor, VL53L0X_ACCURACY_GOOD)) {
        ESP_LOGW(TAG, "Advertencia: Error configurando precisión por defecto");
    }
    
    // Configurar modo por defecto
    if (!vl53l0x_set_mode(sensor, VL53L0X_MODE_SINGLE)) {
        ESP_LOGW(TAG, "Advertencia: Error configurando modo por defecto");
    }
    
    ESP_LOGI(TAG, "✓ Sensor VL53L0X inicializado correctamente");
    return true;
}

uint16_t vl53l0x_read_distance(vl53l0x_sensor_t *sensor) {
    if (!sensor || !sensor->initialized) {
        ESP_LOGW(TAG, "Sensor no inicializado");
        return 0;
    }
    
    esp_err_t ret;
    uint8_t status;
    uint16_t range_mm = 0;
    uint32_t timeout_count = 0;
    const uint32_t max_timeout = 500; // 5 segundos máximo
    
    // En modo único, iniciar una medición
    if (sensor->mode == VL53L0X_MODE_SINGLE) {
        ret = vl53l0x_write_reg(sensor, REG_SYSRANGE_START, 0x01);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error iniciando medición");
            return 0;
        }
        
        // Esperar a que la medición se complete
        do {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            ret = vl53l0x_read_reg(sensor, REG_RESULT_INTERRUPT_STATUS, &status);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Error leyendo status de interrupción");
                return 0;
            }
            timeout_count++;
        } while ((status & 0x07) == 0 && timeout_count < max_timeout);
        
        if (timeout_count >= max_timeout) {
            ESP_LOGW(TAG, "Timeout esperando medición");
            return 0;
        }
        
        // Leer el resultado del rango
        ret = vl53l0x_read_reg16(sensor, REG_RESULT_RANGE_STATUS + 10, &range_mm);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error leyendo resultado de distancia");
            return 0;
        }
        
        // Limpiar la interrupción
        vl53l0x_write_reg(sensor, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
        
    } else {
        // En modo continuo, simplemente leer el último valor
        ret = vl53l0x_read_reg16(sensor, REG_RESULT_RANGE_STATUS + 10, &range_mm);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error leyendo distancia en modo continuo");
            return 0;
        }
    }
    
    // Verificar si la medición es válida
    if (range_mm == 0xFFFF) {
        ESP_LOGW(TAG, "Medición fuera de rango");
        return 0;
    }
    
    // Aplicar factor de calibración
    uint16_t calibrated_distance = (uint16_t)((float)range_mm * sensor->calibration_factor);
    
    return calibrated_distance;
}

bool vl53l0x_set_mode(vl53l0x_sensor_t *sensor, vl53l0x_mode_t mode) {
    if (!sensor || !sensor->initialized) {
        return false;
    }
    
    ESP_LOGI(TAG, "Configurando modo: %d", mode);
    
    // Detener cualquier medición en curso
    esp_err_t ret = vl53l0x_write_reg(sensor, REG_SYSRANGE_START, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error deteniendo mediciones");
        return false;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    switch (mode) {
        case VL53L0X_MODE_SINGLE:
            // Modo single shot por defecto, no necesita configuración adicional
            ESP_LOGI(TAG, "Modo single shot configurado");
            break;
            
        case VL53L0X_MODE_CONTINUOUS:
            // Iniciar modo continuo
            ret = vl53l0x_write_reg(sensor, REG_SYSRANGE_START, 0x02);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Error configurando modo continuo");
                return false;
            }
            ESP_LOGI(TAG, "Modo continuo configurado");
            break;
            
        case VL53L0X_MODE_TIMED:
            // Configurar modo temporizado
            ret = vl53l0x_write_reg(sensor, REG_SYSRANGE_START, 0x03);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Error configurando modo temporizado");
                return false;
            }
            ESP_LOGI(TAG, "Modo temporizado configurado");
            break;
            
        default:
            ESP_LOGW(TAG, "Modo no válido: %d", mode);
            return false;
    }
    
    sensor->mode = mode;
    return true;
}

bool vl53l0x_set_accuracy(vl53l0x_sensor_t *sensor, vl53l0x_accuracy_t accuracy) {
    if (!sensor || !sensor->initialized) {
        return false;
    }
    
    ESP_LOGI(TAG, "Configurando precisión: %d", accuracy);
    
    uint16_t return_limit;
    
    switch (accuracy) {
        case VL53L0X_ACCURACY_FASTEST:
            // Más rápido (~10ms)
            vl53l0x_write_reg(sensor, REG_MSRC_CONFIG_CONTROL, 0x18);
            return_limit = 0x1400; // ~0.5 MCPS
            break;
            
        case VL53L0X_ACCURACY_FAST:
            // Rápido (~20ms)
            vl53l0x_write_reg(sensor, REG_MSRC_CONFIG_CONTROL, 0x1C);
            return_limit = 0x1400; // ~0.5 MCPS
            break;
            
        case VL53L0X_ACCURACY_GOOD:
            // Precisión estándar (~30ms)
            vl53l0x_write_reg(sensor, REG_MSRC_CONFIG_CONTROL, 0x1D);
            return_limit = 0x1400; // ~0.5 MCPS
            break;
            
        case VL53L0X_ACCURACY_BETTER:
            // Mejor precisión (~70ms)
            vl53l0x_write_reg(sensor, REG_MSRC_CONFIG_CONTROL, 0x1E);
            return_limit = 0x0A00; // ~0.25 MCPS
            break;
            
        case VL53L0X_ACCURACY_BEST:
            // La mejor precisión (~200ms)
            vl53l0x_write_reg(sensor, REG_MSRC_CONFIG_CONTROL, 0x1F);
            return_limit = 0x0A00; // ~0.25 MCPS
            break;
            
        default:
            ESP_LOGW(TAG, "Nivel de precisión no válido: %d", accuracy);
            return false;
    }
    
    // Establecer límite de tasa de retorno
    vl53l0x_write_reg16(sensor, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, return_limit);
    
    // Configurar parámetros de fase
    vl53l0x_write_reg16(sensor, REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    vl53l0x_write_reg16(sensor, REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x78);
    
    sensor->accuracy = accuracy;
    ESP_LOGI(TAG, "Precisión configurada correctamente");
    return true;
}

void vl53l0x_set_calibration(vl53l0x_sensor_t *sensor, float factor) {
    if (sensor != NULL && factor > 0) {
        sensor->calibration_factor = factor;
        ESP_LOGI(TAG, "Factor de calibración establecido: %.3f", factor);
    }
}

bool vl53l0x_wake_up(vl53l0x_sensor_t *sensor) {
    if (!sensor || !sensor->initialized) {
        return false;
    }
    
    ESP_LOGI(TAG, "Despertando sensor");
    return vl53l0x_write_reg(sensor, REG_SYSRANGE_START, 0x00) == ESP_OK;
}

bool vl53l0x_sleep(vl53l0x_sensor_t *sensor) {
    if (!sensor || !sensor->initialized) {
        return false;
    }
    
    ESP_LOGI(TAG, "Poniendo sensor en modo bajo consumo");
    return vl53l0x_write_reg(sensor, REG_SYSRANGE_START, 0x00) == ESP_OK;
}