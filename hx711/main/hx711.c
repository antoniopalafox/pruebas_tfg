#include "hx711.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include <limits.h>

static const char *TAG = "HX711";
static portMUX_TYPE hx711_spinlock = portMUX_INITIALIZER_UNLOCKED;

bool hx711_init(hx711_sensor_t *sensor, int dout_pin, int sck_pin, hx711_gain_t gain) {
    if (sensor == NULL) {
        return false;
    }
    
    sensor->dout_pin = dout_pin;
    sensor->sck_pin = sck_pin;
    sensor->offset = 0;
    sensor->scale = 1.0f;
    sensor->gain = gain;
    
    // Configurar pines
    gpio_config_t io_conf = {};
    
    // Configurar SCK como salida
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << sck_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // Configurar DOUT como entrada
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << dout_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // Inicializar SCK en nivel bajo
    gpio_set_level(sck_pin, 0);
    
    // Despertar el sensor y esperar estabilización
    hx711_power_up(sensor);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Establecer la ganancia
    hx711_set_gain(sensor, gain);
    
    // Esperar a que el sensor esté listo
    int timeout = 0;
    while (gpio_get_level(dout_pin) == 1 && timeout < 500) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timeout++;
    }
    
    if (timeout >= 500) {
        ESP_LOGE(TAG, "Timeout esperando que el sensor esté listo");
        return false;
    }
    
    // Descartar primeras lecturas inestables
    for (int i = 0; i < 5; i++) {
        hx711_read_raw(sensor);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "HX711 inicializado correctamente");
    return true;
}

bool hx711_is_ready(hx711_sensor_t *sensor) {
    if (sensor == NULL) {
        return false;
    }
    return (gpio_get_level(sensor->dout_pin) == 0);
}

int32_t hx711_read_raw(hx711_sensor_t *sensor) {
    if (sensor == NULL) {
        return INT32_MIN;
    }
    
    // Esperar a que el sensor esté listo
    int timeout = 0;
    while (!hx711_is_ready(sensor) && timeout < 100) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        timeout++;
    }
    
    if (timeout >= 100) {
        ESP_LOGW(TAG, "Sensor no está listo para lectura");
        return INT32_MIN;
    }
    
    uint32_t data = 0;
    
    // Deshabilitar interrupciones durante la lectura crítica
    taskENTER_CRITICAL(&hx711_spinlock);
    
    // Leer 24 bits de datos
    for (int i = 23; i >= 0; i--) {
        gpio_set_level(sensor->sck_pin, 1);
        esp_rom_delay_us(2);
        
        if (gpio_get_level(sensor->dout_pin)) {
            data |= (1UL << i);
        }
        
        gpio_set_level(sensor->sck_pin, 0);
        esp_rom_delay_us(2);
    }
    
    // Pulsos adicionales para configurar ganancia
    for (int i = 0; i < sensor->gain; i++) {
        gpio_set_level(sensor->sck_pin, 1);
        esp_rom_delay_us(2);
        gpio_set_level(sensor->sck_pin, 0);
        esp_rom_delay_us(2);
    }
    
    taskEXIT_CRITICAL(&hx711_spinlock);
    
    // Convertir a signed de 24 bits
    if (data & 0x800000) {
        data |= 0xFF000000;
    }
    
    return (int32_t)data;
}

int32_t hx711_read_average(hx711_sensor_t *sensor, int samples) {
    if (sensor == NULL || samples <= 0) {
        return INT32_MIN;
    }
    
    int64_t sum = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < samples; i++) {
        int32_t reading = hx711_read_raw(sensor);
        if (reading != INT32_MIN) {
            sum += reading;
            valid_readings++;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    
    if (valid_readings == 0) {
        ESP_LOGE(TAG, "No se pudieron obtener lecturas válidas");
        return INT32_MIN;
    }
    
    return (int32_t)(sum / valid_readings);
}

float hx711_read_units(hx711_sensor_t *sensor) {
    if (sensor == NULL) {
        return 0.0f;
    }
    
    int32_t raw_value = hx711_read_average(sensor, 3);
    if (raw_value == INT32_MIN) {
        return 0.0f;
    }
    
    return (float)(raw_value - sensor->offset) / sensor->scale;
}

void hx711_tare(hx711_sensor_t *sensor, int samples) {
    if (sensor == NULL || samples <= 0) {
        return;
    }
    
    ESP_LOGI(TAG, "Realizando tara con %d muestras...", samples);
    
    int32_t tare_value = hx711_read_average(sensor, samples);
    if (tare_value != INT32_MIN) {
        sensor->offset = tare_value;
        ESP_LOGI(TAG, "Tara completada. Offset: %ld", sensor->offset);
    } else {
        ESP_LOGE(TAG, "Error al realizar tara");
    }
}

void hx711_calibrate(hx711_sensor_t *sensor, float known_weight, int samples) {
    if (sensor == NULL || samples <= 0 || known_weight <= 0) {
        return;
    }
    
    ESP_LOGI(TAG, "Calibrando con peso conocido: %.2f g", known_weight);
    
    int32_t cal_reading = hx711_read_average(sensor, samples);
    if (cal_reading != INT32_MIN) {
        float raw_difference = (float)(cal_reading - sensor->offset);
        if (raw_difference != 0) {
            sensor->scale = raw_difference / known_weight;
            ESP_LOGI(TAG, "Calibración completada. Escala: %.2f", sensor->scale);
        } else {
            ESP_LOGE(TAG, "Error: No hay diferencia en las lecturas");
        }
    } else {
        ESP_LOGE(TAG, "Error al leer durante calibración");
    }
}

void hx711_set_gain(hx711_sensor_t *sensor, hx711_gain_t gain) {
    if (sensor != NULL && (gain == HX711_GAIN_128 || gain == HX711_GAIN_64 || gain == HX711_GAIN_32)) {
        sensor->gain = gain;
        
        // Descartar lecturas para aplicar nueva ganancia
        for (int i = 0; i < 3; i++) {
            hx711_read_raw(sensor);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        
        ESP_LOGI(TAG, "Ganancia configurada a: %d", gain);
    }
}

void hx711_power_down(hx711_sensor_t *sensor) {
    if (sensor != NULL) {
        gpio_set_level(sensor->sck_pin, 1);
        esp_rom_delay_us(60);
        ESP_LOGI(TAG, "Sensor en modo de bajo consumo");
    }
}

void hx711_power_up(hx711_sensor_t *sensor) {
    if (sensor != NULL) {
        gpio_set_level(sensor->sck_pin, 0);
        ESP_LOGI(TAG, "Sensor despertado");
    }
}

void hx711_debug_info(hx711_sensor_t *sensor) {
    if (sensor == NULL) return;
    
    ESP_LOGI(TAG, "=== HX711 DEBUG INFO ===");
    ESP_LOGI(TAG, "DOUT Pin: %d", sensor->dout_pin);
    ESP_LOGI(TAG, "SCK Pin: %d", sensor->sck_pin);
    ESP_LOGI(TAG, "Offset: %ld", sensor->offset);
    ESP_LOGI(TAG, "Scale: %.6f", sensor->scale);
    ESP_LOGI(TAG, "Gain: %d", sensor->gain);
    ESP_LOGI(TAG, "Sensor Ready: %s", hx711_is_ready(sensor) ? "YES" : "NO");
    
    ESP_LOGI(TAG, "Lecturas RAW recientes:");
    for (int i = 0; i < 5; i++) {
        int32_t raw = hx711_read_raw(sensor);
        ESP_LOGI(TAG, "  Lectura %d: %ld", i+1, raw);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}