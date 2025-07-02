#ifndef HX711_H
#define HX711_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

// Definiciones de ganancia
typedef enum {
    HX711_GAIN_128 = 1,  // Canal A, ganancia 128
    HX711_GAIN_64 = 3,   // Canal A, ganancia 64
    HX711_GAIN_32 = 2    // Canal B, ganancia 32
} hx711_gain_t;

// Estructura del sensor HX711
typedef struct {
    int dout_pin;        // Pin de datos (DOUT)
    int sck_pin;         // Pin de reloj (SCK)
    int32_t offset;      // Valor de tara (sin peso)
    float scale;         // Factor de escala para calibración
    hx711_gain_t gain;   // Ganancia configurada
} hx711_sensor_t;

// Funciones principales
bool hx711_init(hx711_sensor_t *sensor, int dout_pin, int sck_pin, hx711_gain_t gain);
bool hx711_is_ready(hx711_sensor_t *sensor);
int32_t hx711_read_raw(hx711_sensor_t *sensor);
int32_t hx711_read_average(hx711_sensor_t *sensor, int samples);
float hx711_read_units(hx711_sensor_t *sensor);

// Funciones de calibración
void hx711_tare(hx711_sensor_t *sensor, int samples);
void hx711_calibrate(hx711_sensor_t *sensor, float known_weight, int samples);
void hx711_set_gain(hx711_sensor_t *sensor, hx711_gain_t gain);

// Funciones de control de energía
void hx711_power_down(hx711_sensor_t *sensor);
void hx711_power_up(hx711_sensor_t *sensor);

// Función de debug
void hx711_debug_info(hx711_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif // HX711_H