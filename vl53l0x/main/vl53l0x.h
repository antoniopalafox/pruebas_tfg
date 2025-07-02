/**
 * @file vl53l0x.h
 * @brief Driver para sensor de distancia láser VL53L0X
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modos de operación del sensor
 */
typedef enum {
    VL53L0X_MODE_SINGLE,        ///< Medición única
    VL53L0X_MODE_CONTINUOUS,    ///< Medición continua
    VL53L0X_MODE_TIMED          ///< Medición temporizada
} vl53l0x_mode_t;

/**
 * @brief Niveles de precisión del sensor
 */
typedef enum {
    VL53L0X_ACCURACY_FASTEST,   ///< Más rápido, menos preciso (~10ms)
    VL53L0X_ACCURACY_FAST,      ///< Rápido, precisión estándar (~20ms)
    VL53L0X_ACCURACY_GOOD,      ///< Buena precisión (~30ms)
    VL53L0X_ACCURACY_BETTER,    ///< Mejor precisión (~70ms)
    VL53L0X_ACCURACY_BEST       ///< La mejor precisión (~200ms)
} vl53l0x_accuracy_t;

/**
 * @brief Estructura del sensor VL53L0X
 */
typedef struct {
    i2c_port_t i2c_port;           ///< Puerto I2C
    uint8_t address;               ///< Dirección I2C del sensor
    uint32_t timeout_ms;           ///< Timeout para operaciones I2C
    float calibration_factor;      ///< Factor de calibración
    vl53l0x_mode_t mode;          ///< Modo de operación actual
    vl53l0x_accuracy_t accuracy;   ///< Nivel de precisión actual
    bool initialized;              ///< Estado de inicialización
} vl53l0x_sensor_t;

/**
 * @brief Inicializar el sensor VL53L0X
 */
bool vl53l0x_init(vl53l0x_sensor_t *sensor, i2c_port_t i2c_port, uint8_t address);

/**
 * @brief Leer distancia del sensor
 */
uint16_t vl53l0x_read_distance(vl53l0x_sensor_t *sensor);

/**
 * @brief Configurar modo de operación
 */
bool vl53l0x_set_mode(vl53l0x_sensor_t *sensor, vl53l0x_mode_t mode);

/**
 * @brief Configurar precisión del sensor
 */
bool vl53l0x_set_accuracy(vl53l0x_sensor_t *sensor, vl53l0x_accuracy_t accuracy);

/**
 * @brief Configurar factor de calibración
 */
void vl53l0x_set_calibration(vl53l0x_sensor_t *sensor, float factor);

/**
 * @brief Probar conexión I2C con el sensor
 */
bool vl53l0x_test_i2c_connection(i2c_port_t i2c_port, uint8_t address);

/**
 * @brief Escanear bus I2C para encontrar dispositivos
 */
void vl53l0x_i2c_scanner(i2c_port_t i2c_port);

/**
 * @brief Despertar el sensor
 */
bool vl53l0x_wake_up(vl53l0x_sensor_t *sensor);

/**
 * @brief Poner el sensor en modo bajo consumo
 */
bool vl53l0x_sleep(vl53l0x_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif // VL53L0X_H
