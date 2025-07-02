/**
 * @file main.c
 * @brief Diagnóstico completo para VL53L0X
 */

/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "vl53l0x.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define VL53L0X_ADDRESS             0x29

vl53l0x_sensor_t laser_sensor;

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

// Escaneo I2C detallado
void detailed_i2c_scan(void) {
    printf("\n=== ESCANEO I2C DETALLADO ===\n");
    printf("Buscando dispositivos en el bus I2C...\n");
    
    bool device_found = false;
    
    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("✓ DISPOSITIVO ENCONTRADO en dirección: 0x%02X\n", address);
            device_found = true;
            
            // Si encontramos el VL53L0X en dirección diferente
            if (address != VL53L0X_ADDRESS) {
                printf("  ⚠️  Esta NO es la dirección esperada (0x%02X)\n", VL53L0X_ADDRESS);
            } else {
                printf("  ✓ Esta ES la dirección correcta del VL53L0X!\n");
            }
        }
    }
    
    if (!device_found) {
        printf("❌ NO SE ENCONTRARON DISPOSITIVOS I2C\n");
        printf("\nPOSIBLES CAUSAS:\n");
        printf("1. VCC no conectado a 3.3V\n");
        printf("2. GND no conectado\n");
        printf("3. SCL no conectado a GPIO 22\n");
        printf("4. SDA no conectado a GPIO 21\n");
        printf("5. Cables defectuosos\n");
        printf("6. Sensor dañado\n");
    }
    printf("===============================\n\n");
}

// Test de diferentes velocidades I2C
void test_i2c_speeds(void) {
    printf("=== TEST DE VELOCIDADES I2C ===\n");
    
    uint32_t speeds[] = {100000, 400000, 1000000}; // 100kHz, 400kHz, 1MHz
    const char* speed_names[] = {"100 kHz", "400 kHz", "1 MHz"};
    
    for (int i = 0; i < 3; i++) {
        printf("Probando velocidad: %s\n", speed_names[i]);
        
        // Reconfigurar I2C
        i2c_driver_delete(I2C_MASTER_NUM);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = speeds[i],
            .clk_flags = 0,
        };
        
        esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (ret == ESP_OK) {
            ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        }
        
        if (ret == ESP_OK) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // Probar comunicación
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            
            esp_err_t test_ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (test_ret == ESP_OK) {
                printf("  ✓ FUNCIONA a %s\n", speed_names[i]);
            } else {
                printf("  ❌ Error a %s: %s\n", speed_names[i], esp_err_to_name(test_ret));
            }
        } else {
            printf("  ❌ Error configurando %s\n", speed_names[i]);
        }
    }
    printf("===============================\n\n");
}

void sensor_task(void *pvParameters) {
    printf("\n🔍 DIAGNÓSTICO COMPLETO VL53L0X\n");
    printf("================================\n");
    
    printf("Configuración actual:\n");
    printf("  SCL: GPIO %d\n", I2C_MASTER_SCL_IO);
    printf("  SDA: GPIO %d\n", I2C_MASTER_SDA_IO);
    printf("  Dirección esperada: 0x%02X\n", VL53L0X_ADDRESS);
    printf("  Velocidad I2C: %d Hz\n\n", I2C_MASTER_FREQ_HZ);
    
    // Esperar estabilización
    printf("Esperando estabilización del sistema...\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Escaneo detallado
    detailed_i2c_scan();
    
    // Test de velocidades
    test_i2c_speeds();
    
    // Restaurar configuración original
    i2c_driver_delete(I2C_MASTER_NUM);
    i2c_master_init();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    printf("=== INTENTANDO INICIALIZACIÓN ===\n");
    if (!vl53l0x_init(&laser_sensor, I2C_MASTER_NUM, VL53L0X_ADDRESS)) {
        printf("❌ Inicialización FALLÓ\n");
        printf("\nRECOMENDACIONES:\n");
        printf("1. Verificar que VCC está conectado a 3.3V (NO 5V)\n");
        printf("2. Verificar todas las conexiones con multímetro\n");
        printf("3. Probar con cables más cortos\n");
        printf("4. Verificar que el sensor no esté dañado\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("✓ Inicialización EXITOSA!\n");
    vl53l0x_set_accuracy(&laser_sensor, VL53L0X_ACCURACY_GOOD);
    
    printf("\n=== MEDICIONES ===\n");
    while (1) {
        uint16_t distance = vl53l0x_read_distance(&laser_sensor);
        
        if (distance > 0) {
            printf("%u mm\n", distance);
        } else {
            printf("Error\n");
        }
        
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    nvs_flash_init();
    
    if (i2c_master_init() != ESP_OK) {
        printf("❌ Error inicializando I2C\n");
        return;
    }
    
    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);
}*/

/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0

static esp_err_t i2c_init_with_config(uint32_t freq, bool pullup) {
    // Eliminar driver previo si existe
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .scl_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .master.clk_speed = freq,
        .clk_flags = 0,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void test_i2c_scan_only(const char* description) {
    printf("\n=== %s ===\n", description);
    
    uint8_t found_devices[10];
    int found_count = 0;
    
    for (uint8_t address = 1; address < 127 && found_count < 10; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("  ✓ Dispositivo: 0x%02X\n", address);
            found_devices[found_count++] = address;
        }
    }
    
    printf("  Total encontrados: %d\n", found_count);
    
    // Probar lectura simple en cada dispositivo
    for (int i = 0; i < found_count; i++) {
        uint8_t addr = found_devices[i];
        printf("  Probando lectura en 0x%02X: ", addr);
        
        // Intentar leer 1 byte desde registro 0x00
        uint8_t reg = 0x00;
        uint8_t data;
        esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, addr, 
                                                   &reg, 1, &data, 1, 
                                                   500 / portTICK_PERIOD_MS);
        
        if (ret == ESP_OK) {
            printf("✓ Éxito (0x%02X)\n", data);
        } else {
            printf("❌ %s\n", esp_err_to_name(ret));
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void test_different_speeds() {
    uint32_t speeds[] = {50000, 100000, 200000, 400000};
    const char* speed_names[] = {"50 kHz", "100 kHz", "200 kHz", "400 kHz"};
    
    for (int i = 0; i < 4; i++) {
        printf("\n======================================\n");
        printf("PROBANDO VELOCIDAD: %s\n", speed_names[i]);
        
        if (i2c_init_with_config(speeds[i], true) == ESP_OK) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            test_i2c_scan_only(speed_names[i]);
        } else {
            printf("❌ Error configurando %s\n", speed_names[i]);
        }
    }
}

void test_pullup_configurations() {
    printf("\n=====================================\n");
    printf("PROBANDO CONFIGURACIONES DE PULL-UP\n");
    printf("=====================================\n");
    
    // Test con pull-ups internos habilitados
    printf("\n--- CON PULL-UPS INTERNOS ---\n");
    if (i2c_init_with_config(100000, true) == ESP_OK) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        test_i2c_scan_only("Pull-ups internos ON");
    }
    
    // Test sin pull-ups internos
    printf("\n--- SIN PULL-UPS INTERNOS ---\n");
    if (i2c_init_with_config(100000, false) == ESP_OK) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        test_i2c_scan_only("Pull-ups internos OFF");
    }
}

void test_vl53l0x_specific() {
    printf("\n=====================================\n");
    printf("TEST ESPECÍFICO PARA VL53L0X\n");
    printf("=====================================\n");
    
    // Probar direcciones típicas de VL53L0X
    uint8_t vl53l0x_addresses[] = {0x29, 0x52, 0x54};
    
    for (int i = 0; i < 3; i++) {
        uint8_t addr = vl53l0x_addresses[i];
        printf("\nProbando dirección VL53L0X típica: 0x%02X\n", addr);
        
        // Test de conectividad básica
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("  ✓ Responde en 0x%02X\n", addr);
            
            // Intentar leer registro de identificación
            uint8_t reg = 0xC0;  // Model ID register
            uint8_t model_id;
            ret = i2c_master_write_read_device(I2C_MASTER_NUM, addr, 
                                             &reg, 1, &model_id, 1, 
                                             1000 / portTICK_PERIOD_MS);
            
            if (ret == ESP_OK) {
                printf("  ✓ Model ID: 0x%02X", model_id);
                if (model_id == 0xEE) {
                    printf(" ← ¡ESTE ES VL53L0X!\n");
                } else {
                    printf(" (no es VL53L0X)\n");
                }
            } else {
                printf("  ❌ Error leyendo Model ID: %s\n", esp_err_to_name(ret));
            }
        } else {
            printf("  ❌ No responde en 0x%02X\n", addr);
        }
    }
}

void check_gpio_status() {
    printf("\n=====================================\n");
    printf("VERIFICACIÓN DE ESTADO GPIO\n");
    printf("=====================================\n");
    
    // Leer estado actual de los pines
    int scl_level = gpio_get_level(I2C_MASTER_SCL_IO);
    int sda_level = gpio_get_level(I2C_MASTER_SDA_IO);
    
    printf("Estado actual de pines:\n");
    printf("  SCL (GPIO %d): %s\n", I2C_MASTER_SCL_IO, scl_level ? "HIGH" : "LOW");
    printf("  SDA (GPIO %d): %s\n", I2C_MASTER_SDA_IO, sda_level ? "HIGH" : "LOW");
    
    if (scl_level == 0 || sda_level == 0) {
        printf("\n⚠️  PROBLEMA: Uno o ambos pines están en LOW\n");
        printf("    Esto indica:\n");
        printf("    - Pull-ups insuficientes\n");
        printf("    - Dispositivo manteniendo línea baja\n");
        printf("    - Cortocircuito\n");
    } else {
        printf("\n✓ Ambos pines están en HIGH (estado idle correcto)\n");
    }
}

void app_main(void) {
    printf("\n🔧 DIAGNÓSTICO I2C AVANZADO\n");
    printf("===========================\n");
    printf("Análisis detallado de problemas I2C\n\n");
    
    // Verificar estado inicial de GPIO
    check_gpio_status();
    
    // Probar diferentes configuraciones de pull-up
    test_pullup_configurations();
    
    // Probar diferentes velocidades
    test_different_speeds();
    
    // Test específico para VL53L0X
    test_vl53l0x_specific();
    
    printf("\n=====================================\n");
    printf("RECOMENDACIONES BASADAS EN RESULTADOS:\n");
    printf("=====================================\n");
    printf("1. Si alguna configuración permitió lecturas:\n");
    printf("   → Usar esa configuración específica\n");
    printf("2. Si ninguna funcionó:\n");
    printf("   → Añadir resistencias pull-up externas (4.7kΩ)\n");
    printf("   → Verificar conexiones físicas\n");
    printf("   → Probar con cables más cortos\n");
    printf("3. Si encontró VL53L0X:\n");
    printf("   → Anotar la dirección que funcionó\n");
    printf("=====================================\n");
    
    // Mantener programa ejecutándose
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}*/
/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define SCL_PIN 22
#define SDA_PIN 21

void test_gpio_without_i2c(void) {
    printf("\n🔧 TEST GPIO SIN I2C (SENSOR DESCONECTADO)\n");
    printf("==========================================\n");
    
    // Configurar como entrada con pull-up
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SCL_PIN) | (1ULL << SDA_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    int scl_level = gpio_get_level(SCL_PIN);
    int sda_level = gpio_get_level(SDA_PIN);
    
    printf("Con pull-ups internos:\n");
    printf("  SCL (GPIO %d): %s\n", SCL_PIN, scl_level ? "HIGH ✓" : "LOW ❌");
    printf("  SDA (GPIO %d): %s\n", SDA_PIN, sda_level ? "HIGH ✓" : "LOW ❌");
    
    if (scl_level && sda_level) {
        printf("\n✅ PINES OK SIN SENSOR\n");
        printf("Problema: Pull-ups insuficientes cuando sensor conectado\n");
        printf("SOLUCIÓN: Añadir resistencias pull-up externas\n");
    } else {
        printf("\n❌ PINES DEFECTUOSOS\n");
        printf("Problema: GPIO del ESP32 dañados o configuración incorrecta\n");
        printf("SOLUCIÓN: Usar otros pines GPIO\n");
    }
}

void test_alternative_pins(void) {
    printf("\n🔄 PROBANDO PINES ALTERNATIVOS\n");
    printf("==============================\n");
    
    int alternative_pins[][2] = {
        {18, 19},
        {16, 17}, 
        {25, 26},
        {32, 33}
    };
    
    for (int i = 0; i < 4; i++) {
        int scl_alt = alternative_pins[i][0];
        int sda_alt = alternative_pins[i][1];
        
        printf("\nProbando SCL:%d, SDA:%d\n", scl_alt, sda_alt);
        
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << scl_alt) | (1ULL << sda_alt),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&io_conf);
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
        
        int scl_level = gpio_get_level(scl_alt);
        int sda_level = gpio_get_level(sda_alt);
        
        printf("  SCL:%s SDA:%s", 
               scl_level ? "HIGH" : "LOW",
               sda_level ? "HIGH" : "LOW");
        
        if (scl_level && sda_level) {
            printf(" ✅ ESTOS PINES FUNCIONAN\n");
        } else {
            printf(" ❌\n");
        }
    }
}

void app_main(void) {
    printf("\n⚠️  INSTRUCCIONES IMPORTANTES ⚠️\n");
    printf("================================\n");
    printf("1. DESCONECTA COMPLETAMENTE el sensor VL53L0X\n");
    printf("2. Deja solo el ESP32 conectado por USB\n");
    printf("3. Este test verificará los GPIO sin sensor\n");
    printf("================================\n\n");
    
    printf("Esperando 5 segundos para desconectar sensor...\n");
    for (int i = 5; i > 0; i--) {
        printf("%d...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    test_gpio_without_i2c();
    test_alternative_pins();
    
    printf("\n📋 PRÓXIMOS PASOS:\n");
    printf("=================\n");
    printf("1. Si GPIO 22/21 funcionan sin sensor:\n");
    printf("   → Añadir resistencias pull-up externas\n");
    printf("   → O usar pines alternativos que funcionen\n\n");
    
    printf("2. Si GPIO 22/21 NO funcionan:\n");
    printf("   → Usar pines alternativos\n");
    printf("   → Cambiar configuración en el código\n\n");
    
    printf("3. Después de elegir pines:\n");
    printf("   → Reconectar sensor\n");
    printf("   → Probar nuevamente\n");
    printf("=================\n");
}
*/
/**
 * @file test_conexiones.c
 * @brief Verificación paso a paso de conexiones
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_SCL_PIN    18
#define I2C_SDA_PIN    19
#define XSHUT_PIN      23
#define I2C_MASTER_NUM 0

static esp_err_t i2c_init_simple(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void test_step_by_step(void) {
    printf("\n🔧 VERIFICACIÓN PASO A PASO\n");
    printf("===========================\n");
    printf("Conecta los cables UNO POR UNO y presiona Enter\n");
    printf("===========================\n");
    
    printf("\n📋 PASO 1: Solo conectar VCC y GND\n");
    printf("-----------------------------------\n");
    printf("Conecta SOLO:\n");
    printf("  Sensor VCC  → ESP32 3.3V\n");
    printf("  Sensor GND  → ESP32 GND\n");
    printf("NO conectes SCL, SDA, ni XSHUT todavía\n");
    printf("Presiona Enter cuando esté listo...\n");
    
    // Esperar entrada (simulado con delay)
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    printf("✅ Paso 1 completado\n");
    printf("¿Se enciende algún LED en el sensor? (Anota la respuesta)\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    printf("\n📋 PASO 2: Conectar XSHUT\n");
    printf("--------------------------\n");
    printf("Añade SOLO:\n");
    printf("  Sensor XSHUT → ESP32 GPIO 23\n");
    printf("Presiona Enter cuando esté listo...\n");
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    // Configurar XSHUT
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << XSHUT_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(XSHUT_PIN, 1);
    
    printf("✅ XSHUT configurado en HIGH\n");
    printf("¿Cambia algo en el sensor? (LED se enciende/apaga?)\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    printf("\n📋 PASO 3: Conectar SCL\n");
    printf("------------------------\n");
    printf("Añade SOLO:\n");
    printf("  Sensor SCL → ESP32 GPIO 18\n");
    printf("Presiona Enter cuando esté listo...\n");
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    // Verificar GPIO después de conectar SCL
    int scl_level = gpio_get_level(I2C_SCL_PIN);
    printf("Estado SCL después de conectar: %s\n", scl_level ? "HIGH ✅" : "LOW ❌");
    
    if (!scl_level) {
        printf("⚠️ PROBLEMA: SCL se fue a LOW\n");
        printf("Esto indica:\n");
        printf("  - Cable SCL con cortocircuito\n");
        printf("  - Sensor defectuoso\n");
        printf("  - Conexión incorrecta\n");
        return;
    }
    
    printf("\n📋 PASO 4: Conectar SDA\n");
    printf("------------------------\n");
    printf("Añade FINALMENTE:\n");
    printf("  Sensor SDA → ESP32 GPIO 19\n");
    printf("Presiona Enter cuando esté listo...\n");
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    // Verificar GPIO después de conectar SDA
    scl_level = gpio_get_level(I2C_SCL_PIN);
    int sda_level = gpio_get_level(I2C_SDA_PIN);
    
    printf("Estado final:\n");
    printf("  SCL: %s\n", scl_level ? "HIGH ✅" : "LOW ❌");
    printf("  SDA: %s\n", sda_level ? "HIGH ✅" : "LOW ❌");
    
    if (!scl_level || !sda_level) {
        printf("\n❌ PROBLEMA IDENTIFICADO\n");
        printf("Una línea I2C se fue a LOW al conectar\n");
        printf("Soluciones:\n");
        printf("  1. Cambiar cable defectuoso\n");
        printf("  2. Usar breadboard diferente\n");
        printf("  3. El sensor está realmente defectuoso\n");
        return;
    }
    
    printf("\n✅ TODAS LAS CONEXIONES OK\n");
    printf("Procediendo a test I2C...\n");
}

void test_i2c_final(void) {
    printf("\n🔍 TEST I2C FINAL\n");
    printf("==================\n");
    
    if (i2c_init_simple() != ESP_OK) {
        printf("❌ Error inicializando I2C\n");
        return;
    }
    
    printf("Escaneando I2C...\n");
    bool found = false;
    
    for (uint8_t addr = 0x20; addr < 0x60; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 500 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("✅ Dispositivo encontrado: 0x%02X", addr);
            if (addr == 0x29) printf(" ← ¡VL53L0X!");
            printf("\n");
            found = true;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    if (!found) {
        printf("❌ NO se encontraron dispositivos\n");
        printf("\nPOSIBLES CAUSAS:\n");
        printf("1. Ambos sensores defectuosos\n");
        printf("2. Conexiones aún incorrectas\n");
        printf("3. Sensores falsificados\n");
        printf("4. Necesita resistencias pull-up externas\n");
    } else {
        printf("\n🎉 ¡SENSOR DETECTADO!\n");
        printf("El problema estaba en las conexiones\n");
    }
}

void app_main(void) {
    printf("\n🔧 VERIFICACIÓN CONEXIONES PASO A PASO\n");
    printf("======================================\n");
    printf("Vamos a conectar el sensor paso a paso\n");
    printf("para identificar exactamente dónde está el problema\n");
    printf("======================================\n");
    
    // Asegurar que GPIO estén en estado conocido
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << I2C_SCL_PIN) | (1ULL << I2C_SDA_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    test_step_by_step();
    test_i2c_final();
    
    printf("\n📋 RESUMEN:\n");
    printf("===========\n");
    printf("Si encontró dispositivo: ¡Problema solucionado!\n");
    printf("Si NO encontró: Sensores defectuosos o necesita pull-ups externos\n");
    printf("===========\n");
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}