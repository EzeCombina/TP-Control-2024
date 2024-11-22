#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_err.h"    

// ADC
#define ADC_WIDTH           12                 // Puede ser un valor entre 9 (0 – 511) y 12 bits (0 – 4095).
#define ADC_CHANNEL         ADC1_CHANNEL_0     // GPIO34 (ADC1_CHANNEL_6) en ESP32 
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define SAMPLES             64
#define BITS8               255
#define BITS9               511
#define BITS12              4095
#define T_MOTOR             24.0

static uint16_t adc_value;

static const char *TAG = "MAIN";

/*

    Valor minimo medido -> 100 mV

    Valor máximo medido -> 3.10 V

    Partiendo de dos puntos de medicion realizados se obtuvo la conversion del valor medido y así
    obtenér el valor que nos interesa a nosotros. 

*/

void app_main(void) {
    
    // Configuración del ancho de bits del ADC
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));

    // Configuración del canal y atenuación del ADC
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));

    float voltage = 0.0;
    int smooth_value = 0;
    //uint16_t convert = (0.15 * BITS12) / 3; 

    while (1) {

        // Leer el valor analógico del canal
        adc_value = adc1_get_raw(ADC_CHANNEL);

        // Promedio para mejorar la medición 
        for(int i = 0; i < SAMPLES; i++)
        {
            smooth_value += adc_value = adc1_get_raw(ADC_CHANNEL);
        }
        smooth_value /= SAMPLES;

        if(smooth_value > BITS12)
        {
            smooth_value = BITS12;
        }

        // Recta para modificar el valor a lo que nos interesa 
        smooth_value *= ((-0.13 * (smooth_value - 2047)) / 2048) + 1.13; 

        voltage = ((smooth_value)*T_MOTOR) / BITS12;

        // Imprimir el valor leído 
        ESP_LOGI(TAG, "Valor Tensión: %.3f", voltage);
        ESP_LOGI(TAG, "Valor leido: %d\n", smooth_value);

        // Esperar 100 milisegundos antes de la siguiente lectura
        vTaskDelay(pdMS_TO_TICKS(500));

    }
}

/*
 *     +----------+-------------+-----------------+
 *     |          | attenuation | suggested range |
 *     |    SoC   |     (dB)    |      (mV)       |
 *     +==========+=============+=================+
 *     |          |       0     |    100 ~  950   |
 *     |          +-------------+-----------------+
 *     |          |       2.5   |    100 ~ 1250   |
 *     |   ESP32  +----------- --+-----------------+
 *     |          |       6     |    150 ~ 1750   |
 *     |          +-------------+-----------------+
 *     |          |    11 - 12  |    150 ~ 2450   |
 *     +----------+-------------+-----------------+
 *     |          |       0     |      0 ~  750   |
 *     |          +-------------+-----------------+
 *     |          |       2.5   |      0 ~ 1050   |
 *     | ESP32-S2 +-------------+-----------------+
 *     |          |       6     |      0 ~ 1300   |
 *     |          +-------------+-----------------+
 *     |          |      11     |      0 ~ 2500   |
 *     +----------+-------------+-----------------+
 */