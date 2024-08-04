#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_err.h"  
#include "driver/ledc.h"
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

// PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5)                                     // Pin de salida del PWM 
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT                        // Resolución del PWM (8 bits)
#define LEDC_DUTY_PERCENT       100                                     // Duty porcentual
#define LEDC_DUTY               ((LEDC_DUTY_PERCENT*32)/100)            // Duty al 50%. (2 ** 8) * 50% = 128
#define LEDC_FREQUENCY          (1000)                                  // Frequency in Hertz. Set frequency at 1 kHz

#define PROCESADORA     0
#define PROCESADORB     1

/*==================[Variables]======================*/
static uint16_t adc_value;

static const char *TAG = "MAIN";

/*==================[Prototipos de funciones]======================*/
void TaskADC(void *taskParmPtr);       //Prototipo de la función de la tarea 
void TaskPWM(void *taskParmPtr);

static void example_ledc_init(void);

void app_main(void)
{
    BaseType_t errA = xTaskCreatePinnedToCore(
        TaskADC,                     	        // Funcion de la tarea a ejecutar
        "TaskADC",   	                        // Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 		    // Cantidad de stack de la tarea
        NULL,                          	        // Parametros de tarea
        tskIDLE_PRIORITY+1,         	        // Prioridad de la tarea -> Queremos que este un nivel encima de IDLE
        NULL,                          		    // Puntero a la tarea creada en el sistema
        PROCESADORA                             // Numero de procesador
    );

    // Gestion de errores
    if(errA == pdFAIL)
    {
        ESP_LOGI(TAG, "Error al crear la tarea.");
        while(1);    // Si no pudo crear la tarea queda en un bucle infinito
    }else{ESP_LOGI(TAG, "Tarea creada correctamente");}

    BaseType_t errB = xTaskCreatePinnedToCore(
        TaskPWM,                     	        // Funcion de la tarea a ejecutar
        "TaskPWM",   	                        // Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 		    // Cantidad de stack de la tarea
        NULL,                          	        // Parametros de tarea
        tskIDLE_PRIORITY+1,         	        // Prioridad de la tarea -> Queremos que este un nivel encima de IDLE
        NULL,                          		    // Puntero a la tarea creada en el sistema
        PROCESADORB                             // Numero de procesador
    );

    // Gestion de errores
    if(errB == pdFAIL)
    {
        ESP_LOGI(TAG, "Error al crear la tarea.");
        while(1);    // Si no pudo crear la tarea queda en un bucle infinito
    }else{ESP_LOGI(TAG, "Tarea creada correctamente");}
}

/*==================[Implementacion de la tarea]======================*/
void TaskADC(void *taskParmPtr)
{
    /*==================[Configuraciones]======================*/
    //uint8_t i = (uint8_t) taskParmPtr; // Recibimos el indice led correspondiente a la tarea
    
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

void TaskPWM(void *taskParmPtr)
{
    /*==================[Configuraciones]======================*/
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    while(1){vTaskDelay(pdMS_TO_TICKS(100));};
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 1 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

