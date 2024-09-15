#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_err.h"  
#include "driver/ledc.h"
#include "esp_err.h"

// ADC
// #define ADC_WIDTH           12                 // Puede ser un valor entre 9 (0 – 511) y 12 bits (0 – 4095).
// #define ADC_CHANNEL         ADC1_CHANNEL_0     // GPIO34 (ADC1_CHANNEL_6) en ESP32 
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SAMPLES             64
// #define BITS8               255
// #define BITS9               511
// #define BITS12              4095
// #define T_MOTOR             24.0

#define DIRECTO         0
#define MULTISAMPLING   1
#define ESCALON         2
#define GRAFICA         0
#define HISTOGRAMA      1

#define ADC_WIDTH           12                  // Puede ser un valor entre 9 (0 – 511) y 12 bits (0 – 4095).
#define FILTRO              1000
#define ADC_CHANNEL         ADC1_CHANNEL_4      // Corresponde al GPIO 32
#define ADC_ATTEN           ADC_ATTEN_DB_12     // Atenuación de 12 dB en el voltaje de entrada para usar el ADC
#define MUESTRAS            64                  // Muestras para obtener un promedio del valor del ADC
#define T_MOTOR             21.12
#define BITS12              4096

int modo                        = ESCALON;              // Elegir que tipo de técnica se le aplica a la señal del ADC para obtener un mejor valor
int visualizar                  = GRAFICA;              // Forma de presentar los valores del ADC
int LecturaCruda                = 0;                    // Lectura del ADC
int LecturaSuavizada            = 0;                    // Valor promedio de la lectura cruda
int LecturaFiltrada             = 0;                    // VAlor filtrado de la lectura cruda
int Lectura                     = 0;
double filtro                   = 0;                    // Variable para generar el filtro
int veces[4096];                                        // Contar las cantidad de veces que aparece la misma lectura del ACD
uint64_t millisAnt              = 0;                    // Toma el valor del momento
float Vout_filtrada_corregida   = 0;
float Vout_motor                = 0;

// PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5)                                     // Pin de salida del PWM 
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT                        // Resolución del PWM (8 bits)
#define LEDC_DUTY_PERCENT       20                                     // Duty porcentual
#define LEDC_DUTY               ((LEDC_DUTY_PERCENT*255)/100)            // Duty al 50%. (2 ** 8) * 50% = 128
#define LEDC_FREQUENCY          (1000)                                  // Frequency in Hertz. Set frequency at 1 kHz

#define PROCESADORA     0
#define PROCESADORB     1

/*==================[Variables]======================*/

static const char *TAG = "MAIN";

/*==================[Prototipos de funciones]======================*/
void TaskADC(void *taskParmPtr);       //Prototipo de la función de la tarea 

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

    // ---PWM---
    example_ledc_init();
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
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

    for(int i = 0; i <= 4096; i++)
    {
        veces[i]=0;
    }

    while (1) {

        if(modo == DIRECTO)
        {
            LecturaCruda = adc1_get_raw(ADC_CHANNEL);
        }
        else if(modo == MULTISAMPLING)
        {
            for(int i = 0; i < MUESTRAS; i++)
            {
                LecturaSuavizada += LecturaCruda = adc1_get_raw(ADC_CHANNEL);
            }
            LecturaSuavizada /= MUESTRAS;
        }
        else if(modo == ESCALON)
        {
            LecturaCruda = adc1_get_raw(ADC_CHANNEL);    
            filtro += ((double)LecturaCruda - filtro) / (double)FILTRO;
            LecturaFiltrada = (int)filtro;
        }

        //Aumenta el número de veces que aparece una lectura del ADC
        veces[Lectura]++;
        LecturaFiltrada *= ((-0.13 * (LecturaFiltrada - 2047)) / 2048) + 1.13;
        Vout_filtrada_corregida = (LecturaFiltrada * 3.22) / 4096;
        Vout_motor = ((LecturaFiltrada)*T_MOTOR) / BITS12;
        //vTaskDelay(pdMS_TO_TICKS(100));

        //Visualizar el resultado de la conversión del ADC en función del modo escogido
        if(visualizar == HISTOGRAMA)
        {
            //El hstograma se actualiza cada 1 s
            if(esp_timer_get_time() == millisAnt + 1000*1000)
            {
                for(int i = (1600 - 25); i < (1600 + 25); i++)
                {
                    ESP_LOGI(TAG, "%d\n", veces[i]);
                }
                for(int i = 0; i < 4096; i++)
                {
                    veces[i] = 0;
                }
                millisAnt = esp_timer_get_time();
            }
        }
        else if(visualizar == GRAFICA)
        {
            //Se muestra un punto en SerialPloter cada décima de segundo
            if(esp_timer_get_time() > millisAnt + 100*1000)
            {
                ESP_LOGI(TAG, "Lectura Cruda: %d\n", LecturaCruda);
                
                if(modo == MULTISAMPLING)
                {
                    ESP_LOGI(TAG, "Lectura Suavizada: %d\n", LecturaSuavizada);
                }
            
                if(modo == ESCALON)
                {
                    ESP_LOGI(TAG, "Lectura Filtrada: %d\n", LecturaFiltrada);
                }
                millisAnt = esp_timer_get_time();
                ESP_LOGI(TAG, "Vout_filtrada_corregida: %.03f", Vout_filtrada_corregida);
                ESP_LOGI(TAG, "Vout_motor: %.03f", Vout_motor);
            }
        }
    }
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

