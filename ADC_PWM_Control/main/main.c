// #include <stdio.h>
// #include <math.h>
// #include <float.h> 
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/FreeRTOSConfig.h"
// #include "driver/ledc.h"
// #include "driver/adc.h"
// #include "driver/timer.h"
// #include "esp_timer.h"
// #include "esp_log.h"
// #include "esp_err.h" 

// // ADC
// // #define ADC_WIDTH           12                 // Puede ser un valor entre 9 (0 – 511) y 12 bits (0 – 4095).
// // #define ADC_CHANNEL         ADC1_CHANNEL_0     // GPIO34 (ADC1_CHANNEL_6) en ESP32 
// // #define ADC_ATTEN           ADC_ATTEN_DB_12
// // #define SAMPLES             64
// // #define BITS8               255
// // #define BITS9               511
// // #define BITS12              4095
// // #define T_MOTOR             24.0

// // ADC
// #define DIRECTO         0
// #define MULTISAMPLING   1
// #define ESCALON         2
// #define GRAFICA         0
// #define HISTOGRAMA      1

// // ADC
// #define ADC_WIDTH           12                  // Puede ser un valor entre 9 (0 – 511) y 12 bits (0 – 4095).
// #define FILTRO              1000
// #define ADC_CHANNEL         ADC1_CHANNEL_4      // Corresponde al GPIO 32
// #define ADC_ATTEN           ADC_ATTEN_DB_12     // Atenuación de 12 dB en el voltaje de entrada para usar el ADC
// #define MUESTRAS            64                  // Muestras para obtener un promedio del valor del ADC
// #define T_MOTOR             19.5
// #define BITS12              4096


// // #define TIMER_GROUP             TIMER_GROUP_0
// // #define TIMER_IDX               TIMER_0
// // #define TIMER_DIVIDER           80          // Divisor del reloj (80 MHz / 80 = 1 MHz)
// // #define TIMER_SCALE             (80000000 / TIMER_DIVIDER)  // 1 MHz (1 tick = 1 microsegundo)
// // #define TIMER_INTERVAL_SEC      1           // Intervalo de la interrupción en segundos
// // #define TIMER_INTERVAL          1000 / 1000000 // Intervalo de 1 milisegundo (en microsegundos)

// #define TIMER_DIVIDER         80               // Divisor del reloj (80 MHz / 80 = 1 MHz)
// #define TIMER_SCALE           (1000000)        // 1 MHz (1 tick = 1 microsegundo)
// #define TIMER_INTERVAL_SEC    0.001                // Intervalo de 1 segundo

// // PWM
// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (5)                                     // Pin de salida del PWM 
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define LEDC_DUTY_RES           LEDC_TIMER_8_BIT                        // Resolución del PWM (8 bits)
// #define LEDC_DUTY_PERCENT       80                                      // Duty porcentual
// #define LEDC_DUTY               ((LEDC_DUTY_PERCENT*255)/100)           // Duty al 50%. (2 ** 8) * 50% = 128
// #define LEDC_FREQUENCY          (1000)                                  // Frequency in Hertz. Set frequency at 1 kHz

// #define PROCESADORA     0
// #define PROCESADORB     1

// /*==================[Variables]======================*/

// static const char *TAG = "MAIN";

// bool flag_calculos = false;

// // PID
// double Kp = 0.2936;
// double Ki = 3.799;
// double Kd = 0.0008684;
// double EscalaAD = 0.5859375; 

// // ADC
// int modo                        = ESCALON;              // Elegir que tipo de técnica se le aplica a la señal del ADC para obtener un mejor valor
// int visualizar                  = GRAFICA;              // Forma de presentar los valores del ADC
// int LecturaCruda                = 0;                    // Lectura del ADC
// int LecturaSuavizada            = 0;                    // Valor promedio de la lectura cruda
// int LecturaFiltrada             = 0;                    // VAlor filtrado de la lectura cruda
// int Lectura                     = 0;
// double filtro                   = 0;                    // Variable para generar el filtro
// int veces[4096];                                        // Contar las cantidad de veces que aparece la misma lectura del ACD
// uint64_t millisAnt              = 0;                    // Toma el valor del momento
// float Vout_filtrada_corregida   = 0;
// float Vout_motor                = 0;

// // PID 
// float REF                       = 128;
// float ERRX[4]                   = {0, 0, 0, 0};
// float ADCRLM[2]                 = {0, 0};
// //const double T                  = 1000/1000000;  // Periodo en milisegundos 
// double T                        = 0.001;
// double A, B, C;
// double NEWDTY;
// double OLDDTY;
// float MAX                       = 255;
// double K1;
// double K2;
// double K3;
// double ADC;
// int DUTY_PWM                    = 128;

// /*==================[Prototipos de funciones]======================*/
// void TaskADC(void *taskParmPtr);       //Prototipo de la función de la tarea 

// // Función interrupción
// void IRAM_ATTR timer_isr(void* arg);

// // Función inicialización
// static void init_timer();
// static void example_ledc_init(void);
// static void k_init();

// void app_main(void)
// {
//     // ---INTERRUPCIÓN---
//     init_timer();

//     BaseType_t errA = xTaskCreatePinnedToCore(
//         TaskADC,                     	        // Funcion de la tarea a ejecutar
//         "TaskADC",   	                        // Nombre de la tarea como String amigable para el usuario
//         configMINIMAL_STACK_SIZE*3, 		    // Cantidad de stack de la tarea
//         NULL,                          	        // Parametros de tarea
//         tskIDLE_PRIORITY+1,         	        // Prioridad de la tarea -> Queremos que este un nivel encima de IDLE
//         NULL,                          		    // Puntero a la tarea creada en el sistema
//         PROCESADORA                             // Numero de procesador
//     );

//     // Gestion de errores
//     if(errA == pdFAIL)
//     {
//         ESP_LOGI(TAG, "Error al crear la tarea.");
//         while(1);    // Si no pudo crear la tarea queda en un bucle infinito
//     }else{ESP_LOGI(TAG, "Tarea creada correctamente");}

//     // ---PWM---
//     example_ledc_init();

//     // ---INTERRUPCIÓN---
//     //init_timer();

//     k_init();
    
//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_PWM));
    
//     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

// }

// /*==================[Implementacion de la tarea]======================*/
// void TaskADC(void *taskParmPtr)
// {
//     /*==================[Configuraciones]======================*/
//     //uint8_t i = (uint8_t) taskParmPtr; // Recibimos el indice led correspondiente a la tarea
    
//     // Configuración del ancho de bits del ADC
//     ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));

//     // Configuración del canal y atenuación del ADC
//     ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));

//     for(int i = 0; i <= 4096; i++)
//     {
//         veces[i]=0;
//     }

//     while (1) {

//         if(modo == DIRECTO)
//         {
//             LecturaCruda = adc1_get_raw(ADC_CHANNEL);
//         }
//         else if(modo == MULTISAMPLING)
//         {
//             for(int i = 0; i < MUESTRAS; i++)
//             {
//                 LecturaSuavizada += LecturaCruda = adc1_get_raw(ADC_CHANNEL);
//             }
//             LecturaSuavizada /= MUESTRAS;
//         }
//         else if(modo == ESCALON)
//         {
//             LecturaCruda = adc1_get_raw(ADC_CHANNEL);    
//             filtro += ((double)LecturaCruda - filtro) / (double)FILTRO;
//             LecturaFiltrada = (int)filtro;
//         }

//         //Aumenta el número de veces que aparece una lectura del ADC
//         veces[Lectura]++;
//         LecturaFiltrada *= ((-0.13 * (LecturaFiltrada - 2047)) / 2048) + 1.13;
//         Vout_filtrada_corregida = (LecturaFiltrada * 3.22) / 4096;
//         Vout_motor = ((LecturaFiltrada)*T_MOTOR) / BITS12;
//         //vTaskDelay(pdMS_TO_TICKS(100));

//         if(flag_calculos == true)
//         {
//             ERRX[0] = REF - (uint8_t)LecturaFiltrada;
//             A = K1*ERRX[0];
//             B = K2*ERRX[1];
//             C = K3*ERRX[2];
//             NEWDTY = A + B + C + OLDDTY;

//             if (NEWDTY > MAX)
//             { 
//                 NEWDTY = MAX; 
//             }
//             else if(NEWDTY < 0.0)
//             {
//                 NEWDTY = 0.0; /* beyond saturation */
//             }

//             //ADCRLM[1] = ADC; 
//             ERRX[3] = ERRX[2]; 
//             ERRX[2] = ERRX[1];
//             ERRX[1] = ERRX[0];
//             OLDDTY = NEWDTY; 
//             DUTY_PWM = (int)NEWDTY; 

//             ESP_LOGI(TAG, "Duty -> %d", DUTY_PWM);

//             ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_PWM));
//             ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

//             flag_calculos = false;
//         }

//         //Visualizar el resultado de la conversión del ADC en función del modo escogido
//         if(visualizar == HISTOGRAMA)
//         {
//             //El hstograma se actualiza cada 1 s
//             if(esp_timer_get_time() == millisAnt + 1000*3000)
//             {
//                 for(int i = (1600 - 25); i < (1600 + 25); i++)
//                 {
//                     ESP_LOGI(TAG, "%d\n", veces[i]);
//                 }
//                 for(int i = 0; i < 4096; i++)
//                 {
//                     veces[i] = 0;
//                 }
//                 millisAnt = esp_timer_get_time();
//             }
//         }
//         else if(visualizar == GRAFICA)
//         {
//             //Se muestra un punto en SerialPloter cada décima de segundo
//             if(esp_timer_get_time() > millisAnt + 100*3000)
//             {
//                 ESP_LOGI(TAG, "Lectura Cruda: %d\n", LecturaCruda);
                
//                 if(modo == MULTISAMPLING)
//                 {
//                     ESP_LOGI(TAG, "Lectura Suavizada: %d\n", LecturaSuavizada);
//                 }
            
//                 if(modo == ESCALON)
//                 {
//                     ESP_LOGI(TAG, "Lectura Filtrada: %d\n", LecturaFiltrada);
//                 }
//                 millisAnt = esp_timer_get_time();
//                 ESP_LOGI(TAG, "Vout_filtrada_corregida: %.03f", Vout_filtrada_corregida);
//                 ESP_LOGI(TAG, "Vout_motor: %.03f", Vout_motor);
//             }
//         }
//     }
// }

// void k_init()
// {
//     K1 = (2*T*Kp + Ki*T*T + 2*Kd) / (2*T);
//     K2 = (Ki*T*T - 2*Kp*T - 4*Kd) / (2*T); 
//     K3 = Kd / T;
// }

// void IRAM_ATTR timer_isr(void* arg) 
// {
//     //ADC = (float)Vout_motor * EscalaAD;

//     ESP_LOGI(TAG, "Interrupcion");
//     flag_calculos = true;
//     timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0); // Limpia la interrupción
//     timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0); // Rehabilita la alarma para el siguiente ciclo
//     //TIMER_0.int_clr_timers.t0 = 1;

//     // ERRX[0] = REF - (uint8_t)LecturaFiltrada;
//     // A = K1*ERRX[0];
//     // B = K2*ERRX[1];
//     // C = K3*ERRX[2];
//     // NEWDTY = A + B + C + OLDDTY;

//     // if (NEWDTY > MAX)
//     // { 
//     //     NEWDTY = MAX; 
//     // }
//     // else if(NEWDTY < 0.0)
//     // {
//     //     NEWDTY = 0.0; /* beyond saturation */
//     // }

//     // //ADCRLM[1] = ADC; 
//     // ERRX[3] = ERRX[2]; 
//     // ERRX[2] = ERRX[1];
//     // ERRX[1] = ERRX[0];
//     // OLDDTY = NEWDTY; 
//     // DUTY_PWM = (int)NEWDTY; 

//     // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_PWM));
//     // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

// }

// static void init_timer() 
// {
//     // Configura el temporizador
//     timer_config_t config = {
//         .divider = TIMER_DIVIDER,                  // Configura el divisor del reloj
//         .counter_dir = TIMER_COUNT_UP,             // Contador hacia arriba
//         .counter_en = TIMER_PAUSE,                 // Inicia en pausa
//         .alarm_en = TIMER_ALARM_EN,                // Habilitar alarma (interrupción)
//         .auto_reload = true,                       // Recargar automáticamente
//     };

//     // Inicializa el temporizador del grupo 0, temporizador 0
//     timer_init(TIMER_GROUP_0, TIMER_0, &config);

//     // Configura el valor de la alarma para el intervalo deseado
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_SCALE * TIMER_INTERVAL_SEC);

//     // Habilita la interrupción y registra la ISR
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

//     // Inicia el temporizador
//     timer_start(TIMER_GROUP_0, TIMER_0);
// }

// static void example_ledc_init(void)
// {
//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_MODE,
//         .timer_num        = LEDC_TIMER,
//         .duty_resolution  = LEDC_DUTY_RES,
//         .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 1 kHz
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel = {
//         .speed_mode     = LEDC_MODE,
//         .channel        = LEDC_CHANNEL,
//         .timer_sel      = LEDC_TIMER,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = LEDC_OUTPUT_IO,
//         .duty           = 0, // Set duty to 0%
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
// }

#include <stdio.h>
#include <math.h>
#include <float.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h" 

#define DIRECTO         0
#define MULTISAMPLING   1
#define ESCALON         2
#define GRAFICA         0
#define HISTOGRAMA      1

// ADC
#define ADC_WIDTH           12                  // Puede ser un valor entre 9 (0 – 511) y 12 bits (0 – 4095).
#define FILTRO              1000
#define ADC_CHANNEL         ADC1_CHANNEL_4      // Corresponde al GPIO 32
#define ADC_ATTEN           ADC_ATTEN_DB_12     // Atenuación de 12 dB en el voltaje de entrada para usar el ADC
#define MUESTRAS            64                  // Muestras para obtener un promedio del valor del ADC
#define T_MOTOR             21.5 // 19.5
#define BITS12              4096

// PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5)                                     // Pin de salida del PWM 
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT                        // Resolución del PWM (8 bits)
#define LEDC_DUTY_PERCENT       80                                      // Duty porcentual
#define LEDC_DUTY               ((LEDC_DUTY_PERCENT*255)/100)           // Duty al 50%. (2 ** 8) * 50% = 128
#define LEDC_FREQUENCY          (1000)                                  // Frequency in Hertz. Set frequency at 1 kHz

#define PROCESADORA     0
#define PROCESADORB     1

// PID
double Kp = 0.7;    // 0.7 Funciona
// double Ki = 13.05;
// double Kd = 0.00133;
double Ki = 3.3;      // 2.6 - 4 Funciona
double Kd = 1.2;    // 0.15 Funciona
double EscalaAD = 0.5859375; 

// ADC
int modo                        = ESCALON;              // Elegir que tipo de técnica se le aplica a la señal del ADC para obtener un mejor valor
int visualizar                  = GRAFICA;              // Forma de presentar los valores del ADC
int LecturaCruda                = 0;                    // Lectura del ADC
int LecturaSuavizada            = 0;                    // Valor promedio de la lectura cruda
volatile int LecturaFiltrada    = 0;                    // VAlor filtrado de la lectura cruda
volatile int LecturaFiltrada8   = 0;
int Lectura                     = 0;
double filtro                   = 0;                    // Variable para generar el filtro
int veces[4096];                                        // Contar las cantidad de veces que aparece la misma lectura del ACD
uint64_t millisAnt              = 0;                    // Toma el valor del momento
float Vout_filtrada_corregida   = 0;
float Vout_motor                = 0;

// PID 
float REF                       = 230;
float ERRX[4]                   = {0, 0, 0, 0};
float ADCRLM[2]                 = {0, 0};
//const double T                  = 1000/1000000;  // Periodo en milisegundos 
double T                        = 0.030;
double A                        = 0.0; 
double B                        = 0.0; 
double C                        = 0.0;
int NEWDTY;
int OLDDTY                      = 0;
int MAX                         = 255;
double K1;
double K2;
double K3;
int ADC;
volatile int DUTY_PWM           = 0;

#define TIMER_DIVIDER         80               // Divisor del reloj (80 MHz / 80 = 1 MHz)
#define TIMER_SCALE           (1000000)        // 1 MHz (1 tick = 1 microsegundo)
#define TIMER_INTERVAL_SEC    0.03                // Intervalo de 1 segundo

static const char *TAG = "TIMER_EXAMPLE";

// Función inicialización
static void init_timer();
static void example_ledc_init(void);
static void k_init();

int interrupt_count = 0;  // Contador de interrupciones
bool flag = false;

// ISR: Esta función se llama cada vez que ocurre la interrupción
void IRAM_ATTR timer_isr(void *param) {
    interrupt_count++; // Incrementa el contador cada segundo
    flag = true;
    gpio_set_level(GPIO_NUM_4, 1);
    //ESP_LOGI(TAG, "Interrupcion");

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0); // Limpia la interrupción
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0); // Rehabilita la alarma para el siguiente ciclo
}

// Configuración del temporizador
static void init_timer() {
    timer_config_t config = {
        .divider = TIMER_DIVIDER,                  // Configura el divisor del reloj
        .counter_dir = TIMER_COUNT_UP,             // Contador hacia arriba
        .counter_en = TIMER_PAUSE,                 // Inicia en pausa
        .alarm_en = TIMER_ALARM_EN,                // Habilitar alarma (interrupción)
        .auto_reload = true,                       // Recargar automáticamente
    };
    
    // Inicializa el temporizador del grupo 0, temporizador 0
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Configura el valor de la alarma para el intervalo deseado
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_SCALE * TIMER_INTERVAL_SEC);

    // Habilita la interrupción y registra la ISR
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Inicia el temporizador
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main() {
    // Inicializa el temporizador y la interrupción
    init_timer();

    // ---PWM---
    example_ledc_init();

    k_init();

    esp_rom_gpio_pad_select_gpio(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    // Configuración del ancho de bits del ADC
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));

    // Configuración del canal y atenuación del ADC
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));

    // PWM
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_PWM));
    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    // Tarea principal que imprime el conteo de interrupciones
    while (1) {
        //ESP_LOGI(TAG, "Interrupciones: %d", interrupt_count);

        // if(modo == DIRECTO)
        // {
        //     LecturaCruda = adc1_get_raw(ADC_CHANNEL);
        // }
        // else if(modo == MULTISAMPLING)
        // {
        //     for(int i = 0; i < MUESTRAS; i++)
        //     {
        //         LecturaSuavizada += LecturaCruda = adc1_get_raw(ADC_CHANNEL);
        //     }
        //     LecturaSuavizada /= MUESTRAS;
        // }
        if(modo == ESCALON)
        {
            LecturaCruda = adc1_get_raw(ADC_CHANNEL);    
            filtro += ((double)LecturaCruda - filtro) / (double)FILTRO;
            LecturaFiltrada = (int)filtro;
            //LecturaFiltrada += 120;
        }

        //Aumenta el número de veces que aparece una lectura del ADC
        //veces[Lectura]++;
        // LecturaFiltrada *= ((-0.13 * (LecturaFiltrada - 2047)) / 2048) + 1.13;
        // LecturaFiltrada8 = (LecturaFiltrada*255)/4096;
        // Vout_filtrada_corregida = (LecturaFiltrada * 3.3) / 4096;
        // Vout_motor = ((LecturaFiltrada)*T_MOTOR) / BITS12;
        //vTaskDelay(pdMS_TO_TICKS(10));

        //Visualizar el resultado de la conversión del ADC en función del modo escogido
        // if(visualizar == HISTOGRAMA)
        // {
        //     //El hstograma se actualiza cada 1 s
        //     if(esp_timer_get_time() == millisAnt + 1000*3000)
        //     {
        //         for(int i = (1600 - 25); i < (1600 + 25); i++)
        //         {
        //             ESP_LOGI(TAG, "%d\n", veces[i]);
        //         }
        //         for(int i = 0; i < 4096; i++)
        //         {
        //             veces[i] = 0;
        //         }
        //         millisAnt = esp_timer_get_time();
        //     }
        // }
        if(visualizar == GRAFICA)
        {
            //Se muestra un punto en SerialPloter cada décima de segundo
            if(esp_timer_get_time() > millisAnt + 100*3000)
            {
                ESP_LOGI(TAG, "Lectura Cruda: %d\n", LecturaCruda);
                
                if(modo == MULTISAMPLING)
                {
                    ESP_LOGI(TAG, "Lectura Suavizada: %d\n", LecturaSuavizada);
                }
            
                if(modo == ESCALON)
                {
                    //int LecturaFiltrada8 = (LecturaFiltrada*255)/4096;
                    ESP_LOGI(TAG, "Lectura Filtrada: %d (12 Bits)", LecturaFiltrada);
                    ESP_LOGI(TAG, "Lectura Filtrada 8 bits: %d (8 Bits)\n", LecturaFiltrada8);
                    ESP_LOGI(TAG, "Duty + Lectura -> %d", DUTY_PWM + LecturaFiltrada8); 
                    ESP_LOGI(TAG, "Duty -> %d", DUTY_PWM); 
                    ESP_LOGI(TAG, "Error 0 -> %f", ERRX[0]);
                }
                millisAnt = esp_timer_get_time();
                ESP_LOGI(TAG, "Vout_filtrada_corregida: %.03f", Vout_filtrada_corregida);
                ESP_LOGI(TAG, "Vout_motor: %.03f", Vout_motor);
            }
        }

        if(flag == true)
        {
            //LecturaCruda = adc1_get_raw(ADC_CHANNEL);
            //LecturaFiltrada *= ((-0.13 * (LecturaFiltrada - 2047)) / 2048) + 1.13;
            //LecturaFiltrada += 120;
            LecturaFiltrada8 = ((LecturaFiltrada+150)*255)/4096;
            Vout_filtrada_corregida = (LecturaFiltrada * 3.3) / 4096;
            Vout_motor = ((LecturaFiltrada)*T_MOTOR) / BITS12;

            ERRX[0] = REF - LecturaFiltrada8;
            //ESP_LOGI(TAG, "Error 0 -> %f", ERRX[0]);
            A = K1*ERRX[0];
            B = K2*ERRX[1];
            C = K3*ERRX[2];
            //ESP_LOGI(TAG, "A = %f - B = %f - C = %f", A, B, C);
            NEWDTY = A + B + C + OLDDTY;

            if (NEWDTY > MAX)
            { 
                NEWDTY = MAX; 
            }
            else if(NEWDTY < 0.0)
            {
                NEWDTY = 0.0; /* beyond saturation */
            }

            //ADCRLM[1] = ADC; 
            ERRX[3] = ERRX[2]; 
            ERRX[2] = ERRX[1];
            ERRX[1] = ERRX[0];
            OLDDTY = NEWDTY; 
            DUTY_PWM = NEWDTY; // En lugar de (int) deberia ser (uint8_t) - Teniendo en cuenta esto, el else if < 0.0 deberia quedarse 

            //ESP_LOGI(TAG, "Duty -> %d", DUTY_PWM); 

            // PWM
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DUTY_PWM));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

            flag = false;
            gpio_set_level(GPIO_NUM_4, 0);

        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Espera 1 segundo
    }
}

void k_init()
{
    K1 = (2*T*Kp + Ki*T*T + 2*Kd) / (2*T);
    K2 = (Ki*T*T - 2*Kp*T - 4*Kd) / (2*T); 
    K3 = Kd / T;
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