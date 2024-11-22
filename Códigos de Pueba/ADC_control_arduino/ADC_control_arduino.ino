#include <driver/adc.h>

#define DIRECTO 0
#define MULTISAMPLING 1
#define ESCALON 2

#define GRAFICA 0
#define HISTOGRAMA 1

#define FILTRO 1000
#define ADC_CHANNEL ADC1_CHANNEL_4    //Corresponde al GPIO 32
#define ADC_ATTEN ADC_ATTEN_DB_12     //Atenuación de 12 dB en el voltaje de entrada para usar el ADC
#define MUESTRAS 64                   //Muestras para obtener un promedio del valor del ADC
#define T_MOTOR  21.12
#define BITS12   4096

int modo = ESCALON;               //Elegir que tipo de técnica se le aplica a la señal del ADC para obtener un mejor valor
int visualizar = GRAFICA;      //Forma de presentar los valores del ADC
int LecturaCruda = 0;                //Lectura del ADC
int LecturaSuavizada = 0;            //Valor promedio de la lectura cruda
int LecturaFiltrada = 0;             //VAlor filtrado de la lectura cruda
int Lectura = 0;
double filtro = 0;                   //Variable para generar el filtro
int veces[4096];                     //Contar las cantidad de veces que aparece la misma lectura del ACD
int millisAnt = 0;                    //Toma el valor del momento
float Vout_filtrada_corregida = 0;
float Vout_motor = 0;

//unsigned int Dout  = 0;              //Lectura cruda del ADC
//unsigned float Smooth_val = 0;                //Valor promedio de la lectura cruda
//float Vout = 0;
//float Vout_smooth = 0;

void setup() {
  adc1_config_channel_atten(ADC_CHANNEL,ADC_ATTEN);   //Ajusta la atenuación del valor medido de voltaje
  adc1_config_width(ADC_WIDTH_BIT_12);                //Ajusta la resulución del ADC
  Serial.begin(115200);
  for(int i=0; i<=4096; i++){
    veces[i]=0;
  } 
}

void loop() {
  if(modo == DIRECTO){
    LecturaCruda = adc1_get_raw(ADC_CHANNEL);
  }
  else if(modo == MULTISAMPLING){
    for(int i = 0; i < MUESTRAS; i++){
    LecturaSuavizada += LecturaCruda = adc1_get_raw(ADC_CHANNEL);
    }
    LecturaSuavizada /= MUESTRAS;
  }
  else if(modo == ESCALON){
    LecturaCruda = adc1_get_raw(ADC_CHANNEL);    
    filtro += ((double)LecturaCruda - filtro) / (double)FILTRO;
    LecturaFiltrada = (int)filtro;
  }

  //Aumenta el número de veces que aparece una lectura del ADC
  veces[Lectura]++;
  LecturaFiltrada *= ((-0.13 * (LecturaFiltrada - 2047)) / 2048) + 1.13;
  Vout_filtrada_corregida = (LecturaFiltrada * 3.22) / 4096;
  Vout_motor = ((LecturaFiltrada)*T_MOTOR) / BITS12;

  //Visualizar el resultado de la conversión del ADC en función del modo escogido
  if(visualizar == HISTOGRAMA){
    //El hstograma se actualiza cada 1 s
    if(millis() == millisAnt + 1000){
      for(int i = 1600-25; i<1600+25; i++){
        Serial.print(veces[i]);
      }
      for(int i=0; i<4096; i++){
        veces[i] = 0;
      }
      millisAnt = millis();
    }
  }
  else if(visualizar == GRAFICA){
    //Se muestra un punto en SerialPloter cada décima de segundo
    if(millis() > millisAnt + 100){
        Serial.print("LecturaCruda:");
        Serial.println(LecturaCruda);
        
      if(modo == MULTISAMPLING){
        Serial.print("LecturaSuavizada:");
        Serial.println(LecturaSuavizada);
      }
      
      if(modo == ESCALON){
        Serial.print("LecturaFiltrada:");
        Serial.println(LecturaFiltrada);
      }
        millisAnt = millis();
        Serial.print("Vout_filtrada_corregida:");
        Serial.println(Vout_filtrada_corregida);
        Serial.print("Vout_motor:");
        Serial.println(Vout_motor);
    }
  }
}
