#include "battery_level.h"

//#include <stdlib.h>
#include <esp32-hal.h>


#define ADC_PIN 35
//#define vref  1100
#define vref  1032

#define correction_factor 4.26 / 4.54


float battery_level::battery_voltage(){
    uint16_t v = analogRead(ADC_PIN);
    float res = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
 
    return res;
}

float battery_level::battery_percent (){
    return 0.0;
}

 