#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>

#define HE0 0
#define HE1 1
#define BED 2
#define STM 3

#define ADC_VOLTAGE 3.3
#define ADC_SAMPLING 65536
#define V30 0.76
#define AVG_SLOPE 0.0025

#define BED_THERM_A 0.00128
#define BED_THERM_B 0.00024
#define BED_THERM_C 8.5E-08
#define BED_RESISTANCE 2200

#define HE_THERM_A 0.000699582
#define HE_THERM_B 0.000219518
#define HE_THERM_C 9.071E-08
#define HE_RESISTANCE 4700

#define KELVIN_OFFSET 273.15

double GetTemperature(uint16_t adc, uint32_t value);
void Heat(uint16_t heater, uint16_t target, double current);

#endif
