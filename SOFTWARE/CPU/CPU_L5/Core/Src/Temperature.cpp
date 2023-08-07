#include "Temperature.h"
#include <math.h>
#include "main.h"


double GetTemperature(uint16_t adc, uint32_t value)
{
	double resistance, val;
	switch (adc)
	{
	case BED:
		val = (ADC_VOLTAGE / ADC_SAMPLING) * value;
		resistance = val / ((ADC_VOLTAGE - val) / BED_RESISTANCE);
		return 1 / (BED_THERM_A + BED_THERM_B * log(resistance) + BED_THERM_C * pow(log(resistance), 3)) - KELVIN_OFFSET;
		break;
	case HE0:
	case HE1:
		val = (ADC_VOLTAGE / ADC_SAMPLING) * value;
		resistance = val / ((ADC_VOLTAGE - val) / HE_RESISTANCE);
		return 1 / (HE_THERM_A + HE_THERM_B * log(resistance) + HE_THERM_C * pow(log(resistance), 3)) - KELVIN_OFFSET;
		break;
	case STM:
		return ((V30 - (ADC_VOLTAGE / ADC_SAMPLING * value)) / AVG_SLOPE + 30);
		break;
	}
}

void Heat(uint16_t heater, uint16_t target, double current)
{
	switch(heater)
	{
	case BED:
		TIM15->CCR2 = 70;
		break;
	}
}
