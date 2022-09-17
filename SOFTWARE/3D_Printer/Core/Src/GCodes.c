#include "GCodes.h"

double Cur_E = 0;
double Cur_X = 0;
double Cur_Y = 0;
double Cur_Z = 0;
uint16_t Cur_F = 0;
bool isRunning = false;
uint32_t last_time_E = 0;
uint32_t last_time_X = 0;
uint32_t last_time_Y = 0;

void M105(uint16_t R, uint16_t T)
{
    print_temperature();
#ifdef REDUNDANT_TEMP_SENSOR
    if (R != -1)
    {
    }
#endif
}

uint16_t M155(uint16_t S)
{
    return S * 1000;
}

void G0()
{
    double E_dif = E - Cur_E;
    double X_dif = X - Cur_X;
    double Y_dif = Y - Cur_Y;
    uint16_t speed = (double)1 / (double)F * (double)60 * E_MM_PER_REV / STEPS_PER_REV * (double)1000000;

    Move(E_dif, last_time_E, E_AXIS, speed);
    Move(X_dif, last_time_X, X_AXIS, speed);
}

void M104()
{
    Cur_temp = GetTemperature(ADC_HOT_END, value[ADC_HOT_END]);
    if (Cur_temp <= temp_goal)
    {
        SetHeating(HOT_END, -9.0f / 10.0f * (double)Cur_temp + (double)temp_goal + 100.0f);
    }
    else
    {
        SetHeating(HOT_END, 0);
    }
}

void M106(uint16_t S)
{

    SetFanSpeed(HOT_END_FAN, S);
}
