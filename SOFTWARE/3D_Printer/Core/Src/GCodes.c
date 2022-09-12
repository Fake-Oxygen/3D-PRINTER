#include "GCodes.h"

double Cur_E = 0;
double Cur_X = 0;
double Cur_Y = 0;
double Cur_Z = 0;
uint16_t Cur_F = 0;
bool isRunning = false;
uint32_t last_time_E = 0;

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
    uint16_t speed = (double)1 / (double)F * (double)60 * MM_PER_REV / STEPS_PER_REV * (double)1000000;

    if (E_dif >= 0.0012 || E_dif <= -0.0012)
    {
        if (GetTicks() - last_time_E >= speed && speed > 0)
        {
            isRunning = true;
            if (E_dif < 0)
            {
                CHANGE_MOTOR_DIR(E_AXIS_DIR, COUNTERCLOCKWISE);
                Cur_E -= MM_PER_REV / STEPS_PER_REV;
            }
            else
            {
                CHANGE_MOTOR_DIR(E_AXIS_DIR, CLOCKWISE);
                Cur_E += MM_PER_REV / STEPS_PER_REV;
            }
            MAKE_MOTOR_STEP(E_AXIS_STEP);
            last_time_E = GetTicks();
            // DelayMicrosecond(speed);
        }
    }
    else
    {
        isRunning = false;
    }
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