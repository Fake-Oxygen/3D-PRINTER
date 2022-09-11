#include "GCodes.h"

double Cur_E = 0;
double Cur_X = 0;
double Cur_Y = 0;
double Cur_Z = 0;

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
    if (E_dif >= 0.005 || E_dif <= -0.005)
    {
        if (E_dif < 0)
        {
            CHANGE_MOTOR_DIR(E_AXIS_DIR, COUNTERCLOCKWISE);
            Cur_E -= 23.0f/19256.0f;
        }
        else
        {
            CHANGE_MOTOR_DIR(E_AXIS_DIR, CLOCKWISE);
            Cur_E += 23.0f/19256.0f;
        }
        MAKE_MOTOR_STEP(E_AXIS_STEP);
        DelayMicrosecond(500);
    }
}