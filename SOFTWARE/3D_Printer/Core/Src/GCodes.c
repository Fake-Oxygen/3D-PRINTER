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
uint32_t last_time_Z = 0;
uint32_t last_tick_A = 0;
uint32_t last_tick_B = 0;
uint32_t last_tick_AB = 0;

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
    double Z_dif = Z - Cur_Z;
    double dif_x = X - Cur_X;
    double dif_y = Y - Cur_Y;
    double xy_len = sqrt(dif_x * dif_x + dif_y * dif_y);
    double dir_x = dif_x / xy_len;
    double dir_y = dif_y / xy_len;
    uint32_t speed = (double)1 / (double)F * (double)60 * E_MM_PER_REV / STEPS_PER_REV * (double)1000000;
    Move(Z_dif, last_time_Z, Z_AXIS, speed);
    Move(E_dif, last_time_E, E_AXIS, speed);
    if(fabs(dif_x) > OFFSET_P || fabs(dif_y) > OFFSET_P) {
        MoveXY(dir_x, dir_y);
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

void G28()
{
    Cur_E = 0;
    E = 0;
    MoveAndWait(1, 400, X_AXIS, CLOCKWISE, CLOCKWISE);
    MoveAndWait(0, 1000, X_AXIS, COUNTERCLOCKWISE, COUNTERCLOCKWISE);
    MoveAndWait(1, 1000, X_AXIS, CLOCKWISE, CLOCKWISE);
    MoveAndWait(0, 1000, X_AXIS, COUNTERCLOCKWISE, COUNTERCLOCKWISE);
    Cur_X = 0;
    X = 0;
    MoveAndWait(1, 400, Y_AXIS, CLOCKWISE, COUNTERCLOCKWISE);
    MoveAndWait(0, 1000, Y_AXIS, COUNTERCLOCKWISE, CLOCKWISE);
    MoveAndWait(1, 1000, Y_AXIS, CLOCKWISE, COUNTERCLOCKWISE);
    MoveAndWait(0, 1000, Y_AXIS, COUNTERCLOCKWISE, CLOCKWISE);
    Cur_Y = 0;
    Y = 0;
}