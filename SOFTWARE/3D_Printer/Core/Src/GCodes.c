#include "GCodes.h"
#include "math.h"

double Cur_E = 0;
double Cur_X = 0;
double Cur_Y = 0;
double Cur_Z = 0;
uint16_t Cur_F = 0;
bool isRunning = false;
uint32_t last_time_E = 0;
uint32_t last_time_A = 0;
uint32_t last_time_B = 0;
uint32_t last_time_Z = 0;

void M105(uint16_t R, uint16_t T)
{
    myprintf("T: %.2f B: %.2f\r\n", GetTemperature(ADC_HOT_END, value[ADC_HOT_END]), GetTemperature(ADC_BED, value[ADC_BED]));
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
    // isRunning = true;
    double E_dif = E - Cur_E;
    double Z_dif = Z - Cur_Z;
    // double E_dif = 0;
    // double Z_dif = 0;
    double dif_x = X - Cur_X;
    double dif_y = Y - Cur_Y;
    // double xy_len = sqrt(dif_x * dif_x + dif_y * dif_y);
    // double dir_x = dif_x / xy_len;
    // double dir_y = dif_y / xy_len;
    // uint32_t speed = (double)1 / (double)F * (double)60 * E_MM_PER_REV / STEPS_PER_REV * (double)1000000;
    // Move(Z_dif, last_time_Z, Z_AXIS, speed);
    // Move(E_dif, last_time_E, E_AXIS, speed);
    // Move2(E_dif);
    Move(1 * dif_x, -1 * dif_y, E_dif, Z_dif);
    if (E_dif == 0 && Z_dif == 0 && dif_x == 0 && dif_y == 0)
    {
        isRunning = false;
    }
}

void M104()
{
    uint16_t Cur_temp = GetTemperature(ADC_HOT_END, value[ADC_HOT_END]);
    if (hot_end_temp_goal > 0)
    {
        uint16_t power = (-1 * pow((hot_end_temp_goal - Cur_temp - 270), 2)) / 300 + 255;
        SetHeating(HOT_END, power);
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

void M140()
{
    uint16_t Cur_temp = GetTemperature(ADC_BED, value[ADC_BED]);
    if (hot_bed_temp_goal > 0)
    {
        uint16_t power = (-1 * pow((hot_bed_temp_goal - Cur_temp - 100), 2)) / 40 + 255;
        SetHeating(HOT_BED, power);
    }
    else
    {
        SetHeating(HOT_BED, 0);
    }
}

void G28()
{
    isRunning = true;
    Cur_E = 0;
    E = 0;
    MoveAndWait(1, 400, X_AXIS, COUNTERCLOCKWISE, COUNTERCLOCKWISE);
    MoveAndWait(0, 300, X_AXIS, CLOCKWISE, CLOCKWISE);
    Cur_X = 0;
    X = 0;
    MoveAndWait(1, 400, Y_AXIS, CLOCKWISE, COUNTERCLOCKWISE);
    MoveAndWait(0, 300, Y_AXIS, COUNTERCLOCKWISE, CLOCKWISE);
    Cur_Y = 0;
    Y = 0;
    isRunning = false;
}