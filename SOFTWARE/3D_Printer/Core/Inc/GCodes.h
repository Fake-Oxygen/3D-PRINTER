//  G0-G1: Linear Move
//  G2-G3: Arc or Circle Move
//  G4: Dwell
//  G5: Bézier cubic spline
//  G6: Direct Stepper Move
//  G10: Retract

//  M17: Enable Steppers
//  M18, M84: Disable steppers
//  M104: Set Hotend Temperature
//  M105: Report Temperatures
//  M106: Set Fan Speed
//  M107: Fan Off
//  M109: Wait for Hotend Temperature
//  M118: Serial print
//  M155: Temperature Auto-Report
#ifndef GCodes_H
#define GCodes_H

#include "Config.h"
#include "Functions.h"
#include "main.h"
#include <stdbool.h>

extern double Cur_X, Cur_Y, Cur_Z, Cur_E;
extern uint16_t Cur_F;
extern bool isRunning;
extern uint16_t temp_goal;
extern uint16_t Cur_temp;
extern uint32_t last_time_E;
extern uint32_t last_time_X;
extern uint32_t last_time_Y;
extern uint32_t last_tick_A;
extern uint32_t last_tick_B;
extern uint32_t last_tick_AB;

void M104();
void M105(uint16_t R, uint16_t T);
void M106(uint16_t S);
uint16_t M155(uint16_t S);
void G0();
void G28();

#endif