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


void M105(int R, int T);
int M155(int S);
#endif