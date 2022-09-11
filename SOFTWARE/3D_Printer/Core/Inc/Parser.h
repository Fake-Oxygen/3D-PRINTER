#ifndef PARSER_H
#define PARSER_H

#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

extern uint16_t F;
extern uint16_t interval;
extern double X, Y, Z, E;

void reset_args();
void get_command(uint8_t buf[]);
void split_gcode(uint8_t buf[]);

#endif