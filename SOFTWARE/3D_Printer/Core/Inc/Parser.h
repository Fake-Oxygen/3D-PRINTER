#ifndef PARSER_H
#define PARSER_H

#include "main.h"

static uint16_t R, T, S;
extern uint16_t interval;


void reset_args();
void get_command(uint8_t buf[]);
void split_gcode(uint8_t buf[]);

#endif