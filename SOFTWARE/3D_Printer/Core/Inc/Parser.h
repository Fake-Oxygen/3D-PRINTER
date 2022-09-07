#include "main.h"
#include <string.h>
#include <stdlib.h>

static char *delim = " ";
static char *command; 
static uint16_t R;
static uint16_t T;

void get_command(char buf[]);
uint16_t get_argument(char arg[]);
char *split_gcode(char buf[]);