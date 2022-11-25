#include "Parser.h"

uint16_t R, T, S;
uint16_t interval = 0;
uint16_t F = 0;
double E = 0;
double X = 0;
double Y = 0;
double Z = 0;
uint16_t hot_end_temp_goal = 0;
uint16_t hot_bed_temp_goal = 0;

void reset_args()
{
    R = -1;
    T = 0;
    S = 0;
}

void get_command(uint8_t buf[])
{
    char *command;
    char *delim = " ";

    split_gcode(buf);
    command = strtok(buf, delim);
    switch (command[0])
    {
    case 'M':
        HAL_Delay(1);
        switch (atoi(buf + 1))
        {
        case 105:
            M105(R, T);
            break;
        case 104:
            hot_end_temp_goal = S;
            M104();
            break;
        case 106:
            M106(S);
            break;
        case 140:
            hot_bed_temp_goal = S;
            M140();
            break;
        case 155:
            interval = M155(S);
            break;
        }
        break;
    case 'G':
        HAL_Delay(1);
        switch(atoi(buf + 1))
        {
            case 0:
            case 1:
                 G0();
                break;
            case 28:
                G28();
                break;
        }
        break;
    }
    reset_args();
    memset(&command[0], 0, sizeof(command));
}

void split_gcode(uint8_t buf[])
{
    char *buf2;
    HAL_Delay(1);
    buf2 = strtok(buf, " ");
    while (buf2 != NULL)
    {
        switch (buf2[0])
        {
        case 'R':
            R = atoi(buf2 + 1);
            break;
        case 'T':
            T = atoi(buf2 + 1);
            break;
        case 'S':
            S = atoi(buf2 + 1);
            break;
        case 'X':
            X = atof(buf2 + 1);
            break;
        case 'Y':
            Y = atof(buf2 + 1);
            break;
        case 'Z':
            Z = atof(buf2 + 1);
            break;
        case 'E':
            E = atof(buf2 + 1);
            break;
        case 'F':
            F = atoi(buf2 + 1);
            break;
        }
        HAL_Delay(1);
        buf2 = strtok(NULL, " ");
    }
}