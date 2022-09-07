#include "Parser.h"
#include "GCodes.h"

void get_command(char buf[])
{
    command = strtok(buf, delim);
    //split_gcode(command);
    switch(command[0])
    {
        case 'M':
        HAL_Delay(1);
        switch(get_argument(command))
        {
            case 105:
                M105(R, T);
                break;
        }
            break;
    }
    memset(&command[0], 0, sizeof(command));
}

uint16_t get_argument(char arg[])
{
    return atoi(arg+1);
}

char *split_gcode(char buf[])
{
    buf = strtok(NULL, delim);
    switch(buf[0])
    {
        case 'R':
            R = get_argument(buf);
            break;
        case 'T':
            T = get_argument(buf);
            break;
    }
    if(buf == NULL)
        return NULL;
    else return split_gcode(buf);
}