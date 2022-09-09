#include "GCodes.h"

void M105(int R, int T)
{
    print_temperature();
    #ifdef REDUNDANT_TEMP_SENSOR
    if(R != -1)
    {
        
    }
    #endif
}

int M155(int S)
{
    return S*1000;
}