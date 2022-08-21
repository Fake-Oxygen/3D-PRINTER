#include "Functions.h"

void DelayMicrosecond(TIM_HandleTypeDef *timer, uint8_t time)
{
    __HAL_TIM_SET_COUNTER(timer,0);
	while (__HAL_TIM_GET_COUNTER(timer) < time);
}
void SelectDriver(uint8_t axis)
{
    switch(axis)
    {
        case X_AXIS:
            WRITE_PIN(MPLX_PIN_A, 0);
            WRITE_PIN(MPLX_PIN_B, 0);
            break;
        case Y_AXIS:
            WRITE_PIN(MPLX_PIN_A, 1);
            WRITE_PIN(MPLX_PIN_B, 0);
            break;
        case E_AXIS:
            WRITE_PIN(MPLX_PIN_A, 0);
            WRITE_PIN(MPLX_PIN_B, 1);
            break;
        case Z_AXIS:
            WRITE_PIN(MPLX_PIN_A, 1);
            WRITE_PIN(MPLX_PIN_B, 1);
            break;
    }
}