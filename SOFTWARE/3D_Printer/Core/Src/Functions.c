#include "Functions.h"

void SelectDriver(uint16_t axis)
{
    switch (axis)
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

double GetTemperature(uint16_t adc, uint16_t value)
{
    double resistance, val;
    switch (adc)
    {
    case ADC_HOT_END:
        val = (ADC_VOLTAGE / ADC_SAMPLING) * value;
        resistance = val / ((ADC_VOLTAGE - val) / HOT_END_RESISTANCE);
        return 1 / (HOT_END_THERM_A + HOT_END_THERM_B * log(resistance) + HOT_END_THERM_C * pow(log(resistance), 3)) - KELVIN_OFFSET;
        break;
    case ADC_STM_TEMP:
        return ((V30 - (ADC_VOLTAGE / ADC_SAMPLING * value)) / AVG_SLOPE + 30);
        break;
    }
}

void SetHeating(uint16_t heater, uint16_t power)
{
    switch (heater)
    {
    case HOT_END:
        TIM3->CCR2 = power;
        break;
    }
}

void SetFanSpeed(uint16_t fan, uint16_t speed)
{
    switch (fan)
    {
    case HOT_END_FAN:
        TIM3->CCR3 = speed;
        break;
    }
}

void Move45() {
    //1539 49mm
    //3078 50mm
    //4000 48mm
    if(GetTicks() - last_time_X > 1539) {
        CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
        MAKE_MOTOR_STEP(X_AXIS_STEP);
        g50_steps++;
        last_time_X = GetTicks();
    }
}

void MoveY() {
    if(GetTicks() - last_time_Y > 1539) {
        CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
        MAKE_MOTOR_STEP(X_AXIS_STEP);
        CHANGE_MOTOR_DIR(Y_AXIS_DIR, COUNTERCLOCKWISE);
        MAKE_MOTOR_STEP(Y_AXIS_STEP);
        g50_steps++;
        last_time_Y = GetTicks();
    }
}

void MoveXY(double dir_x, double dir_y) {
    double engine_dir_B = dir_x * SQRT_2_BY_2 - dir_y * SQRT_2_BY_2; // engine B (image Y)
    double engine_dir_A = dir_x * SQRT_2_BY_2 + dir_y * SQRT_2_BY_2; // engine A (image X)

    double feed_rate_A = fabs(F * engine_dir_A); // F = feed rate
    double feed_rate_B = fabs(F * engine_dir_B);

    if(feed_rate_A > 0) {
        uint32_t speed_A = (double)1 / (double)feed_rate_A * (double)60 * XY_MM_PER_REV / XY_STEPS_PER_REV * (double)1000000;

        if(GetTicks() - last_tick_A >= speed_A && fabs(engine_dir_A) > ENGINE_MIN_SPEED) {
            // isRunning = true;
            if(engine_dir_A > 0) {
                CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
                MAKE_MOTOR_STEP(X_AXIS_STEP);
            } else if(engine_dir_A < 0) {
                CHANGE_MOTOR_DIR(X_AXIS_DIR, COUNTERCLOCKWISE);
                MAKE_MOTOR_STEP(X_AXIS_STEP);
            }
            last_tick_A = GetTicks();
        }
    }

    if(feed_rate_B > 0) {
        uint32_t speed_B = (double)1 / (double)feed_rate_B * (double)60 * XY_MM_PER_REV / XY_STEPS_PER_REV * (double)1000000;

        if(GetTicks() - last_tick_B >= speed_B && fabs(engine_dir_B) > ENGINE_MIN_SPEED) {
            // isRunning = true;
            if(engine_dir_B > 0) {
                CHANGE_MOTOR_DIR(Y_AXIS_DIR, CLOCKWISE);
                MAKE_MOTOR_STEP(Y_AXIS_STEP);
            } else if(engine_dir_B < 0) {
                CHANGE_MOTOR_DIR(Y_AXIS_DIR, COUNTERCLOCKWISE);
                MAKE_MOTOR_STEP(Y_AXIS_STEP);
            }
            last_tick_B = GetTicks();
        }
    }

    if(F > 0) {
        uint32_t speed_AB = (double)1 / (double)F * (double)60 * XY_MM_PER_REV / XY_STEPS_PER_REV * (double)1000000;
        if(GetTicks() - last_tick_AB >= speed_AB) {
            Cur_X += dir_x * XY_MM_PER_REV / XY_STEPS_PER_REV;
            Cur_Y += dir_y * XY_MM_PER_REV / XY_STEPS_PER_REV;
        }
    }

    // isRunning = false;
}

void Move(double dif, uint32_t last_time, uint16_t axis, uint16_t speed)
{
    if (dif >= OFFSET_P || dif <= OFFSET_N)
    {
        if (GetTicks() - last_time >= speed && speed > 0)
        {
            isRunning = true;
            if (dif < 0)
            {
                switch (axis)
                {
                case E_AXIS:
                    CHANGE_MOTOR_DIR(E_AXIS_DIR, COUNTERCLOCKWISE);
                    Cur_E -= E_MM_PER_REV / STEPS_PER_REV;
                    MAKE_MOTOR_STEP(E_AXIS_STEP);
                    last_time_E = GetTicks();
                    break;
                case X_AXIS:
                    CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
                    CHANGE_MOTOR_DIR(Y_AXIS_DIR, CLOCKWISE);
                    Cur_X -= XY_MM_PER_REV / XY_STEPS_PER_REV;
                    MAKE_MOTOR_STEP(X_AXIS_STEP);
                    MAKE_MOTOR_STEP(Y_AXIS_STEP);
                    last_time_X = GetTicks();
                    break;
                case Y_AXIS:
                    CHANGE_MOTOR_DIR(X_AXIS_DIR, CLOCKWISE);
                    CHANGE_MOTOR_DIR(Y_AXIS_DIR, COUNTERCLOCKWISE);
                    Cur_Y -= XY_MM_PER_REV / XY_STEPS_PER_REV;
                    MAKE_MOTOR_STEP(X_AXIS_STEP);
                    MAKE_MOTOR_STEP(Y_AXIS_STEP);
                    last_time_Y = GetTicks();
                    break;
                default:
                    break;
                }
            }
            else
            {
                switch (axis)
                {
                case E_AXIS:
                    CHANGE_MOTOR_DIR(E_AXIS_DIR, CLOCKWISE);
                    Cur_E += E_MM_PER_REV / STEPS_PER_REV;
                    MAKE_MOTOR_STEP(E_AXIS_STEP);
                    last_time_E = GetTicks();
                    break;
                case X_AXIS:
                    CHANGE_MOTOR_DIR(X_AXIS_DIR, COUNTERCLOCKWISE);
                    CHANGE_MOTOR_DIR(Y_AXIS_DIR, COUNTERCLOCKWISE);
                    Cur_X += XY_MM_PER_REV / XY_STEPS_PER_REV;
                    MAKE_MOTOR_STEP(X_AXIS_STEP);
                    MAKE_MOTOR_STEP(Y_AXIS_STEP);
                    last_time_X = GetTicks();
                    break;
                case Y_AXIS:
                    CHANGE_MOTOR_DIR(X_AXIS_DIR, COUNTERCLOCKWISE);
                    CHANGE_MOTOR_DIR(Y_AXIS_DIR, CLOCKWISE);
                    Cur_Y += XY_MM_PER_REV / XY_STEPS_PER_REV;
                    MAKE_MOTOR_STEP(X_AXIS_STEP);
                    MAKE_MOTOR_STEP(Y_AXIS_STEP);
                    last_time_Y = GetTicks();
                    break;
                default:
                    break;
                }
            }

            // DelayMicrosecond(speed);
        }
    }
    else
    {
        isRunning = false;
    }
}

void MoveAndWait(uint16_t state, uint16_t speed, uint16_t axis, uint16_t dir_X, uint16_t dir_Y)
{
    switch (axis)
    {
    case X_AXIS:
        CHANGE_MOTOR_DIR(X_AXIS_DIR, dir_X);
        CHANGE_MOTOR_DIR(Y_AXIS_DIR, dir_Y);
        while (READ_PIN(X_STOP_PIN) != state)
        {
            MAKE_MOTOR_STEP(X_AXIS_STEP);
            MAKE_MOTOR_STEP(Y_AXIS_STEP);
            DelayMicrosecond(speed);
        }
        break;
    case Y_AXIS:
        CHANGE_MOTOR_DIR(X_AXIS_DIR, dir_X);
        CHANGE_MOTOR_DIR(Y_AXIS_DIR, dir_Y);
        while (READ_PIN(Y_STOP_PIN) != state)
        {
            MAKE_MOTOR_STEP(X_AXIS_STEP);
            MAKE_MOTOR_STEP(Y_AXIS_STEP);
            DelayMicrosecond(speed);
        }
        break;
    }
}
