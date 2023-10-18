/*
 * MoveControl.c
 *
 *  Created on: Aug 30, 2023
 *      Author: Morroway
 */

#include "MoveControl.h"

extern header head;

void move_xy(gc_reader *reader) {

	vector2D AB_delta = head.AB_delta;
	int8_t dir_A = sign(AB_delta.x);
	int8_t dir_B = sign(AB_delta.y);

	uint32_t pd_time = (length(head.d) / head.F) * (60 * 1000000);
//	if(pd_time > 65535)
//	{
//		head.isrunning = FREE;
//		return;
//	}
	double ds = PI * 11 * 2 / 400;
	uint32_t step_speed_B = 0;
	uint32_t step_speed_A = 0;
	vector2D AB_delta_steps = mult(AB_delta, 1 / ds);
	if (AB_delta_steps.x != 0)
	{
		step_speed_A = pd_time / abs(AB_delta_steps.x);
		head.StepsX = abs(AB_delta_steps.x);
	}
	if (AB_delta_steps.y != 0)
	{
		step_speed_B = pd_time / abs(AB_delta_steps.y);
		head.StepsY = abs(AB_delta_steps.y);
	}


	double steps_diff_A = 0;
	double steps_diff_B = 0;

	if (GetTicks() - head.last_time_X > step_speed_A && step_speed_A != 0) {
		if (dir_A < 0)
			HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(STEP_X_GPIO_Port, STEP_X_Pin);
		steps_diff_A += dir_A;
		head.CurStepsX++;
		head.last_time_X = GetTicks();
	}

	if (GetTicks() - head.last_time_Y > step_speed_B && step_speed_B != 0) {
		if (dir_B < 0)
			HAL_GPIO_WritePin(DIR_E0_GPIO_Port, DIR_E0_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(DIR_E0_GPIO_Port, DIR_E0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(STEP_E0_GPIO_Port, STEP_E0_Pin);
		steps_diff_B += dir_B;
		head.CurStepsY++;
		head.last_time_Y = GetTicks();
	}
	vector2D diff = calc_diff(steps_diff_A * ds, steps_diff_B * ds);
	head.X += diff.x;
	head.Y += diff.y;
}
vector2D delta(vector2D A, vector2D B) {
	vector2D d = { B.x - A.x, B.y - A.y };
	return d;
}
vector2D move(vector2D diff) {
	vector2D ab = { diff.x + diff.y, diff.x - diff.y };
	return ab;
}
double sign(double x) {
	if (x == 0)
		return 0;
	return x / abs(x);
}
double length(vector2D A) {
	return sqrt(A.x * A.x + A.y * A.y);
}
vector2D mult(vector2D A, double v) {
	vector2D x = { round(A.x * v), round(A.y * v)};
	return x;
}
void step_A(double d) {

}
void step_B(double d) {

}
vector2D calc_diff(double A, double B) {
	vector2D diff = { 0.5 * (A + B), 0.5 * (A - B) };
	return diff;
}
void add_vec(vector2D A, vector2D B) {

}
