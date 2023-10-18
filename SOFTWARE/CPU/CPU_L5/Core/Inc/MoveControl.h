/*
 * MoveControl.h
 *
 *  Created on: Aug 29, 2023
 *      Author: Morroway
 */

#ifndef INC_MOVECONTROL_H_
#define INC_MOVECONTROL_H_

#include <math.h>
#include "types.h"
#include "main.h"

void move_xy(gc_reader *reader);
vector2D delta(vector2D A, vector2D B);
vector2D move(vector2D diff);
double sign(double x);
double length(vector2D A);
vector2D mult(vector2D A, double v);
void step_A(double d);
void step_B(double d);
vector2D calc_diff(double A, double B);
void add_vec(vector2D A, vector2D B);

#endif /* INC_MOVECONTROL_H_ */
