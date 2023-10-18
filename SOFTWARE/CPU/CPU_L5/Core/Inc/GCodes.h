/*
 * GCodes.h
 *
 *  Created on: Aug 28, 2023
 *      Author: Morroway
 */

#ifndef INC_GCODES_H_
#define INC_GCODES_H_

#include "types.h"
#include "main.h"
#include "Temperature.h"
#include "MoveControl.h"

#define XY_STEPS_PER_REV 256.0f
#define XY_MM_PER_REV 69.115f
#define Z_MM_PER_REV 8.0f
#define STEPS_PER_REV 256.0f
#define E_MM_PER_REV 23.0f
#define MIN_TO_US 60000000.0f

void execute(gc_reader *reader);
void G0(gc_reader *reader);
void G1(gc_reader *reader);
void G28();
void G29(gc_reader *reader);
void G92(gc_reader *reader);
void M82();
void M104(gc_reader *reader);
void M105();
void M107();
void M109(gc_reader *reader);
void M140(gc_reader *reader);
void M190(gc_reader *reader);
void M204(gc_reader *reader);
void M280(gc_reader *reader);

#endif /* INC_GCODES_H_ */
