/*
 * GCodes.c
 *
 *  Created on: Aug 28, 2023
 *      Author: Morroway
 */

#include "GCodes.h"
#include "Temperature.h"
#include "types.h"
#include "tim.h"

extern uint32_t ADC_value[ADC_CHANNELS];
extern header head;

void execute(gc_reader *reader) {
	switch (reader->code_type) {
	case 'G':
		switch (reader->code_id) {
		case 0:
		case 1:
			head.isrunning = RUNNING;
			vector2D P = { head.X, head.Y };
			vector2D D = { reader->X, -1 * reader->Y };
			head.d = delta(P, D);
			head.AB_delta = move(head.d);
			move_xy(reader);
			G0(reader);
			break;
		case 28:
			G28();
			break;
		case 29:
			G29(reader);
			break;
		case 92:
			G92(reader);
			break;
		}
		break;
	case 'M':
		switch (reader->code_id) {
		case 82:
			M82();
			break;
		case 104:
			M104(reader);
			break;
		case 105:
			M105();
			break;
		case 107:
			M107();
			break;
		case 109:
			M109(reader);
			break;
		case 140:
			M140(reader);
			break;
		case 190:
			M190(reader);
			break;
		case 204:
			M204(reader);
			break;
		case 280:
			M280(reader);
			break;
		}
		break;
	}
}

void M82() {
	head.extruder_mode = ABSOLUTE_MODE;
}

void M104(gc_reader *reader) {
	TIM2->CCR4 = 255;
	TIM3->CCR4 = 255;
	head.temp_he1 = reader->S;
}

void M105() {
	float bed = GetTemperature(BED, ADC_value[BED]);
	float he0 = GetTemperature(HE0, ADC_value[HE0]);
	printf("ok T0:%.2f B:%.2f\r\n", he0, bed);
}

void M107() {
	TIM3->CCR3 = 0;
}

void M109(gc_reader *reader) {
	TIM2->CCR4 = 255;
	TIM3->CCR4 = 255;
	head.temp_he1 = reader->S;
	head.isrunning = WAIT_FOR_HE1_TEMP;
}

void M140(gc_reader *reader) {
	head.temp_bed = reader->S;
}

void M190(gc_reader *reader) {
	head.temp_bed = reader->S;
	head.isrunning = WAIT_FOR_BED_TEMP;
}

void M204(gc_reader *reader) {
	head.normal_accel = reader->S;
	head.retract_accel = reader->T;
}

void M280(gc_reader *reader) {
	//10 ->3.24% -> 32
	uint16_t PWM = reader->S * 10;
	TIM17->CCR1 = PWM;
	HAL_Delay(1000);
	TIM17->CCR1 = 0;
}

void G0(gc_reader *reader) {
	head.F = reader->F;
	double z_diff = reader->Z - head.Z;
	uint8_t check = 0;
	if (head.CurStepsX < head.StepsX || head.CurStepsY < head.StepsY)
		move_xy(reader);
	else
		check = 1;
	uint16_t steps_z = abs(z_diff / Z_MM_PER_REV * 400);
	if (z_diff != 0) {
		uint32_t speed_z = ((z_diff / head.F) * (60 * 1000000)) / steps_z;
		if (GetTicks() - head.last_time_Z > speed_z) {
			if (z_diff < 0)
				HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(STEP_Z_GPIO_Port, STEP_Z_Pin);
			head.CurStepsZ++;
			head.last_time_Z = GetTicks();
		}
	}
	if (head.CurStepsZ >= steps_z && check == 1) {
		head.isrunning = FREE;
		uint8_t msg[10] = "Done\n";
		CDC_Transmit_FS(msg, 5);
		head.Z = reader->Z;
		head.CurStepsZ = 0;
		head.CurStepsX = 0;
		head.CurStepsY = 0;
		head.StepsX = 0;
		head.StepsY = 0;
	}
}

void G1(gc_reader *reader) {
	// To muszą być stałe
	double difx = reader->X - head.X;
	double dify = -1 * (reader->Y - head.Y);
	double dife = reader->E - head.E;
	double difz = reader->Z - head.Z;
	double difA = difx + dify;
	double difB = difx - dify;
	double feedrateA = fabs(difA) / (fabs(difA) + fabs(difB));
	double feedrateB = fabs(difB) / (fabs(difA) + fabs(difB));
	uint16_t speed_A = (double) 1 / (double) reader->F * XY_MM_PER_REV
			/ XY_STEPS_PER_REV * MIN_TO_US / (double) feedrateA;
	uint16_t speed_B = (double) 1 / (double) reader->F * XY_MM_PER_REV
			/ XY_STEPS_PER_REV * MIN_TO_US / (double) feedrateB;
	uint16_t speed_Z = (double) 1 / (double) reader->F * Z_MM_PER_REV
			/ XY_STEPS_PER_REV * MIN_TO_US;
	uint16_t stepsA = abs(difA / XY_MM_PER_REV * XY_STEPS_PER_REV);
	uint16_t stepsB = abs(difB / XY_MM_PER_REV * XY_STEPS_PER_REV);
	uint16_t stepsE = abs(dife / E_MM_PER_REV * STEPS_PER_REV);
	uint16_t stepsZ = abs(difz / Z_MM_PER_REV * XY_STEPS_PER_REV);
	uint32_t execution_time = stepsA * speed_A * feedrateA
			+ stepsB * speed_B * feedrateB;
	uint16_t speed_E = execution_time / 1;

	if (stepsA >= head.CurStepsX) {
		if (GetTicks() - head.last_time_X >= speed_A) {
			if (difA < 0) {
				HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_SET);
			}
			HAL_GPIO_TogglePin(STEP_Z_GPIO_Port, STEP_Z_Pin);
//            HAL_GPIO_TogglePin(STEP_X_GPIO_Port, STEP_X_Pin);
			head.last_time_X = GetTicks();
			head.CurStepsX++;
		}
	}
	if (stepsB >= head.CurStepsY) {
		if (GetTicks() - head.last_time_Y >= speed_B) {
			if (difB < 0) {
				HAL_GPIO_WritePin(DIR_E0_GPIO_Port, DIR_E0_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(DIR_E0_GPIO_Port, DIR_E0_Pin, GPIO_PIN_SET);
			}
			HAL_GPIO_TogglePin(STEP_E0_GPIO_Port, STEP_E0_Pin);
//			HAL_GPIO_TogglePin(STEP_E0_GPIO_Port, STEP_E0_Pin);
			head.last_time_Y = GetTicks();
			head.CurStepsY++;
		}
	}
	if (stepsE >= head.CurStepsE) {
		if (GetTicks() - head.last_time_E >= speed_E) {
			if (dife < 0) {
				HAL_GPIO_WritePin(DIR_E1_GPIO_Port, DIR_E1_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(DIR_E1_GPIO_Port, DIR_E1_Pin, GPIO_PIN_SET);
			}
			HAL_GPIO_TogglePin(STEP_E1_GPIO_Port, STEP_E1_Pin);
//			HAL_GPIO_TogglePin(STEP_E1_GPIO_Port, STEP_E1_Pin);
			head.last_time_E = GetTicks();
			head.CurStepsE++;
		}
	}
	if (stepsZ >= head.CurStepsZ) {
		if (GetTicks() - head.last_time_Z >= speed_Z) {
			if (difz < 0) {
				HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_SET);
			}
			HAL_GPIO_TogglePin(STEP_Z_GPIO_Port, STEP_Z_Pin);
//			HAL_GPIO_TogglePin(STEP_Z_GPIO_Port, STEP_Z_Pin);
			head.last_time_Z = GetTicks();
			head.CurStepsZ++;
		}
	}

	if (stepsB <= head.CurStepsY - 1 && stepsA <= head.CurStepsX - 1
			&& stepsE <= head.CurStepsE - 1 && stepsZ <= head.CurStepsZ - 1) {
		head.X = reader->X;
		head.Y = reader->Y;
		head.E = reader->E;
		head.Z = reader->Z;
		head.isrunning = FREE;
		head.Z = reader->Z;
		head.CurStepsZ = 0;
		head.CurStepsX = 0;
		head.CurStepsY = 0;
		head.StepsX = 0;
		head.StepsY = 0;
		head.StepsZ = 0;
		uint8_t msg[10] = "Done\n";
		CDC_Transmit_FS(msg, 5);
	}
}

void G28() {
	while (HAL_GPIO_ReadPin(X_STOP_GPIO_Port, X_STOP_Pin)) {
		if (GetTicks() - head.last_time_X > 6000) {
			HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_E0_GPIO_Port, DIR_E0_Pin, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(STEP_X_GPIO_Port, STEP_X_Pin);
			HAL_GPIO_TogglePin(STEP_E0_GPIO_Port, STEP_E0_Pin);
			head.last_time_X = GetTicks();
		}
	}
	head.X = 0;
	while (HAL_GPIO_ReadPin(Y_STOP_GPIO_Port, Y_STOP_Pin)) {
		if (GetTicks() - head.last_time_X > 6000) {
			HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_E0_GPIO_Port, DIR_E0_Pin, GPIO_PIN_SET);
			HAL_GPIO_TogglePin(STEP_X_GPIO_Port, STEP_X_Pin);
			HAL_GPIO_TogglePin(STEP_E0_GPIO_Port, STEP_E0_Pin);
			head.last_time_X = GetTicks();
		}
	}
	head.Y = 0;
}

void G29(gc_reader *reader) {
	reader->X = 150;
	reader->Y = 150;

	head.isrunning = RUNNING;
	vector2D P = { head.X, head.Y };
	vector2D D = { reader->X, -1 *reader->Y };
	head.d = delta(P, D);
	head.AB_delta = move(head.d);
	reader->F = 1500;
	while(head.isrunning != FREE) {
		G0(reader);
	}

	reader->S = 3.2;
	M280(reader);
//	reader->S = 5.8;
//	M280(reader);
	HAL_GPIO_WritePin(DIR_Z_GPIO_Port, DIR_Z_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(Z_MIN_GPIO_Port, Z_MIN_Pin) == GPIO_PIN_RESET) {
		HAL_GPIO_TogglePin(STEP_Z_GPIO_Port, STEP_Z_Pin);
	}
	reader->S = 7.4;
	M280(reader);
	head.Z = 0;
}

void G92(gc_reader *reader) {
	head.E = reader->E;
	head.Z = reader->Z;
	head.X = reader->X;
	head.Y = reader->Y;
}
