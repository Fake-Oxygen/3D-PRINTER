/*
 * TMC2208_MACR.h
 *
 *  Created on: Jul 1, 2023
 *      Author: Morroway
 */

#ifndef INC_TMC2208_MACR_H_
#define INC_TMC2208_MACR_H_

#include "TMC2208.hpp"
#include "TMC2208_REGDEFS.h"

#define REGISTER_W(R) sendDatagram(TMC2208_WRITE|REG_##R, R##_sr);
#define REGISTER_R(R) bool b = sendDatagram(TMC2208_READ|REG_##R, data); R##_sr = *data; return b;

#define MOD_REG(REG, SETTING) REG##_sr &= ~SETTING##_bm; REG##_sr |= ((uint32_t)B<<SETTING##_bp)&SETTING##_bm; REGISTER_W(REG);

#define GET_BYTE(REG, SETTING) REG(&REG##_sr); 	return (REG##_sr&SETTING##_bm)	>>SETTING##_bp;
#define GET_BYTE_R(REG, SETTING) REG(&tmp_sr); 	return (tmp_sr&SETTING##_bm)	>>SETTING##_bp;

#define MAKE_STEP(AXIS) HAL_GPIO_TogglePin(AXIS)
#define TMC_S1(STATE) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, STATE)
#define TMC_S2(STATE) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, STATE)
#define TMC_S3(STATE) HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, STATE)

#endif /* INC_TMC2208_MACR_H_ */
