/*
 * TMC2208.hpp
 *
 *  Created on: Jul 2, 2023
 *      Author: Morroway
 */

#ifndef INC_TMC2208_HPP_
#define INC_TMC2208_HPP_

#include "main.h"
#include "stdbool.h"
#include "TMC2208_MACR.h"
#include "TMC2208_REGDEFS.h"
#include "usbd_cdc_if.h"

#define TMC2208STEPPER_VERSION 0x000205 // v0.2.5

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E0_AXIS 3
#define E1_AXIS 4

#define REPLY_DELAY 5



void SelectAxis(uint16_t axis);

class TMC2208 {
public:
	TMC2208(UART_HandleTypeDef * SerialPort, bool has_rx=true);
	uint8_t test_connection();
	uint16_t replyDelay = 5;
private:

	UART_HandleTypeDef* HWSerial = NULL;

	void sendDatagram(uint8_t addr, uint32_t regVal, uint8_t len=7);
	bool sendDatagram(uint8_t addr, uint32_t *data, uint8_t len=3);
	uint64_t _sendDatagram(UART_HandleTypeDef &serPtr, uint8_t datagram[], uint8_t len, uint16_t replyDelay);
	uint8_t calcCRC(uint8_t datagram[], uint8_t len);

	bool DRV_STATUS(uint32_t *data);
	uint32_t DRV_STATUS();

	// Shadow registers
	// Default values assume no changes in OTP
	uint32_t 	GCONF_sr = 			0x00000141UL, // Added default: pdn_disable = 1;
				GSTAT_sr = 			0x00000000UL,
				SLAVECONF_sr = 		0x00000000UL,
				OTP_PROG_sr = 		0x00000000UL,
				OTP_READ_sr = 		0x00000000UL,
				FACTORY_CONF_sr = 	0x00000000UL,
				IHOLD_IRUN_sr = 	0x00010000UL, // Reset default would be IRUN=31 IHOLD=16
				TPOWERDOWN_sr = 	0x00000014UL,
				TPWMTHRS_sr = 		0x00000000UL,
				VACTUAL_sr = 		0x00000000UL,
				CHOPCONF_sr = 		0x10000053UL,
				PWMCONF_sr = 		0xC10D0024UL,
				tmp_sr = 			0x00000000UL;

	bool write_only;
	uint16_t mA_val = 0;
};


#endif /* INC_TMC2208_HPP_ */
