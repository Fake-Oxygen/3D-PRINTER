/*
 * TMC2208Constants.h
 *
 *  Created on: Jul 27, 2023
 *      Author: isan
 */

#ifndef INC_TMC2208CONSTANTS_H_
#define INC_TMC2208CONSTANTS_H_

#define TMC2208_PREAMBULE 0x05
#define TMC2208_SLAVE_ADDR 0
#define TMC2208_MASTER_ADDR 0xFF

#define TMC2208_READ 0
#define TMC2208_WRITE 0b10000000

// register addresses
#define GCONF_ADDR 0x00
#define GSTAT_ADDR 0x01
#define IFCNT_ADDR 0x02
#define SLAVECONF_ADDR 0x03
#define OTP_PROG_ADDR 0x04
#define OTP_READ_ADDR 0x05
#define IOIN_ADDR 0x06
#define FACTORY_CONF_ADDR 0x07
#define IHOLD_IRUN_ADDR 0x10
#define CHOPCONF_ADDR 0x6C
#define DRV_STATUS_ADDR 0x6F

// func results consts
#define NO_ERROR 0

#define SET_RMS_CURR_ERR_CHOPCONF   1
#define SET_RMS_CURR_ERR_IHOLD_IRUN 2

#endif /* INC_TMC2208CONSTANTS_H_ */
