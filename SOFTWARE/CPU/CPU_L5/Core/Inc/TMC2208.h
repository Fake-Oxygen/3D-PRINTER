/*
 * TMC2208Constants.h
 *
 *  Created on: Jul 27, 2023
 *      Author: isan
 */

#ifndef INC_TMC2208_H_
#define INC_TMC2208_H_

#include <stdint.h>
#include "stm32l5xx.h"
#include "main.h"
#include "TMC2208Constants.h"
#include <cstring>
#include <cstdlib>

extern uint8_t transEnd;
extern uint8_t commData[12];

// state registers
extern uint8_t ifcntVal;

// write-only registers values
extern uint8_t irunVal;
extern uint8_t iholdVal;
extern uint8_t iholdDelayVal;

// saved parameters
extern float RSense;

void sendWriteDatagram(UART_HandleTypeDef* uart, uint8_t registerAddress, uint8_t* data);
uint8_t sendReadDatagram(UART_HandleTypeDef* uart, uint8_t registerAddress, uint8_t* responseData);
void calcCRC(uint8_t* datagram, uint8_t len);
uint8_t checkCRC(uint8_t* dataReceived, int size);

uint32_t getRegVal(UART_HandleTypeDef* uart, uint8_t regAddr);
void setRegVal(UART_HandleTypeDef* uart, uint8_t regAddr, uint32_t value);
uint8_t setRegValSafe(UART_HandleTypeDef* uart, uint8_t regAddr, uint32_t value);

void initTMC2208Registers(UART_HandleTypeDef* uart);

//Select TMC2208 IC for programming
void selectAxis(uint8_t axis);

// default multiplier should be 0.5 and RS 0.11
uint8_t setRMSCurrent(UART_HandleTypeDef* uart, uint16_t mA, float multiplier, float RS);
uint8_t setRMSCurrentDefault(UART_HandleTypeDef* uart, uint16_t mA);
float getRMSCurrent(UART_HandleTypeDef* uart, float RS);
float getRMSCurrentDefault(UART_HandleTypeDef* uart);
uint32_t getIHOLD_IRUN();

uint8_t getBit(uint32_t value, uint8_t num);
uint32_t setBit(uint32_t value, uint8_t num, uint8_t newValue);

uint8_t getVsense(uint32_t chopconf);
uint32_t setVsense(uint32_t chopconf, uint8_t vsense);
uint32_t setIRUN(uint32_t ihold_irun, uint8_t irun);
uint32_t setIHOLD(uint32_t ihold_irun, uint8_t ihold);
uint8_t getPDNDisable(uint32_t gconf);
uint32_t setPDNDisable(uint32_t gconf, uint8_t pdnDisable);
uint8_t getIScaleAnalog(uint32_t gconf);
uint32_t setIScaleAnalog(uint32_t gconf, uint8_t iScaleAnalog);
uint8_t getMstepRegSelect(uint32_t gconf);
uint32_t setMstepRegSelect(uint32_t gconf, uint8_t mstepRegSelect);
uint8_t getTOFF(uint32_t chopconf);
uint32_t setTOFF(uint32_t chopconf, uint8_t toff);
uint8_t getMRESVal(uint32_t chopconf);
uint32_t setMRESVal(uint32_t chopconf, uint8_t mres);
uint16_t getMRES(uint32_t chopconf);
uint32_t setMRES(uint32_t chopconf, uint16_t mres);

uint8_t isResetFlag(uint32_t gstat);
uint8_t isDrvErrFlag(uint32_t gstat);
uint8_t isUvCpFlat(uint32_t gstat);

#endif /* INC_TMC2208_H_ */
