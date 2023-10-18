/*
 * TMC2208.c
 *
 *  Created on: Jul 26, 2023
 *      Author: isan
 */
#include "TMC2208.h"

uint8_t transEnd;
uint8_t commData[12];

// state registers
uint8_t ifcntVal = 0;

// write-only registers values
uint8_t irunVal = 31;
uint8_t iholdVal = 0;
uint8_t iholdDelayVal = 0;

// saved parameters
float RSense = 0.11;

void sendWriteDatagram(UART_HandleTypeDef* uart, uint8_t registerAddress, uint8_t* data) {
	uint8_t datagram[8];
	datagram[0] = TMC2208_PREAMBULE;
	datagram[1] = TMC2208_SLAVE_ADDR;
	datagram[2] = registerAddress | TMC2208_WRITE;
	datagram[3] = data[3];
	datagram[4] = data[2];
	datagram[5] = data[1];
	datagram[6] = data[0];
	calcCRC(datagram, 8);
	HAL_UART_Transmit(uart, datagram, 8, 1000);
	HAL_UART_AbortReceive_IT(uart);
	HAL_UART_Receive_IT(uart, commData, 12);
	HAL_Delay(10);
}

uint8_t sendReadDatagram(UART_HandleTypeDef* uart, uint8_t registerAddress, uint8_t* responseData) {
	transEnd = 0;
	uint8_t accessDatagram[4];
	accessDatagram[0] = TMC2208_PREAMBULE;
	accessDatagram[1] = TMC2208_SLAVE_ADDR;
	accessDatagram[2] = registerAddress | TMC2208_READ;
	calcCRC(accessDatagram, 4);
	HAL_UART_Transmit(uart, accessDatagram, 4, 1000);
	while(transEnd == 0) {}

	responseData[0] = commData[10];
	responseData[1] = commData[9];
	responseData[2] = commData[8];
	responseData[3] = commData[7];

	HAL_Delay(10);

	return checkCRC(commData + 4, 8);
}

uint8_t checkCRC(uint8_t* dataReceived, int size) {
	uint8_t *compareBytes = (uint8_t*)malloc(size);
	memcpy(compareBytes, dataReceived, size); // copy 8 bytes of read reply frame from commData

	calcCRC(compareBytes, size);
	uint8_t isCRCCorrect = (compareBytes[size - 1] == dataReceived[size - 1]);
	free(compareBytes);
	return isCRCCorrect;
}

void calcCRC(uint8_t* datagram, uint8_t len) {
	uint8_t* crc = datagram + (len - 1);
	uint8_t currentByte;

	*crc = 0;
	for(int i = 0; i < (len - 1); i++) {
		currentByte = datagram[i];
		for(int j = 0; j < 8; j++) {
			if((*crc >> 7) ^ (currentByte & 0x01)) {
				*crc = (*crc << 1) ^ 0x07;
			} else {
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		}
	}
}

uint32_t getRegVal(UART_HandleTypeDef* uart, uint8_t regAddr) {
	uint8_t result[4], is_crc_ok = 0;
	while(!is_crc_ok) {
		is_crc_ok = sendReadDatagram(uart, regAddr, result);
	}
	return (result[3] << 24) | (result[2] << 16) | (result[1] << 8) | (result[0] << 0);
}

void setRegVal(UART_HandleTypeDef* uart, uint8_t regAddr, uint32_t value) {
	uint8_t data[4];
	data[3] = value >> 24;
	data[2] = value >> 16;
	data[1] = value >> 8;
	data[0] = value;
	sendWriteDatagram(uart, regAddr, data);
}

uint8_t setRegValSafe(UART_HandleTypeDef* uart, uint8_t regAddr, uint32_t value) {
	setRegVal(uart, regAddr, value);
	uint32_t newIFCNT = getRegVal(uart, IFCNT_ADDR);
	uint8_t result = ((ifcntVal + 1) % 256) == newIFCNT;
	ifcntVal = newIFCNT;
	return result;
}

void initTMC2208Registers(UART_HandleTypeDef* uart) {
	uint32_t val = getRegVal(uart, IFCNT_ADDR);
	ifcntVal = (uint8_t)val;
}

uint8_t setRMSCurrent(UART_HandleTypeDef* uart, uint16_t mA, float multiplier, float RS) {
	RSense = RS;
	uint8_t CS = 32.0 * 1.41421 * mA / 1000.0 * (RS + 0.02) / 0.325 - 1;
	uint32_t chopconf = getRegVal(uart, CHOPCONF_ADDR);
	// If Current Scale is too low, turn on high sensitivity R_sense and calculate again
	if(CS < 16) {
		chopconf = setVsense(chopconf, 1);
		CS = 32.0 * 1.41421 * mA / 1000.0 * (RS + 0.02) / 0.180 - 1;
	} else if(getVsense(chopconf)) { // If CS >= 16, turn off high_sense_r if it's currently ON
		chopconf = setVsense(chopconf, 0);
	}
	uint8_t result = setRegValSafe(uart, CHOPCONF_ADDR, chopconf);
	if(!result) return SET_RMS_CURR_ERR_CHOPCONF;
	uint32_t ihold_irun = getIHOLD_IRUN();
	ihold_irun = setIRUN(ihold_irun, CS);
	ihold_irun = setIHOLD(ihold_irun, CS * multiplier);
	result = setRegValSafe(uart, IHOLD_IRUN_ADDR, ihold_irun);
	if(!result) return SET_RMS_CURR_ERR_IHOLD_IRUN;
	return NO_ERROR;
}

uint8_t setRMSCurrentDefault(UART_HandleTypeDef* uart, uint16_t mA) {
	setRMSCurrent(uart, mA, 0.5, 0.11);
}

uint32_t getIHOLD_IRUN() {
	return (iholdDelayVal << 16) | (irunVal << 8) | (iholdVal << 0);
}

float getRMSCurrent(UART_HandleTypeDef* uart, float RS) {
	uint32_t chopconf = getRegVal(uart, CHOPCONF_ADDR);
	uint8_t vsense = getVsense(chopconf);
	return (float)(irunVal + 1) / 32.0 * (vsense ? 0.180 : 0.325) / (RS + 0.02) / 1.41421 * 1000;
}

float getRMSCurrentDefault(UART_HandleTypeDef* uart) {
	return getRMSCurrent(uart, RSense);
}

uint8_t getBit(uint32_t value, uint8_t num) {
	return (value & (1 << num)) > 0;
}

uint32_t setBit(uint32_t value, uint8_t num, uint8_t newValue) {
	return (value & ~(1 << num)) | (newValue << num);
}

uint8_t getVsense(uint32_t chopconf) {
	return getBit(chopconf, 17);
}

uint32_t setVsense(uint32_t chopconf, uint8_t vsense) {
	return setBit(chopconf, 17, vsense);
}

uint32_t setIRUN(uint32_t ihold_irun, uint8_t irun) {
	irunVal = irun;
	return (ihold_irun & ~(0b11111 << 5)) | (irun << 5);
}

uint32_t setIHOLD(uint32_t ihold_irun, uint8_t ihold) {
	iholdVal = ihold;
	return (ihold_irun & ~0b11111) | ihold;
}

uint8_t getPDNDisable(uint32_t gconf) {
	return getBit(gconf, 6);
}

uint32_t setPDNDisable(uint32_t gconf, uint8_t pdnDisable) {
	return setBit(gconf, 6, pdnDisable);
}

uint8_t getIScaleAnalog(uint32_t gconf) {
	return gconf & 0b1;
}

uint32_t setIScaleAnalog(uint32_t gconf, uint8_t iScaleAnalog) {
	return (gconf & ~0b1) | iScaleAnalog;
}

uint8_t getMstepRegSelect(uint32_t gconf) {
	return getBit(gconf, 7);
}

uint32_t setMstepRegSelect(uint32_t gconf, uint8_t mstepRegSelect) {
	return setBit(gconf, 7, mstepRegSelect);
}

uint8_t getTOFF(uint32_t chopconf) {
	return chopconf & 0b1111;
}

uint32_t setTOFF(uint32_t chopconf, uint8_t toff) {
	return (chopconf & ~0b1111) | toff;
}

uint8_t getMRESVal(uint32_t chopconf) {
	return (chopconf >> 24) & 0b1111;
}

uint32_t setMRESVal(uint32_t chopconf, uint8_t mres) {
	return (chopconf & ~(0b1111 << 24)) | (mres << 24);
}

uint16_t getMRES(uint32_t chopconf) {
	uint8_t mres = getMRESVal(chopconf);
	return 1 << (8 - mres);
}

uint32_t setMRES(uint32_t chopconf, uint16_t mres) {
	uint8_t newMRES;
	switch(mres) {
	case 256:
		newMRES = 0;
		break;
	case 128:
		newMRES = 1;
		break;
	case 64:
		newMRES = 2;
		break;
	case 32:
		newMRES = 3;
		break;
	case 16:
		newMRES = 4;
		break;
	case 8:
		newMRES = 5;
		break;
	case 4:
		newMRES = 6;
		break;
	case 2:
		newMRES = 7;
		break;
	case 1:
		newMRES = 8;
		break;
	}

	return setMRESVal(chopconf, newMRES);
}

uint8_t isResetFlag(uint32_t gstat) {
	return gstat & 0b001;
}

uint8_t isDrvErrFlag(uint32_t gstat) {
	return gstat & 0b010;
}

uint8_t isUvCpFlat(uint32_t gstat) {
	return gstat & 0b100;
}

void selectAxis(uint8_t axis) {
	switch(axis){
	case X_AXIS:
		HAL_GPIO_WritePin(TMC_S1_GPIO_Port, TMC_S1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TMC_S2_GPIO_Port, TMC_S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TMC_S3_GPIO_Port, TMC_S3_Pin, GPIO_PIN_SET);
		break;
	case Y_AXIS:
		HAL_GPIO_WritePin(TMC_S1_GPIO_Port, TMC_S1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TMC_S2_GPIO_Port, TMC_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TMC_S3_GPIO_Port, TMC_S3_Pin, GPIO_PIN_SET);
		break;
	case Z_AXIS:
		HAL_GPIO_WritePin(TMC_S1_GPIO_Port, TMC_S1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TMC_S2_GPIO_Port, TMC_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TMC_S3_GPIO_Port, TMC_S3_Pin, GPIO_PIN_SET);
		break;
	case E0_AXIS:
		HAL_GPIO_WritePin(TMC_S1_GPIO_Port, TMC_S1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TMC_S2_GPIO_Port, TMC_S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TMC_S3_GPIO_Port, TMC_S3_Pin, GPIO_PIN_SET);
		break;
	case E1_AXIS:
		HAL_GPIO_WritePin(TMC_S1_GPIO_Port, TMC_S1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TMC_S2_GPIO_Port, TMC_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TMC_S3_GPIO_Port, TMC_S3_Pin, GPIO_PIN_RESET);
		break;
	}
}
