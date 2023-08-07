/*
 * TMC2208.cpp
 *
 *  Created on: Jul 2, 2023
 *      Author: Morroway
 */


#include "TMC2208.hpp"

void SelectAxis(uint16_t axis)
{
	switch(axis)
	{
	case X_AXIS: TMC_S1(GPIO_PIN_SET); TMC_S2(GPIO_PIN_RESET); TMC_S3(GPIO_PIN_SET); break;
	case Y_AXIS: TMC_S1(GPIO_PIN_SET); TMC_S2(GPIO_PIN_SET); TMC_S3(GPIO_PIN_SET); break;
	case Z_AXIS: TMC_S1(GPIO_PIN_RESET); TMC_S2(GPIO_PIN_SET); TMC_S3(GPIO_PIN_SET); break;
	case E0_AXIS: TMC_S1(GPIO_PIN_RESET); TMC_S2(GPIO_PIN_RESET); TMC_S3(GPIO_PIN_SET); break;
	case E1_AXIS: TMC_S1(GPIO_PIN_RESET); TMC_S2(GPIO_PIN_SET); TMC_S3(GPIO_PIN_RESET); break;
	}
}

bool TMC2208::DRV_STATUS(uint32_t *data) {
	if (write_only) return 1;
	bool b = sendDatagram(TMC2208_READ|REG_DRV_STATUS, data);
	return b;
}

uint32_t TMC2208::DRV_STATUS() {
	uint32_t data = 0;
	DRV_STATUS(&data);
	return data;
}

uint8_t TMC2208::test_connection() {
	uint32_t drv_status;
	DRV_STATUS(&drv_status);
	switch (drv_status) {
	    case 0xFFFFFFFF: return 1;
	    case 0: return 2;
	    default: return 0;
	}
}

TMC2208::TMC2208(UART_HandleTypeDef * SerialPort, bool has_rx) {
	write_only = !has_rx;
	HWSerial = SerialPort;
}

bool TMC2208::sendDatagram(uint8_t addr, uint32_t *data, uint8_t len) {
	uint8_t datagram[] = {TMC2208_SYNC, TMC2208_SLAVE_ADDR, addr, 0x00};
	datagram[len] = calcCRC(datagram, len);
	uint64_t out = 0x00000000UL;
	out = _sendDatagram(*HWSerial, datagram, len, replyDelay);

	uint8_t out_datagram[] = {(uint8_t)(out>>56), (uint8_t)(out>>48), (uint8_t)(out>>40), (uint8_t)(out>>32), (uint8_t)(out>>24), (uint8_t)(out>>16), (uint8_t)(out>>8), (uint8_t)(out>>0)};
	if (calcCRC(out_datagram, 7) == (uint8_t)(out&0xFF)) {
		*data = out>>8;
		return 0;
	} else {
		return 1;
	}
}

uint64_t TMC2208::_sendDatagram(UART_HandleTypeDef &serPtr, uint8_t datagram[], uint8_t len, uint16_t replyDelay) {
	uint64_t out = 0x00000000UL;
	uint8_t data;
	while(__HAL_UART_GET_FLAG(&serPtr, UART_FLAG_RXNE)) HAL_UART_Receive(&serPtr, &data, 1, 1000);

	for(int i=0; i<=len; i++) HAL_UART_Transmit(&serPtr, &datagram[i], 1, 1000);
	for(int byte=0; byte<4; byte++) {
	    if (__HAL_UART_GET_FLAG(&serPtr, UART_FLAG_RXNE)) {
	        HAL_UART_Receive(&serPtr, &data, 1, 0);
	    }
	}
	HAL_Delay(replyDelay);

	while(__HAL_UART_GET_FLAG(&serPtr, UART_FLAG_RXNE)) {
		HAL_UART_Receive(&serPtr, &data, 1, 1000);
		out <<= 8;
		out |= data&0xFF;	}
	return out;
}

uint8_t TMC2208::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}
