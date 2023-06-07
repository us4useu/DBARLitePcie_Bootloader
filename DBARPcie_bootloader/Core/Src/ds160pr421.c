/*
 * ds160pr421.c
 *
 *  Created on: 16 maj 2023
 *      Author: JR
 */

#include "ds160pr421.h"

I2C_HandleTypeDef ds160_hi2c;
const uint8_t DS160ADDR[8] = { 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};

const uint8_t CTLE_GAIN_INDEX[16] = {	0x40, 0x43, 0x08, 0x0A, 	// (-0.25/-0.5 dB), (2/4 dB), (2.5/5 dB), (3/6 dB),
										0x11, 0x12, 0x13, 0x1A, 	// (4/7 dB), (4.5/7.5 dB), (5/8 dB), (6/9.5 dB),
										0x1B, 0x23, 0x2B, 0x2C, 	// (7/10 dB), (8/11 dB), (8.5/12.5 dB), (9/13 dB),
										0x2D, 0x35, 0x36, 0x3F};	// (9.5/14.5 dB), (10/15 dB), (10.5/16 dB), (12/18 dB),

void ds160_enable() {
	HAL_GPIO_WritePin(DS_PD_GPIO_Port, DS_PD_Pin, GPIO_PIN_RESET);
}

void ds160_disable() {
	HAL_GPIO_WritePin(DS_PD_GPIO_Port, DS_PD_Pin, GPIO_PIN_SET);
}

void ds160_write(uint8_t id, uint8_t addr, uint8_t reg){
	uint8_t tx_buf[2];
	tx_buf[0] = addr;
	tx_buf[1] = reg;

	HAL_StatusTypeDef st = HAL_OK;
	st = HAL_I2C_Master_Transmit(&ds160_hi2c, DS160ADDR[id]<<1, tx_buf, 2, 1000);
	if(st != HAL_OK) {
		printf("DS160PR421[0x%02x] I2C Write Error!\n", DS160ADDR[id]);
	}
}

uint8_t ds160_read(uint8_t id, uint8_t addr){
	uint8_t tx_buf = addr;
	uint8_t rx_buf;

	HAL_StatusTypeDef st = HAL_OK;
	st = HAL_I2C_Master_Transmit(&ds160_hi2c, DS160ADDR[id]<<1, &tx_buf, 1, 1000);
	st = HAL_I2C_Master_Receive(&ds160_hi2c, DS160ADDR[id]<<1, &rx_buf, 1, 1000);
	//st = HAL_I2C_Mem_Read(&ds160_hi2c, DS160ADDR[id]<<1, &addr, 1, &rx_buf, 1, 1000);
	if(st != HAL_OK) {
		printf("DS160PR421[0x%02x] I2C Read Error!\n", DS160ADDR[id]);
	}

	return rx_buf;
}

void ds160_writetx(uint8_t addr, uint8_t reg) {
	ds160_write(0, addr, reg);
	ds160_write(1, addr, reg);
	ds160_write(4, addr, reg);
	ds160_write(5, addr, reg);
}

void ds160_writerx(uint8_t addr, uint8_t reg) {
	ds160_write(2, addr, reg);
	ds160_write(3, addr, reg);
	ds160_write(6, addr, reg);
	ds160_write(7, addr, reg);
}

void ds160_writeall(uint8_t addr, uint8_t reg) {
	for(uint8_t n = 0; n<8; n++) {
		ds160_write(n, addr, reg);
	}
}

void ds160_init(I2C_HandleTypeDef *hi2c, uint8_t sff){
	ds160_hi2c = *hi2c;

	ds160_writeall(0x0E, 0x04);
	ds160_writeall(0x0F, 0x04);

	ds160_writeall(0x81, CTLE_GAIN_INDEX[15]); //CTLE Index 2

	ds160_writeall(0x82, 0b00000011); //DC gain = 0, TX VOD = 0
	//reset RX detect state machine

	if(sff == SFF8643) {
		uint8_t reg[16];
		printf("Select SFF8643 (Internal connection) \n");
		for(uint8_t n = 0; n<8; n++) {
			ds160_write(n, 0x0E, 0x04);
			ds160_write(n, 0x0F, 0x04);
			reg[n] = ds160_read(n, 0x0F);
		}
	}
	else {
		printf("Select SFF8644 (External connection) \n");
		for(uint8_t n = 0; n<8; n++) {
			ds160_write(n, 0x0E, 0x04);
			ds160_write(n, 0x0F, 0x00);
		}
	}

	HAL_Delay(10);

	ds160_writeall(0x83, 0b00000010);
	ds160_writeall(0x09, 0b00000100);
	ds160_writeall(0x09, 0b00000000);
}

