/*
 * ds160pr421.c
 *
 *  Created on: 16 maj 2023
 *      Author: JR
 */

#include "ds160pr421.h"

I2C_HandleTypeDef ds160_hi2c;
const uint8_t DS160ADDR[8] = { 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};

void ds160_passI2Chandle(I2C_HandleTypeDef *hi2c){
	ds160_hi2c = *hi2c;
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
	if(st != HAL_OK) {
		printf("DS160PR421[0x%02x] I2C Read Error!\n", DS160ADDR[id]);
	}

	return rx_buf;
}

void ds160_writeall(uint8_t addr, uint8_t reg) {
	for(uint8_t n = 0; n<8; n++) {
		ds160_write(n, addr, reg);
	}
}

void ds160_init(I2C_HandleTypeDef *hi2c){
	ds160_hi2c = *hi2c;

	ds160_writeall(0x81, 0x40); //CTLE Index 0
	ds160_writeall(0x0E, 0x04); //Overwrite SEL pin
	ds160_writeall(0x0F, 0x00); //Select port A
	ds160_writeall(0x82, 0b00000011); //DC gain = 0, TX VOD = 0
}

