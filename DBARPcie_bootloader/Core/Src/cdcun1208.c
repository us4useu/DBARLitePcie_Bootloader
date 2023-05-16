/*
 * cdcun1208.c
 *
 *  Created on: 16 maj 2023
 *      Author: Admin
 */

#include "cdcun1208.h"

I2C_HandleTypeDef cdcun_hi2c;


void cdcun1208_write(uint8_t addr, uint16_t reg){
	uint8_t tx_buf[3];
	tx_buf[0] = addr;
	tx_buf[1] = ((reg & 0xFF00) >> 8);
	tx_buf[2] = (reg & 0xFF);

	HAL_StatusTypeDef st = HAL_OK;
	st = HAL_I2C_Master_Transmit(&cdcun_hi2c, CDCUN1208_ADDRESS<<1, &tx_buf, 3, 1000);
	if(st != HAL_OK) {
		printf("CDCUN1208 I2C Write Error!\n");
	}
}

uint16_t cdcun1208_read(uint8_t addr){
	uint8_t tx_buf = addr;
	uint8_t rx_buf[2];

	HAL_StatusTypeDef st = HAL_OK;
	st = HAL_I2C_Master_Transmit(&cdcun_hi2c, CDCUN1208_ADDRESS<<1, &tx_buf, 1, 1000);
	st = HAL_I2C_Master_Receive(&cdcun_hi2c, CDCUN1208_ADDRESS<<1, rx_buf, 2, 1000);
	if(st != HAL_OK) {
		printf("CDCUN1208 I2C Write Error!\n");
	}

	return ((((uint16_t)rx_buf[0]) << 8) + rx_buf[1]);
}

void cdcun1208_init(I2C_HandleTypeDef *hi2c){
	cdcun_hi2c = *hi2c;

	cdcun1208_write(11, 0x0000); //IN1 selected, LVDS, no division
	cdcun1208_write(0, 0x0218); //OUT1 enabled LVDS
	cdcun1208_write(1, 0x0218); //OUT2 enabled LVDS
	cdcun1208_write(2, 0x0212); //OUT3 enabled LVCMOS, OUT1P only
	cdcun1208_write(3, 0x0212); //OUT4 enabled LVCMOS, OUT1P only
}

