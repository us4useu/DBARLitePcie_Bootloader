/*
 * cdcun1208.h
 *
 *  Created on: 16 maj 2023
 *      Author: JR
 */

#ifndef INC_CDCUN1208_H_
#define INC_CDCUN1208_H_

#include "main.h"

#define CDCUN1208_ADDRESS 0x28

void cdcun1208_write(uint8_t addr, uint16_t reg);
uint16_t cdcun1208_read(uint8_t addr);
void cdcun1208_init(I2C_HandleTypeDef *hi2c);

#endif /* INC_CDCUN1208_H_ */
