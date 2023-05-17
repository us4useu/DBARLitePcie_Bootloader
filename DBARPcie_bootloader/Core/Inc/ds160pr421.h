/*
 * ds160pr421.h
 *
 *  Created on: 16 maj 2023
 *      Author: JR
 */

#ifndef INC_DS160PR421_H_
#define INC_DS160PR421_H_

#include "main.h"

void ds160_enable();
void ds160_disable();
void ds160_write(uint8_t id, uint8_t addr, uint8_t reg);
uint8_t ds160_read(uint8_t id, uint8_t addr);
void ds160_writeall(uint8_t addr, uint8_t reg);
void ds160_init(I2C_HandleTypeDef *hi2c);



#endif /* INC_DS160PR421_H_ */
