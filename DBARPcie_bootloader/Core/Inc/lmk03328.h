/*
 * lmk03328.h
 *
 *  Created on: May 15, 2023
 *      Author: JR
 */

#ifndef INC_LMK03328_H_
#define INC_LMK03328_H_

#include "main.h"

#define LMK_ADDR		0x54
#define LMK_REGS_SZ 	172

void lmk03328_write(uint8_t reg, uint8_t value);
uint8_t lmk03328_read(uint8_t reg);
void lmk03328_init(I2C_HandleTypeDef *hi2c);


#endif /* INC_LMK03328_H_ */
