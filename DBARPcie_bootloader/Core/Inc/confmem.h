/*
 * confMem.h
 *
 *  Created on: May 13, 2023
 *      Author: JRozb
 */

#ifndef INC_CONFMEM_H_
#define INC_CONFMEM_H_

#include "main.h"

typedef struct ConfMem {
	uint8_t serial[3];
	uint8_t hwRevision[1];
	uint8_t reserved[251];
	uint8_t boot;
}ConfMem;

#define CONF_MEM_SZ 		sizeof(ConfMem)
#define CONF_MEM_BASEADDR 	(0x08000000+((FLASH_SECTOR_TOTAL-1)*FLASH_SECTOR_SIZE))

void ConfigMemory_Download(ConfMem* conf);
void ConfigMemory_Upload(ConfMem* conf);

#endif /* INC_CONFMEM_H_ */
