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
	uint8_t res0;
	uint8_t serial[3];
	uint8_t res1[4];
	uint8_t hwVersion;
	uint8_t hwRevision;
	uint8_t res2[6];
	char snString[16];
}ConfMem;

#define CONF_MEM_SZ 		sizeof(ConfMem)
#define CONF_MEM_BASEADDR 	(0x08000000+((FLASH_SECTOR_TOTAL-1)*FLASH_SECTOR_SIZE))

void ConfigMemory_Download(ConfMem* conf);
void ConfigMemory_Upload(ConfMem* conf);

#endif /* INC_CONFMEM_H_ */
