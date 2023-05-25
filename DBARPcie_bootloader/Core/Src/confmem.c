/*
 * confmem.c
 *
 *  Created on: May 13, 2023
 *      Author: JRozb
 */

#include "confmem.h"

void ConfigMemory_Download(ConfMem* conf) {

	uint8_t* destAddress = (uint8_t*)conf;
	//find last saved config
	uint8_t* srcAddress = (uint8_t*)CONF_MEM_BASEADDR;

	uint8_t res0 = *srcAddress;
	if(res0 != 0xAA) {
		printf("No saved configurations found in memory\n");
		conf->boot = 0xAA; //stay in bootloader if no saved configs
		return;
	}

	uint8_t* nextAddress = srcAddress + sizeof(conf);
	res0 = *nextAddress;

	while(res0 == 0xAA) {
		srcAddress += sizeof(conf);
		nextAddress = srcAddress + sizeof(conf);
		res0 = *nextAddress;
	}

	printf("Found config @0x%08X\n", srcAddress);

	memcpy(destAddress, srcAddress, (size_t)CONF_MEM_SZ);
}

void ConfigMemory_Upload(ConfMem* conf) {
	HAL_StatusTypeDef st;
	FLASH_EraseInitTypeDef FLASH_EraseInitStruct;

	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; 	//Erase type set to erase pages( Available other type is mass erase)
	FLASH_EraseInitStruct.Banks = FLASH_BANK_1;
	FLASH_EraseInitStruct.Sector = (FLASH_SECTOR_TOTAL-1);		//Conf mem in last sector
	FLASH_EraseInitStruct.NbSectors = 1;                   	  	//Conf mem takes one sector of flash memory

	uint32_t  errorStatus = 0;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &errorStatus);
	uint8_t* srcAddress;
	uint32_t ptr = 0;

	while(ptr < CONF_MEM_SZ) {
		srcAddress = (uint8_t*)conf + ptr;
		st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)(CONF_MEM_BASEADDR+ptr), (uint32_t)(srcAddress));
		if(st!=HAL_OK) {
			printf("Flash write error\n");
			uint32_t err = HAL_FLASH_GetError();
			char dbg[12];
			sprintf(dbg, "0x%08x\n", err);
			printf(dbg);

		}
		ptr+=(4*FLASH_NB_32BITWORD_IN_FLASHWORD);
	}

	HAL_FLASH_Lock();

}

