/*
 * confmem.c
 *
 *  Created on: May 13, 2023
 *      Author: JRozb
 */

#include "confmem.h"

void ConfigMemory_Erase() {
	FLASH_EraseInitTypeDef FLASH_EraseInitStruct;

	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; 	//Erase type set to erase pages( Available other type is mass erase)
	FLASH_EraseInitStruct.Banks = FLASH_BANK_1;
	FLASH_EraseInitStruct.Sector = (FLASH_SECTOR_TOTAL-1);		//Conf mem in last sector
	FLASH_EraseInitStruct.NbSectors = 1;                   	  	//Conf mem takes one sector of flash memory

	uint32_t  errorStatus = 0;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &errorStatus);
	HAL_FLASH_Lock();
}

uint32_t ConfigMemory_Download(ConfMem* conf) {


	uint8_t* destAddress = (uint8_t*)conf;
	//find last saved config
	uint8_t* srcAddress = (uint8_t*)CONF_MEM_BASEADDR;

	uint8_t res0 = *srcAddress;
	if(res0 != 0xAA) {
		printf("No saved configurations found in memory\n");
		//conf->boot = 0xAA; //stay in bootloader if no saved configs
		return 0;
	}

	uint32_t nConfigs = 1;
	uint8_t* nextAddress = srcAddress + CONF_MEM_SZ;
	res0 = *nextAddress;

	while(res0 == 0xAA && srcAddress < (FLASH_END - CONF_MEM_SZ)) {
		srcAddress += CONF_MEM_SZ;
		nextAddress = srcAddress + CONF_MEM_SZ;
		res0 = *nextAddress;
		nConfigs++;
	}

	printf("Found config @0x%08X\n", srcAddress);

	memcpy(destAddress, srcAddress, (size_t)CONF_MEM_SZ);

	return nConfigs;
}

void ConfigMemory_Upload(ConfMem* conf) {

	uint8_t* srcAddress;
	uint32_t ptr = 0;

	uint32_t* destAddress = CONF_MEM_BASEADDR;
	uint8_t res0 = *destAddress;

	//find address without watermark written
	while(res0 == 0xAA && destAddress < (FLASH_END - CONF_MEM_SZ)) {
		destAddress += CONF_MEM_SZ;
		res0 = *destAddress;
	}

	//if whole memory used, erase and start over
	if(res0 == 0xAA) {
		ConfigMemory_Erase();
		destAddress = CONF_MEM_BASEADDR;
	}

	HAL_StatusTypeDef st;

	while(ptr < CONF_MEM_SZ) {
		srcAddress = (uint8_t*)conf + ptr;
		HAL_FLASH_Unlock();
		st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)(destAddress + ptr), (uint32_t)(srcAddress));
		HAL_FLASH_Lock();
		if(st!=HAL_OK) {
			uint32_t err = HAL_FLASH_GetError();
			printf("Flash write error 0x%08X\n", err);
		}
		ptr += (4*FLASH_NB_32BITWORD_IN_FLASHWORD);
	}
	printf("Saved configuration @0x%08X\n", destAddress);
}

