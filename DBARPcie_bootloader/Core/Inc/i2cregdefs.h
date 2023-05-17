/*
 * i2cregdefs.h
 *
 *  Created on: May 13, 2023
 *      Author: JRozb
 */

#ifndef INC_I2CREGDEFS_H_
#define INC_I2CREGDEFS_H_

#define I2CSLV_REGS_SZ 	0x60

const uint8_t regsMask[I2CSLV_REGS_SZ] = {	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	//0x00 - 0x07
											0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
											0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	//0x10 - 0x17
											0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
											0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x80, 0x00, 0x00,	//0x20 - 0x17
											0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF,
											0x00, 0x7F, 0x07, 0x77, 0xFF, 0xFF, 0xFF, 0xFF,	//0x30 - 0x27
											0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
											0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,	//0x40 - 0x37
											0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
											0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,	//0x50 - 0x47
											0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

typedef struct InfoRegs{
	uint8_t status;
	uint8_t serial[3];
	uint32_t fwVersion;
	uint8_t buildYear;
	uint8_t buildMonth;
	uint8_t buildDay;
	uint8_t buildHour;
	uint8_t buildMin;
	uint8_t buildSec;
	uint8_t hwRevision;
	uint8_t hwVersion;
	char snString[16];
}InfoRegs;

typedef struct BootloaderRegs{
	uint32_t flashOffset;
	uint8_t	flashLock;
	uint8_t flashLoad;
	uint8_t res0[5];
	uint8_t keyCheck;
	uint32_t key;
}BootloaderRegs;

typedef struct I2CMasterRegs{
	uint8_t status;
	uint8_t devAddr;
	uint8_t xferRequest;
	uint8_t length;
	uint8_t wrBuf[4];
	uint8_t rdBuf[4];
	uint8_t res0[3];
	uint8_t errorCode;
}I2CMasterRegs;

typedef struct ThermalsRegs{
	uint8_t Temp_0;
	uint8_t Temp_1;
	uint8_t Fan_0;
	uint8_t Fan_1;
	uint8_t res[12];
}ThermalsRegs;

typedef struct ConfigRegs{
	uint8_t AddrOffset;
	uint8_t Value;
	uint8_t Control;
	uint8_t res[13];
}ConfigRegs;

typedef struct Regs{
	InfoRegs info;
	BootloaderRegs bootlader;
	I2CMasterRegs i2c;
	ThermalsRegs thermals;
	ConfigRegs config;
} Regs;

#endif /* INC_I2CREGDEFS_H_ */
