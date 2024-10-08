#ifndef STMPE1600_H_
#define STMPE1600_H_
//-------------------------------------------------
#include "stm32l1xx_hal.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>
//-------------------------------------------------
#define EXPANDER_I2C_ADDRESS (0x84)
//-------------------------------------------------
#define GPMR 0x10
#define GPSR 0x12
#define GPDR 0x14
//-------------------------------------------------
#define S0 (1<<0)
#define S1 (1<<1)
#define S2 (1<<2)
#define S3 (1<<3)
#define S4 (1<<4)
#define S5 (1<<5)
#define S6 (1<<6)
#define V2_D1 (1<<7)
// second byte or word MSB
#define V2_D2 (1<<8)
#define V2_D3 (1<<9)
#define V2_D4 (1<<10)
#define V2_DISP_SEL (1<<11)
#define V2_CHIPEN (1<<12)
#define V2_CHIPEN_B (1<<13)
#define V2_CHIPEN_L (1<<14)
#define V2_CHIPEN_R (1<<15)
//-------------------------------------------------
int stmpe1600_ini(void);
int stmpe1600_WriteReg( int index, uint8_t *data, int n_data);
int stmpe1600_ReadReg(int index, uint8_t *data, int n_data);
//-------------------------------------------------
#endif /* STMPE1600_H_ */