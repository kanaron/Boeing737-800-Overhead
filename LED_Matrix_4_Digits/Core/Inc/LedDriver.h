/*
 * LedDriver.h
 *
 *  Created on: Oct 5, 2024
 *      Author: PC_Win
 */

#ifndef LEDDRIVER_H
#define LEDDRIVER_H

#include "stm32g0xx_hal.h"

#define Addr_GND_GND 0xA0 // I2C address when ADDR1 and ADDR2 are GND

void IS31FL3733B_Init(I2C_HandleTypeDef *_i2c, uint8_t globalCurrent);
void IS31FL3733B_TestMode1(void);
void IS_IIC_WriteByte(uint8_t Dev_Add, uint8_t Reg_Add, uint8_t Reg_Dat);
void IS31FL3733B_SetAllLeds(uint8_t brightness);



#endif // LEDDRIVER_H
