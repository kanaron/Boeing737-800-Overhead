/*
 * LedDriver.c
 *
 *  Created on: Oct 5, 2024
 *      Author: PC_Win
 */


#include "stm32g0xx_hal.h"

#include "LedDriver.h"

I2C_HandleTypeDef *_i2c;

// Function to write a byte via I2C
void IS_IIC_WriteByte(uint8_t Dev_Add, uint8_t Reg_Add, uint8_t Reg_Dat) {
    uint8_t data[2] = { Reg_Add, Reg_Dat };
    HAL_I2C_Master_Transmit(_i2c, Dev_Add, data, 2, HAL_MAX_DELAY);
}

// Function to initialize the IS31FL3733B driver
void IS31FL3733B_Init(I2C_HandleTypeDef *i2c, uint8_t globalCurrent) {
    uint8_t i;

    _i2c = i2c;

    HAL_Delay(10);

    // Unlock register
    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x03);  // Select page 3: function registers
    IS_IIC_WriteByte(Addr_GND_GND, 0x00, 0x00);  // Enable software shutdown

    // Configure LED control registers
    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x00);  // Select page 0: control registe
    for (i = 0; i < 0x18; i++) {
        IS_IIC_WriteByte(Addr_GND_GND, i, 0xFF);  // Turn on all LEDs
    };;

    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x03);  // Select page 3: function registers
    IS_IIC_WriteByte(Addr_GND_GND, 0x10, 0x07);

    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x03);  // Select page 3: function registers
    IS_IIC_WriteByte(Addr_GND_GND, 0x0F, 0x07);

    // Release shutdown, set normal operation, and configure global current
    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x03);  // Select page 3: function registers
    IS_IIC_WriteByte(Addr_GND_GND, 0x00, 0x11);  // Release shutdown
    IS_IIC_WriteByte(Addr_GND_GND, 0x01, globalCurrent);  // Set global current
}

// Test mode with breathing effect
void IS31FL3733B_TestMode1(void) {
    int i, t;
    while (1) {
        // Fade in
        IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
        IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x01);  // Select page 1: PWM registers
        for (t = 0; t <= 255; t++) {
            for (i = 0; i < 192; i++) {
                IS_IIC_WriteByte(Addr_GND_GND, i, t);  // Update PWM values
            }
        }
        HAL_Delay(1000);  // Delay for 1 second

        // Fade out
        IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
        IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x01);  // Select page 1: PWM registers
        for (t = 255; t >= 0; t--) {
            for (i = 0; i < 192; i++) {
                IS_IIC_WriteByte(Addr_GND_GND, i, t);  // Update PWM values
            }
        }
        HAL_Delay(1000);  // Delay for 1 second
    }
}

void IS31FL3733B_SetAllLeds(uint8_t brightness)
{
	IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
	IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x01);  // Select page 1: PWM registers
	for (int i = 0; i < 192; i++) {
		IS_IIC_WriteByte(Addr_GND_GND, i, brightness);  // Update PWM values
	}
}



