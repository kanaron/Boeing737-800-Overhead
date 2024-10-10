/*
 * Dot.c
 *
 *  Created on: Oct 8, 2024
 *      Author: PC_Win
 */


#include "Dot.h"
#include "LedDriver.h"

void Dot_init(Dot *dot, uint8_t addr) {
    dot->address = addr;
    dot->state = false;
}

void Dot_turn_on(Dot *dot) {
    dot->state = true;
    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    	IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x01);  // Select page 1: PWM registers
    // Write the state to the LED driver
    IS_IIC_WriteByte(Addr_GND_GND, dot->address, 0xFF);  // Turn on LED
}

void Dot_turn_off(Dot *dot) {
    dot->state = false;
    IS_IIC_WriteByte(Addr_GND_GND, 0xFE, 0xC5);  // Unlock FDh
    	IS_IIC_WriteByte(Addr_GND_GND, 0xFD, 0x01);  // Select page 1: PWM registers
    // Write the state to the LED driver
    IS_IIC_WriteByte(Addr_GND_GND, dot->address, 0x00);  // Turn off LED
}

void Dot_update_state(Dot *dot, bool new_state) {
    if (new_state) {
        Dot_turn_on(dot);
    } else {
        Dot_turn_off(dot);
    }
}
