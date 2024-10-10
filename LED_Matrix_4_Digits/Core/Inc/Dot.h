/*
 * Dot.h
 *
 *  Created on: Oct 8, 2024
 *      Author: PC_Win
 */

#ifndef INC_DOT_H_
#define INC_DOT_H_


#include "stm32g0xx_hal.h"
#include <stdbool.h>

// Define a structure for Dot
typedef struct {
    uint8_t address;
    bool state;
} Dot;

// Function prototypes for Dot
void Dot_init(Dot *dot, uint8_t addr);
void Dot_turn_on(Dot *dot);
void Dot_turn_off(Dot *dot);
void Dot_update_state(Dot *dot, bool new_state);


#endif /* INC_DOT_H_ */
