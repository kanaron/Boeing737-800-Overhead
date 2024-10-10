/*
 * Matrix.c
 *
 *  Created on: Oct 8, 2024
 *      Author: PC_Win
 */


#include "Matrix.h"
#include "digits.h"  // Include the digit table

void Matrix_init(Matrix *matrix, uint8_t startingAddress[7]) {
    // Initialize the 7x5 matrix of dots with addresses
    uint8_t address = 0x00;
    for (int row = 0; row < 7; row++) {
    	address = startingAddress[row];
        for (int col = 0; col < 5; col++) {
            Dot_init(&matrix->dots[row][col], address++);
        }
    }
}

void Matrix_clear(Matrix *matrix) {
    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            Dot_turn_off(&matrix->dots[row][col]);
        }
    }
}

void Matrix_set_digit(Matrix *matrix, uint8_t digit) {
    // Clear the matrix before displaying a digit
    Matrix_clear(matrix);

    if (digit > 37) return; // Invalid digit index

    for (int row = 0; row < 5; row++) {
        uint8_t bitmask = digits[digit][row];
        for (int col = 0; col < 7; col++) {
            bool state = bitmask & (1 << (6 - col));
            Dot_update_state(&matrix->dots[col][row], state);
        }
    }
}

