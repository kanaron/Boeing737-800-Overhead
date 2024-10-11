/*
 * Matrix.h
 *
 *  Created on: Oct 8, 2024
 *      Author: PC_Win
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_


#include "Dot.h"

typedef struct {
    Dot dots[7][5];
} Matrix;

void Matrix_init(Matrix *matrix, uint8_t startingAddress[7]);
void Matrix_clear(Matrix *matrix);
void Matrix_set_digit(Matrix *matrix, uint8_t digit);


#endif /* INC_MATRIX_H_ */
