#include <stdlib.h>
#include "usart.h"

typedef struct Matrix {
    int n, m;
    double val[3][4];
} Matrix;

void matrix_init(Matrix *mat, int n, int m);

uint8_t GE(Matrix *mat);
