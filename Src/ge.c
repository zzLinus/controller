#include "math.h"
#include "string.h"
#include "stdint.h"
#include "ge.h"

const double MATRIX_EPS = 1e-6;

void matrix_init(Matrix *mat, int n, int m) {
    mat->n = n;
    mat->m = m;
    memset(mat->val, 0, sizeof(mat->val));
}

void swap(double *x, double *y) {
    double temp = *x;
    *x = *y;
    *y = temp;
}

void swapLine(Matrix *mat, int x, int y) {
    for (int i = 0; i < mat->m; i++) {
        swap(&mat->val[x][i], &mat->val[y][i]);
    }
}

uint8_t GE(Matrix *mat) {
    if (mat->n > mat->m) return 0;
    for (int i = 0; i < mat->n; i++) {
        int maxArg = i;
        for (int j = i + 1; j < mat->n; j++) {
            if (fabs(mat->val[j][i]) > fabs(mat->val[maxArg][i])) {
                maxArg = j;
            }
        }
        swapLine(mat, maxArg, i);
        if (fabs(mat->val[i][i]) < MATRIX_EPS) {
            return 0;
        }
        for (int j = 0; j < mat->n; j++) {
            if (i == j) continue;
            double c = mat->val[j][i] / mat->val[i][i];
            for (int k = i; k < mat->m; k++) {
                mat->val[j][k] -= mat->val[i][k] * c;
            }
        }
    }
    for (int i = 0; i < mat->n; i++) {
        for(int k = mat->n; k < mat->m; k++) {
            mat->val[i][k] /= mat->val[i][i];
        }
        mat->val[i][i] = 1.0;
    }
    return 1;
}