#ifndef _KF_H_
#define _KF_H_

typedef float fp32;

#define max_dim_n 3
#define max_dim_m 3

typedef struct{
	int n, m;
	fp32 val[max_dim_n][max_dim_m];
} Matrix;

typedef struct{
	int dim_x, dim_p;
	Matrix Tr, Trt, H, Ht, B;
	Matrix P, K, x;
	Matrix Q, R;
	Matrix t1, t2, t3, I;
} Kalman_Filter;

extern void Matrix_init(Matrix *p, int n, int m);

extern void Matrix_I_init(Matrix *x, int n, int m);

extern void swap_line(Matrix *x, int from, int to);

extern void Matrix_copy_to(const Matrix *x, Matrix *y);

extern void transpose(const Matrix *x, Matrix *ans);

extern void add(const Matrix *x, const Matrix *y, Matrix *ans);

extern void sub(const Matrix *x, const Matrix *y, Matrix *ans);

extern void dot(fp32 x, const Matrix *y, Matrix *ans);

extern void mul(const Matrix *x, const Matrix *y, Matrix *ans);

extern void inv(Matrix x, Matrix *ans);

extern void kalman_init(Kalman_Filter *Kalman_config, int dim_x, int dim_p, 
	Matrix Tr, Matrix H, Matrix B, fp32 Q, fp32 R);

extern void Kalman_Calc(Kalman_Filter *kf, Matrix *z, Matrix *u);

extern void KF_task(void const *pvParameters);

extern void KF_set_calc(fp32 x, fp32 y);

extern void KF_clear(void);

extern void KF_set_init(void);

extern void KF_set_contorl(fp32 x);

#endif

