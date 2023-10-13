#include "ge.h"

int check_zero_row(double matrix[3][4], int rows, int columns) {
	
	int num = 0;
	int rows_shifted = 0;

	for(int i = 0; i < rows - rows_shifted; i++) {
		for(int j = 0; j < columns; j++) {
			if(matrix[i][j] != 0) {
				break;
			}
			if(j == columns - 1) {
				num++;
				
				double temp;
				for(int k = 0; k <	columns; k++) {
					temp = matrix[i][k];
					matrix[i][k] = matrix[rows - rows_shifted - 1][k];
					matrix[rows - rows_shifted - 1][k] = temp;
				}	
				
				rows_shifted++;
			}	
		} 
	}

	return num;
}


/*
 *	Function: print_matrix
 *	----------------------------
 *	Print the matrix
 *
 *	arr: The matrix to be printed
 *	rows: number of rows in arr
 *	columns: number of columns in arr
 */
void print_matrix(float arr[3][4], int rows, int columns) {
	for(int i = 0; i < rows; i++) {
		for(int j = 0; j < columns; j++) {
			if(arr[i][j] == 0 || arr[i][j] == -0) {
				cprintf(&huart1,"%.2lf  ", 0.0);
			} else {
				cprintf(&huart1,"%.2lf  ", arr[i][j]);
			}
		}
		cprintf(&huart1,"\n");
	}
	cprintf(&huart1,"\n");
}

/*
 *	Function: gaussian_elimination
 *	------------------------------
 *	Gets matrix as a parameter, then uses Gaussian Elimination to solve
 *
 *	@param matrix
 *	 Input matrix to be solved by gaussian elimation
 *	@param rows
 *	 Number of rows in the matrices
 *	@param columns
 *	 Number of columns in the matrices
 *
 *	@return X 
 *	 The solution vector for the matrix
 */
double* gaussian_elimination(double matrix[3][4], int rows, int columns) 
{
	int zero_rows;

	zero_rows = check_zero_row(matrix, rows, columns);
	rows -= zero_rows;


	// Gauss Jordan Elimination
	int n = rows - 1;
	int i, j, k;	
	double c;
	double *X = malloc(sizeof(double) * rows);
	double sum = 0.0;
	
	// Create Upper Triangular matrix
	for(j = 0; j <= n; j++) {
		for(i = 0; i <= n; i++) {
			if(i > j) {
				c = matrix[i][j] / matrix[j][j];
				for(k = 0; k <= n + 1; k++) {
					matrix[i][k] -= (c * matrix[j][k]);
				}
			}
		}
	}
	
	// Print Upper Triangular matrix for debugging
	/**cprintf(&huart1,"\n\n--------Upper Triangular Matrix--------\n");*/
	/**print_matrix(matrix, rows + zero_rows, columns);*/
		
	// Back Substitution
	X[n] =  matrix[n][n + 1] / matrix[n][n];
	
	for(i = n - 1; i >= 0; i--) {
		sum = 0.0;
		for(j = i + 1; j <= n; j++) {
			sum = sum + matrix[i][j] * X[j];
		}
		X[i] = (matrix[i][n + 1] - sum) / matrix[i][i];
	}
	// End Gauss Elimination
	
	return X;
}

