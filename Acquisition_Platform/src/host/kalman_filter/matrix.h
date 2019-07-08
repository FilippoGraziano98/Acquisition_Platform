#pragma once



#ifdef __cplusplus
extern "C" {
#endif

/* VECTORS */

void vector_print(int size, float v[]);

//adds two vectors of size len
void vector_add(int len, float _a[], float _b[], float dest[]);

//subtracts two vectors of size len
void vector_sub(int len, float _a[], float _b[], float dest[]);

/* MATRIXES */

void matrix_print(int rows, int cols, float m[][cols]);

//set to Identity matrix
void matrix_set_identity(int size, float m[][size]);

//calculates the transpose of matrix s
void matrix_transpose(int rows, int cols, float s[][cols], float d[][rows]);

//multiplies matrix per scalar value
void matrix_scalar_mul(int rows, int cols, float m[][cols], float k, float d[][cols]);

//adds two matrices
void matrix_add(int rows, int cols, float _a[][cols], float _b[][cols], float dest[][cols]);

//subtracts two matrices
void matrix_sub(int rows, int cols, float _a[][cols], float _b[][cols], float dest[][cols]);

//calculates the product of two square matrix
void square_matrix_product(int size, float _left_m[][size], float _right_m[][size], float dest_m[][size]);

//calculates the product of two general size matrix [left_m is r1 x c1, right_m is c1 x c2]
void matrix_product(int r1, int c1, int c2, float left_m[][c1], float right_m[][c2], float dest_m[][c2]);

//calculates the product of a matric and a vector
void matrix_vector_product(int rows, int cols, float m[][cols], float _v[], float dest[]);

//calculates the determinant of a matrix
float matrix_determinant(int size, float m[][size]);

//calculates the inverse of matrix m
void matrix_inverse(int size, float m[][size], float inv[][size]);


#ifdef __cplusplus
}
#endif
