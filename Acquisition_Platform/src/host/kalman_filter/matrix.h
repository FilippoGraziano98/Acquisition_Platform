#pragma once



#ifdef __cplusplus
extern "C" {
#endif

/* VECTORS */

void vector_print(int size, float v[]);

void vector_add(int len, float _a[], float _b[], float dest[]);

void vector_sub(int len, float _a[], float _b[], float dest[]);

/* MATRIXES */

void matrix_print(int rows, int cols, float m[][cols]);

void matrix_set_identity(int size, float m[][size]);

void matrix_transpose(int rows, int cols, float s[][cols], float d[][rows]);

void matrix_scalar_mul(int rows, int cols, float m[][cols], float k, float d[][cols]);

void matrix_add(int rows, int cols, float _a[][cols], float _b[][cols], float dest[][cols]);

void matrix_sub(int rows, int cols, float _a[][cols], float _b[][cols], float dest[][cols]);

void square_matrix_product(int size, float _left_m[][size], float _right_m[][size], float dest_m[][size]);

//TODO does not support overlapping between dest and source matrixes
void matrix_product(int r1, int c1, int c2, float left_m[][c1], float right_m[][c2], float dest_m[][c2]);

void matrix_vector_product(int rows, int cols, float m[][cols], float _v[], float dest[]);

float matrix_determinant(int size, float m[][size]);

void matrix_inverse(int size, float m[][size], float inv[][size]);


#ifdef __cplusplus
}
#endif
