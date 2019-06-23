#include "matrix.h"

#include <stdio.h>
#include <string.h>

void vector_print(int size, float v[]) {
	int i;
	printf("[");
	for(i=0; i<size; i++)
			printf("%f, ", v[i]);
	printf("]\n");
}

void vector_add(int len, float _a[], float _b[], float dest[]) {

	float *a = _a;
	float *b = _b;
	
	float aux_v[len];
	
	if( dest == _a) {
		memcpy(aux_v, _a, sizeof(float)*len);
		a = aux_v;
	}
	if( dest == _b) {
		memcpy(aux_v, _b, sizeof(float)*len);
		b = aux_v;
	}
	
	int i;
	for(i=0; i<len; i++)
		dest[i] = a[i]+b[i];
}

void vector_sub(int len, float _a[], float _b[], float dest[]) {

	float *a = _a;
	float *b = _b;
	
	float aux_v[len];
	
	if( dest == _a) {
		memcpy(aux_v, _a, sizeof(float)*len);
		a = aux_v;
	}
	if( dest == _b) {
		memcpy(aux_v, _b, sizeof(float)*len);
		b = aux_v;
	}
	
	int i;
	for(i=0; i<len; i++)
		dest[i] = a[i]-b[i];
}

void matrix_print(int rows, int cols, float m[][cols]) {
	int i,j;
	printf("[");
	for(i=0; i<rows; i++) {
		printf("[");
		for(j=0; j<cols; j++)
			printf("%.10f, ", m[i][j]);
		printf("],\n");
	}
	printf("]\n");
}

void matrix_set_identity(int size, float m[][size]) {
	memset(m, 0, sizeof(float)*size*size);
	
	int i;
	for(i=0; i<size; i++)
		m[i][i] = 1.;
}

void matrix_transpose(int rows, int cols, float s[][cols], float d[][rows]) {
	int i, j;
	
	for(i=0; i<rows; i++)
		for(j=0; j<cols; j++)
			d[j][i] = s[i][j];
}

void matrix_scalar_mul(int rows, int cols, float m[][cols], float k, float d[][cols]) {
	int i, j;
	for(i=0; i<rows; i++)
		for(j=0; j<cols; j++)
			d[i][j] = k*m[i][j];
}

void matrix_add(int rows, int cols, float _a[][cols], float _b[][cols], float dest[][cols]) {
	
	float (*a)[cols] = _a;
	float (*b)[cols] = _b;
	
	float aux_m[rows][cols];
	
	if( dest == _a) {
		memcpy(aux_m, _a, sizeof(float)*rows*cols);
		a = aux_m;
	}
	if( dest == _b) {
		memcpy(aux_m, _b, sizeof(float)*rows*cols);
		b = aux_m;
	}
	
	int i,j;
	for(i=0; i<rows; i++)
		for(j=0; j<cols; j++)
			dest[i][j] = a[i][j]+b[i][j];
}

void matrix_sub(int rows, int cols, float _a[][cols], float _b[][cols], float dest[][cols]) {
	
	float (*a)[cols] = _a;
	float (*b)[cols] = _b;
	
	float aux_m[rows][cols];
	
	if( dest == _a) {
		memcpy(aux_m, _a, sizeof(float)*rows*cols);
		a = aux_m;
	}
	if( dest == _b) {
		memcpy(aux_m, _b, sizeof(float)*rows*cols);
		b = aux_m;
	}
	
	int i,j;
	for(i=0; i<rows; i++)
		for(j=0; j<cols; j++)
			dest[i][j] = a[i][j]-b[i][j];
}

void square_matrix_product(int size, float _left_m[][size], float _right_m[][size], float dest_m[][size]) {
	
	float (*left_m)[size] = _left_m;
	float (*right_m)[size] = _right_m;
	
	float aux_m[size][size];
	
	if( dest_m == _left_m) {
		memcpy(aux_m, _left_m, sizeof(float)*size*size);
		left_m = aux_m;
	}
	if( dest_m == _right_m) {
		memcpy(aux_m, _right_m, sizeof(float)*size*size);
		right_m = aux_m;
	}
	
	int i,j,k;
	for(i=0; i<size; i++)	//TODO inefficient
		for(j=0; j<size; j++) {
			dest_m[i][j] = 0.;
			for(k=0; k<size; k++)
				dest_m[i][j] += left_m[i][k]*right_m[k][j];
		}
}

void matrix_product(int r1, int c1, int c2, float left_m[][c1], float right_m[][c2], float dest_m[][c2]) {
	int i, j, k;
	for(i=0; i<r1; i++)	//TODO inefficient
		for(j=0; j<c2; j++) {
			dest_m[i][j] = 0.;
			for(k=0; k<c1; k++)
				dest_m[i][j] += left_m[i][k]*right_m[k][j];
		}
}

void matrix_vector_product(int rows, int cols, float m[][cols], float _v[], float dest[]) {
	
	float* v = _v;
	
	float aux_v[cols];
	if( dest == _v) {
		memcpy(aux_v, _v, sizeof(float)*cols);
		v = aux_v;
	}
	
	int i, j;
	for(i=0; i<rows; i++) {
		dest[i] = 0.;
		for(j=0; j<cols; j++)
			dest[i] += m[i][j]*v[j];
	}
}

float matrix_determinant(int size, float m[][size]) {
	if(size == 0)
		return 0;
	else if(size == 1)
		return m[0][0];
	
	float det = 0.;
	float aux[size-1][size-1];
	
	int i,j;
	for(i=1; i<size; i++)
		for(j=1; j<size; j++)
			aux[i-1][j-1] = m[i][j];
	
	for(i=0; i<size; i++) {
		if(i > 0)
			for(j=1; j<size; j++)
				aux[i-1][j-1] = m[i-1][j];
		if( !(i & 1) )
			det += m[i][0]*matrix_determinant(size-1, aux);
		else
			det -= m[i][0]*matrix_determinant(size-1, aux);
	}
	
	return det;
}

//TODO more efficient, calculating inverse while doing determinant
void matrix_inverse(int size, float _m[][size], float inv[][size]) {
	if(size == 0)
		return;
	else if(size == 1) {
		inv[0][0] = 1. / _m[0][0];
		return;
	}
	
	float (*m)[size] = _m;
	
	float aux_m[size][size];
	
	if(inv == _m) {
		memcpy(aux_m, _m, sizeof(float)*size*size);
		m = aux_m;
	}
	
	float det = matrix_determinant(size, m);
	
	float aux[size-1][size-1];
	float cof_m[size][size];
	
	int i,j,k;
	
	
	for(j=0; j<size; j++) {
		//initialize matrix for new col
		for(i=1; i<size; i++)
			for(k=0; k<size; k++) {
				if( k < j )
					aux[i-1][k] = m[i][k];
				else if( k == j )
					continue;
				else if( k > j )
					aux[i-1][k-1] = m[i][k];
			}
		
		for(i=0; i<size; i++) {
			if(i > 0)
				for(k=0; k<size; k++) {
					if( k < j )
						aux[i-1][k] = m[i-1][k];
					else if( k == j )
						continue;
					else if( k > j )
						aux[i-1][k-1] = m[i-1][k];
				}
			
			if( !((i+j) & 1) )
				cof_m[i][j] = matrix_determinant(size-1, aux);
			else
				cof_m[i][j] = -matrix_determinant(size-1, aux);
		}
	}
	
	float cof_m_t[size][size];
	matrix_transpose(size, size, cof_m, cof_m_t);
	
	
	matrix_scalar_mul(size, size, cof_m_t, 1./det, inv);	
}

/*
int main() {
	float m[3][3];
	matrix_set_identity(3, m);
	
	float k[3][3] = {{1,4,7},{2,5,8},{3,6,9}};
	float n[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
	
	matrix_print(3,3,k);
	float k_t[3][3];
	printf("======\n");
	matrix_transpose(3, 3, k, k_t);
	matrix_print(3,3, k_t);
	printf("\n\n");
	
	printf("transpose 2\n");
	float kk[2][4] = {{1,4,7,2},{2,5,8,1}};
	float kk_t[4][2];
	matrix_print(2,4, kk);
	printf("======\n");
	matrix_transpose(2, 4, kk, kk_t);
	matrix_print(4,2, kk_t);
	printf("\n\n");
	
	
	
	printf("add\n");
	matrix_print(3,3,k);
	matrix_print(3,3,n);
	matrix_add(3,3, k,n,k);
	printf("======\n");	
	matrix_print(3,3,k);
	matrix_print(3,3,n);
	printf("\n\n");
	
	
	
	
	
	square_matrix_product(3, n,k,m);
	
	matrix_print(3,3,n);
	matrix_print(3,3,k);
	printf("======\n");
	matrix_print(3,3,m);
	printf("\n\n");
	
	
	float g[3] = {1,4,8};
	matrix_print(3,3,n);
	vector_print(3,g);
	printf("======\n");
	matrix_vector_product(3,3, n,g,g);
	vector_print(3, g);
	
	
	printf("matrix prod\n");
	float hh[2][4] = {{1,4,7,2},{2,5,8,1}};
	float jj[4][3] = {{1,4,7},{5,8,1},{2,4,5},{5,6,7}};
	matrix_print(2,4,hh);
	matrix_print(4,3,jj);
	printf("======\n");
	float ll[2][3];
	matrix_product(2,4,3,hh,jj,ll);
	matrix_print(2,3,ll);
	
	
	printf("det\n");
	float t[3][3] = {{53,45,63},{87,23,64},{14,25,46}};
	float r = matrix_determinant(3,t);
	matrix_print(3,3,t);
	printf("det: %f\n",r);
	
	
	printf("inverse\n");
	float i[3][3];
	matrix_inverse(3, t, i);
	matrix_print(3,3, i);
	square_matrix_product(3, t,i,i);
	matrix_print(3,3, i);
	
	
	
	printf("matrx_vec\n");
	float mn[2][3]={{12,5,3},{5,7,43}};
	float gh[3] = {1,4,8};
	float d[2];
	matrix_print(2,3,mn);
	vector_print(3,gh);
	printf("======\n");
	matrix_vector_product(2,3, mn,gh,d);
	vector_print(2, d);
	
	
	
	printf("inverse 2 \n");
	float ds[4][4] = {{13,3,5,7},{23,4,5,6},{7,89,34,56},{12,35,74,87}};
	float ii[4][4] = {{13,3,5,7},{23,4,5,6},{7,89,34,56},{12,35,74,87}};
	matrix_inverse(4, ii, ii);
	matrix_print(4,4, ii);
	square_matrix_product(4, ds,ii,ii);
	matrix_print(4,4, ii);
	
	
	
	printf("vector_add\n");
	float dv[5] = {1,3,5,7,9};
	float hg[5] = {2,4,6,1,8};
	vector_print(5,dv);
	vector_print(5,hg);
	vector_sub(5,dv,hg,hg);
	vector_print(5,hg);
	
	printf("\n\n");
	
	float df[4][4];
	matrix_set_identity(4,df);
	
	matrix_print(4,4,df);
	
	
}
*/
/*
int main(){
	float c[2][8] = {{0.00000000, 0.01000000, 0.00005000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, -0.00098000},
{0.00000000, 0.01000000, 0.00005000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00098000}};
	
	
	
	float a[2][8] = {{0.00000494, 0.00000264, 0.00000070, 0.00000000, 0.00000000, 0.00000000, -0.00001190, -0.00000045},
 {0.00000494, 0.00000264, 0.00000070, 0.00000000, 0.00000000, 0.00000000, 0.00001190, 0.00000045}};
	float ct[8][2];
	
	matrix_transpose(2,8,c,ct);
	
	float r[2][2];
	matrix_product(2,8,2, a, ct,r);
	matrix_print(2,2,r);
	
	float cov[8][8] = {{0.02536845, 0.00049368, -0.00000038, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00049368, 0.00026317, 0.00007015, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{-0.00000038, 0.00007015, 0.00003745, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 251.64759827, 11.43842793, 0.27730024, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 11.43844223, 0.55459273, 0.01512569, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 0.27730024, 0.01512569, 0.00055000, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.44938746, 0.01214009},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.01214009, 0.00046282}};

	float a88[8][8] = {{0.00000000, 0.00982163, 0.00004911, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00524281, 0.00002621, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00139932, 0.00000700, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00233166},
{0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00008889}};

	float r88[8][8];
	square_matrix_product(8,a88,cov,r88);
	
	matrix_print(8,8,r88);
	
}
*/
