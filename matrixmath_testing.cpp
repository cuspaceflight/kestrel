#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

void NormalizeTay3x3(float* A);
float Dot(float* A, float* B, int m, int n, int q, int c, int d);
void Cross(float* A, float* B, int n, int c, int d, float* C, int m, int e);
void Copy(float* A, int n, int m, float* B);
void Normalize3x3(float* A);

int main(){
	float A[3][3] = { { 1.02, 0, 0 }, { 0, 0.99, 0.95 }, { 0.02, -0.1, -0.03 } };
	//float A[9] = {  1, 2, 3, 4, 5, 6, 7, 8, 9 };
	NormalizeTay3x3((float*)A);
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cout << A[i][j] << "  ";
		}
	cout << endl;
	}
	
}



//Matrix normalise rows Routine for a nearly normal matrix
void NormalizeTay3x3(float* A)
{
	// A = input matrix (m x n)

	//an implementation which avoids square roots is below but in matlab code

	float xy_error=0;
	for (int i = 0; i <= 2; i++){ //cycle through rows
	xy_error = xy_error + A[3 * i] * A[1 + 3 * i];  //Matrix.dot(R2(:, 1), R2(:, 2));
	}
	float A2[3][3];
	for (int i = 0; i <= 2; i++){
		A2[i][0] = A[i*3] - 0.5*xy_error*A[i*3+1];
		A2[i][1] = A[i*3+1] - 0.5*xy_error*A[i*3];
	}

	Cross((float*)A2, (float*)A2, 3, 1, 2, (float*)A2, 3, 3);

	//R3(:,1) = R2(:,1) - 0.5*xy_error*R2(:,2);
	//R3(:,2) = R2(:,2) - 0.5*xy_error*R2(:,1);
	//R3(:,3) = cross(R3(:,1),R3(:,2));
	//make magnitudes equal to one, as the difference will be small can use
	//taylor expansion to avoid square root
	for (int j = 0; j <= 2; j++){ //cycle through columns
		float dot = Dot((float*)A2, (float*)A2, 3, 3, 3, (j + 1), (j + 1));
		for (int i = 0; i <= 2; i++){ //cycle through rows
			A2[i][j] = 0.5*(3 - A2[i][j] * dot);
		}
	}

Copy((float*)A2, 3, 3, (float*)A);
//R3(:,1) = 0.5*(3 - dot(R3(:,1),R3(:,1))) * R3(:,1);
//R3(:,2) = 0.5*(3 - dot(R3(:,2),R3(:,2))) * R3(:,2);
//R3(:,3) = 0.5*(3 - dot(R3(:,3),R3(:,3))) * R3(:,3);
}


//Vector dot Routine
// C = A.B
float Dot(float* A, float* B, int m, int n, int q, int c, int d)
{
	// A = input matrix/vector (m x n)
	// B = input vector (m x q)
	// c if column of A to use (1,2 or 3)
	// d if column of B to use (1,2 or 3)
	// C = output scalar = A.B
	c = c - 1;
	d = d - 1;
	float C = 0;
	for (int i = 0; i<m; i++) {
		C = C + A[i*n + c] * B[i*q + d];
	}
	return C;
}

//Vector cross Routine
// C = AxB
void Cross(float* A, float* B, int n, int c, int d, float* C, int m, int e)
{
	// A = input matrix/vector (3 x n)
	// B = input vector (3 x n)
	// c is column of A to use (1,2 or 3)
	// d is column of B to use (1,2 or 3)
	// C = output matrix (3 * m) = AxB
	// e is column of C to use (1,2 or 3)
	c = c - 1;
	d = d - 1;
	e = e - 1;
	C[0 + e] = A[n + c] * B[2 * n + d] - A[2 * n + c] * B[n + d];
	C[m + e] = -A[c] * B[2 * n + d] + A[2 * n + c] * B[d];
	C[2 * m + e] = A[c] * B[n + d] - A[n + c] * B[d];
}

void Copy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i<m; i++)
	for (j = 0; j<n; j++)
	{
		B[n*i + j] = A[n*i + j];
	}
}

//Matrix normalise rows Routine
void Normalize3x3(float* A)
{
	// A = input matrix (m x n)
	int i, j;
	float mag;
	for (i = 0; i<3; i++) { // cycle through columns
		mag = 0.0;
		for (j = 0; j<3; j++) { //cylce through rows
			mag = mag + pow(A[i + j * 3], 2.0); //remember matrix are in row major form, calculate magnitude of vector
		}
		mag = pow(mag, 0.5);
		for (j = 0; j<3; j++) { //cylce through rows
			A[i + j * 3] = A[i + j * 3] / mag; // now divide through by magnitude
		}
	}
}