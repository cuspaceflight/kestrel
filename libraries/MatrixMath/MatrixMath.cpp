/*
 *  MatrixMath.cpp Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, taken from unknown source.
 *
 */

#include "MatrixMath.h"

#define NR_END 1

MatrixMath Matrix;          // Pre-instantiate

// Matrix Printing Routine
// Uses tabs to separate numbers under assumption printed float width won't cause problems
void MatrixMath::Print(float* A, int m, int n, String label){
    // A = input matrix (m x n)
    int i,j;
    Serial.println();
    Serial.println(label);
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
            Serial.print(A[n*i+j]);
            Serial.print("\t");
        }
        Serial.println();
    }
}

void MatrixMath::Copy(float* A, int n, int m, float* B)
{
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
            B[n*i+j] = A[n*i+j];
        }
}

//Matrix Multiplication Routine
// C = A*B
void MatrixMath::Multiply(float* A, float* B, int m, int p, int n, float* C)
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
            C[n*i+j]=0;
            for (k=0;k<p;k++)
                C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}


//Matrix Addition Routine
void MatrixMath::Add(float* A, float* B, int m, int n, float* C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}


//Matrix Subtraction Routine
void MatrixMath::Subtract(float* A, float* B, int m, int n, float* C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}


//Matrix Transpose Routine
void MatrixMath::Transpose(float* A, int m, int n, float* C)
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[m*j+i]=A[n*i+j];
}

void MatrixMath::Scale(float* A, int m, int n, float k)
{
    for (int i=0; i<m; i++)
        for (int j=0; j<n; j++)
            A[n*i+j] = A[n*i+j]*k;
}


//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//   NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int MatrixMath::Invert(float* A, int n)
{
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow;     // keeps track of current pivot row
    int k,i,j;      // k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    float tmp;      // used for finding max value and making column swaps

    for (k = 0; k < n; k++)
    {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < n; i++)
        {
            if (abs(A[i*n+k]) >= tmp)   // 'Avoid using other functions inside abs()?'
            {
                tmp = abs(A[i*n+k]);
                pivrow = i;
            }
        }

        // check for singular matrix
        if (A[pivrow*n+k] == 0.0f)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }

        // Execute pivot (row swap) if needed
        if (pivrow != k)
        {
            // swap row k with pivrow
            for (j = 0; j < n; j++)
            {
                tmp = A[k*n+j];
                A[k*n+j] = A[pivrow*n+j];
                A[pivrow*n+j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)

        tmp = 1.0f/A[k*n+k];    // invert pivot element
        A[k*n+k] = 1.0f;        // This element of input matrix becomes result matrix

        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++)
        {
            A[k*n+j] = A[k*n+j]*tmp;
        }

        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                tmp = A[i*n+k];
                A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++)
                {
                    A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
                }
            }
        }
    }

    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                tmp = A[i*n+k];
                A[i*n+k] = A[i*n+pivrows[k]];
                A[i*n+pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}

//Vector dot Routine
// C = A.B
float MatrixMath::Dot(float* A, float* B, int m, int n, int q, int c, int d)
{
    // A = input matrix/vector (m x n)
    // B = input vector (m x q)
    // c if column of A to use (1,2 or 3)
	// d if column of B to use (1,2 or 3)
    // C = output scalar = A.B
	c = c - 1;
	d = d - 1;
    float C = 0;
    for (int i=0; i<m;i++) {
        C = C + A[i*n + c] * B[i*q+d];
    }
    return C;
}

//Vector cross Routine
// C = AxB
void MatrixMath::Cross(float* A, float* B, int n, int c, int d, float* C, int m, int e)
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
	C[0+e] = A[n+c] * B[2*n+d] - A[2*n+c] * B[n+d];
	C[m+e] =-A[c] * B[2*n+d] + A[2*n+c] * B[d];
	C[2*m+e] = A[c] * B[n+d] - A[n+c] * B[d];
}

//Matrix normalise rows Routine
void MatrixMath::Normalize3x3(float* A)
{
    // A = input matrix (m x n)
    int i, j;
    float mag;
    for (i=0;i<3;i++) { // cycle through columns
        mag = 0.0;
        for(j=0;j<3;j++) { //cylce through rows
            mag = mag + pow( A[i+j*3], 2.0); //remember matrix are in row major form, calculate magnitude of vector
        }
        mag = pow(mag, 0.5);
        for(j=0;j<3;j++) { //cylce through rows
            A[i+j*3] = A[i+j*3] / mag; // now divide through by magnitude
        }
    }
}


//Matrix normalise rows Routine for a nearly normal matrix
void MatrixMath::NormalizeTay3x3(float* A)
{ //WARNING this function doesn't work yet
	// A = input matrix (m x n)

	//an implementation which avoids square roots is below but in matlab code

	float xy_error = 0;
	for (int i = 0; i <= 2; i++){ //cycle through rows
		xy_error = xy_error + A[3 * i] * A[1 + 3 * i];  //Matrix.dot(R2(:, 1), R2(:, 2));
	}
	float A2[3][3];
	for (int i = 0; i <= 2; i++){
		A2[i][0] = A[i * 3] - 0.5*xy_error*A[i * 3 + 1];
		A2[i][1] = A[i * 3 + 1] - 0.5*xy_error*A[i * 3];
	}

	Matrix.Cross((float*)A2, (float*)A2, 3, 1, 2, (float*)A2, 3, 3);

	//R3(:,1) = R2(:,1) - 0.5*xy_error*R2(:,2);
	//R3(:,2) = R2(:,2) - 0.5*xy_error*R2(:,1);
	//R3(:,3) = cross(R3(:,1),R3(:,2));
	//make magnitudes equal to one, as the difference will be small can use
	//taylor expansion to avoid square root
	for (int j = 0; j <= 2; j++){ //cycle through columns
		float dot = Matrix.Dot((float*)A2, (float*)A2, 3, 3, 3, (j + 1), (j + 1));
		for (int i = 0; i <= 2; i++){ //cycle through rows
			A2[i][j] = 0.5*(3 - dot)*A2[i][j];
		}
	
	}
	Matrix.Copy((float*)A2, 3, 3, (float*)A);
	//R3(:,1) = 0.5*(3 - dot(R3(:,1),R3(:,1))) * R3(:,1);
	//R3(:,2) = 0.5*(3 - dot(R3(:,2),R3(:,2))) * R3(:,2);
	//R3(:,3) = 0.5*(3 - dot(R3(:,3),R3(:,3))) * R3(:,3);
}
