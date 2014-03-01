    /*
     *  MatrixMath.h Library for Matrix Math
     *
     *  Created by Charlie Matlack on 12/18/10.
     *  Modified from code by RobH45345 on Arduino Forums, taken from unknown source.
     */

    #ifndef MatrixMath_h
    #define MatrixMath_h

    #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
    #else
    #include "WProgram.h"
    #endif

    //these function use pointers, the identifier of an array is equivalent to the address of its first element,
    //which is the same as its pointer. Therefore an array can be accessed by it's pointer (add 1 to the
    //pointer to access the next element in the array
    //Matrices should be stored in row-major arrays, which is fairly standard. The user must keep track of
    //array dimensions and send them to the functions; mistakes on dimensions will not be caught by the library.

    class MatrixMath
    {
    public:
        //MatrixMath();
        void Print(float* A, int m, int n, String label);
        void Copy(float* A, int n, int m, float* B);
        void Multiply(float* A, float* B, int m, int p, int n, float* C);
        void Add(float* A, float* B, int m, int n, float* C);
        void Subtract(float* A, float* B, int m, int n, float* C);
        void Transpose(float* A, int m, int n, float* C);
        void Scale(float* A, int m, int n, float k);
        int Invert(float* A, int n);
		float Dot(float* A, float* B, int m, int n, int q, int c, int d);
		void Cross(float* A, float* B, int n, int c, int d, float* C, int m, int e);
        void Normalize3x3(float* A);
		void NormalizeTay3x3(float* A);
    };

    extern MatrixMath Matrix;
    #endif
