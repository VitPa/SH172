#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Interpolation.h"
#include "../Error_Warning/ErrorWarning.h"

double interpolation(double** mat_1, int colonna, double alpha){

    //if (alpha < mat_1[125][0]+1e-6 && alpha > mat_1[125][0]) alpha = 20.0;
    if (alpha < mat_1[0][0]-1e-6) MY_ERROR(600, alpha);
    else if (alpha > mat_1[125][0]+1e-6) MY_ERROR(601, alpha);

    int i = 0;
    while (alpha > mat_1[++i][0])

    if (alpha == mat_1[i][0]) return mat_1[i][colonna];
    return mat_1[i-1][colonna] + ((mat_1[i][colonna] - mat_1[i-1][colonna]) / (mat_1[i][0] - mat_1[i-1][0])) * (alpha - mat_1[i-1][0]);
}