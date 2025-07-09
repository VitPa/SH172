#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ErrorWarning.h"

double interpolazioneTotale(double** mat_1, int colonna, double alpha){
    char buf[4];
    sprintf(buf, "%.2lf", alpha);
    if (alpha < mat_1[125][0]+1e-6 && alpha > mat_1[125][0]) alpha = 20.0;
    else if (alpha < mat_1[0][0]-1e-6) Error(600, buf);
    else if (alpha > mat_1[125][0]+1e-6) Error(601, buf);

    int i = 0;
    while (alpha > mat_1[++i][0])

    if (alpha == mat_1[i][0]) return mat_1[i][colonna];
    return mat_1[i-1][colonna] + ((mat_1[i][colonna] - mat_1[i-1][colonna]) / (mat_1[i][0] - mat_1[i-1][0])) * (alpha - mat_1[i-1][0]);
}