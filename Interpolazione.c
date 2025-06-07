#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double interpolazioneSemplice(double** mat_1, int colonna, double alpha){
    if (alpha < mat_1[0][0]){
        alpha = mat_1[0][0];
        printf("Alpha inserito troppo piccolo\nalpha = %lf\n", alpha);
    }
    else if (alpha > mat_1[125][0]){
        alpha = mat_1[125][0];
        printf("Alpha inserito troppo grande\nalpha = %lf\n", alpha);
    }
    int i = 0;
    while (alpha > mat_1[i][0]) ++i;
    return mat_1[i][colonna] + ((mat_1[i+1][colonna] - mat_1[i][colonna]) / (mat_1[i+1][0] - mat_1[i][0])) * (alpha - mat_1[i][0]);
}

double interpolazioneTotale(double** mat_1, int colonna, double alpha){
    if (alpha < mat_1[0][0]){
        printf("Alpha inserito troppo piccolo\nalpha = %lf\n", mat_1[0][0]);
        return mat_1[0][colonna];
    }
    else if (alpha > mat_1[125][0]){
        printf("Alpha inserito troppo grande\nalpha = %lf\n", mat_1[125][0]);
        return mat_1[125][colonna];
    }
    int i = 0;
    //if (alpha > 19.77) printf("%lf\n", mat_1[i][0]); // Cambiare
    while (alpha > mat_1[i][0]) ++i;         // Cambiare
    if (alpha == mat_1[i][0]) return mat_1[i][colonna];
    return mat_1[i][colonna] + ((mat_1[i+1][colonna] - mat_1[i][colonna]) / (mat_1[i+1][0] - mat_1[i][0])) * (alpha - mat_1[i][0]);
}