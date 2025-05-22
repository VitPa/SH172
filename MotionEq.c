#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MotionEq.h"

void equation(double alpha, double rho_h, double *CI, double *body_axes, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **steady_state_coeff, double **control_force_der, double **control_moment_der, double **rotary_der){

    int i = 0, i_e = 0, i_a = 0, flag = 0, de, beta, err[126];

    // Trovare l'indice di alpha 
    while (flag == 0 && i_a < 126){
        if (aer_der_x[i_a][0] == alpha){
            flag = 1;
            break;
        }
        i_a++;
    }

    double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
    printf("-----\n%lf\n", cst);
    while (i < 126){

        double CZ_tot = aer_der_z[i][1] * aer_der_x[i][0] * (3.1415/180);
        double control = fabs(body_axes[0]*9.81*cos(aer_der_x[i][0]*(3.1415/180)) + cst * CZ_tot);

        printf("---------- ALPHA: %lf\n", aer_der_x[i][0]);
        printf("CZ_tot: %lf\n", CZ_tot);
        printf("cst * CZ_tot: %lf\n", cst * CZ_tot);
        printf("m*g*cos(a): %lf\n", body_axes[0]*9.81*cos(aer_der_x[i][0]*(3.1415/180)));

        if (control < 10.0){
            printf("Alpha di Trim trovato");
        } else {
            printf("ABS: %lf\n\n", control);
        }

        i++;
    }
        //double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
        //double CX_tot = aer_der_x[i_a][1] * alpha + control_force_der[i_a][1] * de; //cosa è de??
        //double CZ_tot = aer_der_z[i_a][1] * alpha + control_force_der[i_a][3] * de;
        //double CY_tot = aer_der_y[i_a][1] * beta + control_force_der[i_a][5] * de;

        //double X = cst * CX_tot;
        //double Z = cst * CZ_tot;
        //double Y = cst * CY_tot;

    

    
}