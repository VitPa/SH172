#include <math.h>
#include "MotionEq.h"

void equation(double alpha, double rho_h, double *CI, double *body_axes, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **steady_state_coeff, double **control_force_der, double **control_moment_der, double **rotary_der){

    int i = 0, i_a = 0, flag = 0, de, beta, err[126];

    // Trovare l'indice di alpha 
    while (flag == 0 && i_a < 126){
        if (aer_der_x[i_a][0] == alpha){
            flag = 1;
            break;
        }
        i_a++;
    }

    while (i_a < 126){
        double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
        double CZ_tot = aer_der_z[i_a][1] * alpha + control_force_der[i_a][3] * de;

        if (abs(1046*9.81*cos(aer_der_x[i_a][0]) + 0.5*1.1116*16.3*CI[0]*CI[0]*CZ_tot) < 10){
            printf("Alpha di trim trovato");
        } else {
            err[i++] = abs(1046*9.81*cos(aer_der_x[i_a][0]) + 0.5*1.1116*16.3*CI[0]*CI[0]*CZ_tot);
        }
    }

    printf("il minimo è: %lf", min(err));
        //double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
        //double CX_tot = aer_der_x[i_a][1] * alpha + control_force_der[i_a][1] * de; //cosa è de??
        //double CZ_tot = aer_der_z[i_a][1] * alpha + control_force_der[i_a][3] * de;
        //double CY_tot = aer_der_y[i_a][1] * beta + control_force_der[i_a][5] * de;

        //double X = cst * CX_tot;
        //double Z = cst * CZ_tot;
        //double Y = cst * CY_tot;

    

    
}