#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Interpolazione.h"

void stampaInterpolazione(double *Int_ssc, double *Int_adx, double *Int_ady, double *Int_adz, double *Int_rmd, 
    double *Int_pmd, double *Int_ymd, double *Int_cfd, double *Int_cmd, double *Int_rd){
    for(int j = 0; j < 7; j++){
        if(j == 6){
            printf("Int_adx[%d] = %lf\n", j, Int_adx[j]);
            printf("------------------------------\n");
            printf("Int_adz[%d] = %lf\n", j, Int_adz[j]);
            printf("------------------------------\n");
            printf("Int_pmd[%d] = %lf\n", j, Int_pmd[j]);
            printf("------------------------------\n");
        }else{
            printf("Int_ssc[%d] = %lf\n", j, Int_ssc[j]);
            printf("------------------------------\n");
            printf("Int_adx[%d] = %lf\n", j, Int_adx[j]);
            printf("------------------------------\n");
            printf("Int_ady[%d] = %lf\n", j, Int_ady[j]);
            printf("------------------------------\n");
            printf("Int_adz[%d] = %lf\n", j, Int_adz[j]);
            printf("------------------------------\n");
            printf("Int_rmd[%d] = %lf\n", j, Int_rmd[j]);
            printf("------------------------------\n");
            printf("Int_pmd[%d] = %lf\n", j, Int_pmd[j]);
            printf("------------------------------\n");
            printf("Int_ymd[%d] = %lf\n", j, Int_ymd[j]);
            printf("------------------------------\n");
            printf("Int_cfd[%d] = %lf\n", j, Int_cfd[j]);
            printf("------------------------------\n");
            printf("Int_cmd[%d] = %lf\n", j, Int_cmd[j]);
            printf("------------------------------\n");
            printf("Int_rd[%d] = %lf\n", j, Int_rd[j]);
        }
    }
}

void InterpolazioneCoeff(int stampa, double **body_axes, double *CI, double *temp_h, double mio_alpha,
    double *Int_ssc, double *Int_adx, double *Int_ady, double *Int_adz, double *Int_rmd, 
    double *Int_pmd, double *Int_ymd, double *Int_cfd, double *Int_cmd, double *Int_rd,
    double ***steady_state_coeff, double ***aer_der_x, double ***aer_der_y, 
    double ***aer_der_z, double ***rolling_moment_der, double ***pitch_moment_der, double ***yawing_moment_der, 
    double ***control_force_der, double ***control_moment_der, double ***rotary_der){
    
    //TODO Validare la massa di carburante dinamica
    
    double R = 287.05, a, b, a1, a2;
    int trovato = 0, i = 0;
    
    double M = CI[0]/(sqrt(1.4 * R * *temp_h));

    if (M > (*body_axes)[4]){
        printf("La velocità è maggiore di quella di drag rise!");
        exit(1);
    }
    
    if (mio_alpha > 20.0){
        mio_alpha = 20;
        printf("Il valore di aplha è superiore del valore massimo consentito. \nIl valore impostato è: 20°\n");
    } else if(mio_alpha < -5.0){
        mio_alpha = -5;
        printf("Il valore di aplha è inferiore del valore minimo consentito. \nIl valore impostato è: -5°\n");
    }

    while (trovato == 0){

        if (mio_alpha == (*steady_state_coeff)[i][0]){
            
            for(int j = 1; j < 8; j++){
                if (j == 7){
                    Int_adx[j-1] = (*aer_der_x)[i][j];
                    Int_adz[j-1] = (*aer_der_z)[i][j];
                    Int_pmd[j-1] = (*pitch_moment_der)[i][j];
                } else {
                    Int_ssc[j-1] = (*steady_state_coeff)[i][j];
                    Int_ady[j-1] = (*aer_der_y)[i][j];
                    Int_rmd[j-1] = (*rolling_moment_der)[i][j];
                    Int_ymd[j-1] = (*yawing_moment_der)[i][j];
                    Int_cfd[j-1] = (*control_force_der)[i][j];
                    Int_cmd[j-1] = (*control_moment_der)[i][j];
                    Int_rd[j-1] = (*rotary_der)[i][j];
                    Int_adx[j-1] = (*aer_der_x)[i][j];
                    Int_adz[j-1] = (*aer_der_z)[i][j];
                    Int_pmd[j-1] = (*pitch_moment_der)[i][j];
                }
                
            }

            if (stampa == 1){
                stampaInterpolazione(Int_ssc, Int_adx, Int_ady, Int_adz, Int_rmd, Int_pmd, Int_ymd, Int_cfd, Int_cmd, Int_rd);
            }

            trovato = 1;

        } else if (mio_alpha < (*steady_state_coeff)[i][0]){
            a2 = (*steady_state_coeff)[i][0];
            a1 = (*steady_state_coeff)[i-1][0];

            for(int j = 0; j < 7; j++){
                if (j == 6) {
                    a = ((*aer_der_x)[i-1][j+1] - (*aer_der_x)[i][j+1])/(a1 - a2);
                    b = ((*aer_der_x)[i][j+1] - a * a2);
                    Int_adx[j] = a * mio_alpha + b;

                    a = ((*aer_der_z)[i-1][j+1] - (*aer_der_z)[i][j+1])/(a1 - a2);
                    b = ((*aer_der_z)[i][j+1] - a * a2);
                    Int_adz[j] = a * mio_alpha + b;
                    
                    a = ((*pitch_moment_der)[i-1][j+1] - (*pitch_moment_der)[i][j+1])/(a1 - a2);
                    b = ((*pitch_moment_der)[i][j+1] - a * a2);
                    Int_pmd[j] = a * mio_alpha + b;
                } else {
                    a = ((*steady_state_coeff)[i-1][j+1] - (*steady_state_coeff)[i][j+1])/(a1 - a2);
                    b = ((*steady_state_coeff)[i][j+1] - a * a2);
                    Int_ssc[j] = a * mio_alpha + b;

                    a = ((*aer_der_x)[i-1][j+1] - (*aer_der_x)[i][j+1])/(a1 - a2);
                    b = ((*aer_der_x)[i][j+1] - a * a2);
                    Int_adx[j] = a * mio_alpha + b;

                    a = ((*aer_der_y)[i-1][j+1] - (*aer_der_y)[i][j+1])/(a1 - a2);
                    b = ((*aer_der_y)[i][j+1] - a * a2);
                    Int_ady[j] = a * mio_alpha + b;

                    a = ((*aer_der_z)[i-1][j+1] - (*aer_der_z)[i][j+1])/(a1 - a2);
                    b = ((*aer_der_z)[i][j+1] - a * a2);
                    Int_adz[j] = a * mio_alpha + b;

                    a = ((*rolling_moment_der)[i-1][j+1] - (*rolling_moment_der)[i][j+1])/(a1 - a2);
                    b = (*rolling_moment_der[i][j+1] - a * a2);
                    Int_rmd[j] = a * mio_alpha + b;

                    a = ((*pitch_moment_der)[i-1][j+1] - (*pitch_moment_der)[i][j+1])/(a1 - a2);
                    b = ((*pitch_moment_der)[i][j+1] - a * a2);
                    Int_pmd[j] = a * mio_alpha + b;

                    a = ((*yawing_moment_der)[i-1][j+1] - (*yawing_moment_der)[i][j+1])/(a1 - a2);
                    b = ((*yawing_moment_der)[i][j+1] - a * a2);
                    Int_ymd[j] = a * mio_alpha + b;

                    a = ((*control_force_der)[i-1][j+1] - (*control_force_der)[i][j+1])/(a1 - a2);
                    b = ((*control_force_der)[i][j+1] - a * a2);
                    Int_cfd[j] = a * mio_alpha + b;

                    a = ((*control_moment_der)[i-1][j+1] - (*control_moment_der)[i][j+1])/(a1 - a2);
                    b = ((*control_moment_der)[i][j+1] - a * a2);
                    Int_cmd[j] = a * mio_alpha + b;

                    a = ((*rotary_der)[i-1][j+1] - (*rotary_der)[i][j+1])/(a1 - a2);
                    b = ((*rotary_der)[i][j+1] - a * a2);
                    Int_rd[j] = a * mio_alpha + b;
                }
            }

            if (stampa == 1){
                stampaInterpolazione(Int_ssc, Int_adx, Int_ady, Int_adz, Int_rmd, Int_pmd, Int_ymd, Int_cfd, Int_cmd, Int_rd);
            }
            
            trovato = 1;
        }
        
        i++;
    }
}
