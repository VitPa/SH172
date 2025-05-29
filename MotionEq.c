#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MotionEq.h"
#include "Interpolazione_new.h"
#include "propeller.h"

#define g 9.80665
#define pi 3.14159265

void equation(double alpha, double Pmax_h, double rho_h, double *CI, double *body_axes, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **steady_state_coeff, double **control_force_der, double **control_moment_der, double **rotary_der, double **pitch_moment_der, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *fuel_mass) {

    int i = 0, i_e = 0, i_a = 0, flag = 0, de = 0;
    double alphaTrim = 0.0, deTrim = 0.0, RPMTrim = 0.0, PalTrim = 0.0;

    // Trovare l'indice di alpha 
    while (flag == 0 && i_a < 126){
        if (aer_der_x[i_a][0] == alpha){
            flag = 1;
            break;
        }
        i_a++;
    }

    // ****** TROVARE ALPHA DI TRIM *******

    double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
    int flag_1 = 0;
    double alpha_1 = 0;
    double score_min = 1000;
    
    for (int i = 0; i <= 2470; ++i) {
        alpha_1 = i * 0.01 - 5.0;
        double CZss = interpolazioneTotale(steady_state_coeff, 3, alpha_1);
        double CMss = interpolazioneTotale(steady_state_coeff, 5, alpha_1);
        double CMa = interpolazioneTotale(pitch_moment_der, 1, alpha_1);
        double CMde = interpolazioneTotale(control_moment_der, 3, alpha_1);
        double CZalpha = interpolazioneTotale(aer_der_z, 1, alpha_1);
        double CZde = control_force_der[0][3];
        
        for (double de_1 = -20.0; de_1 <= 20.0; de_1 += 0.01){          // Cambiare in while per quale condizione???
            double CZ_tot = CZss + CZalpha * alpha_1 * (pi/180) + CZde * de_1 * (pi/180);
            double control = fabs(body_axes[0]*g*cos(alpha_1*(pi/180) + CI[2]*(pi/180)) + cst * CZ_tot);
            double control2 = fabs(CMss + CMa * alpha_1 * (pi/180) + CMde * de_1 * (pi/180));

            if (control < 1.0 && control2 < 0.004){

                double score = sqrt(control/10*control/10 + control2/0.004*control2/0.004);
                if (score < score_min) {
                    score_min = score;
                    deTrim = de_1;
                    alphaTrim = alpha_1;
                }

                double de_ref = - (CMss + CMa * alpha_1 * (pi/180)) / CMde;
                printf("Valore di de stimato: %lf\n", de_ref*(180/pi));
                
                flag_1 = 1;
            }
        }
    }
    if (flag_1 == 0) {
        printf("Nessun alpha di Trim trovato!\n");
        //printf("Il valore più basso: %lf\t%lf\n", ver[0], ver[1]);
    } else {
        printf("\n*********************Alpha di Trim trovato**************************************\n\n");
        printf("---------- ALPHA: %lf\t\t DE_TRIM: %lf\n\n", alphaTrim, deTrim);
        /*printf("CZ_tot: %lf\n", CZ_tot);
        printf("cst * CZ_tot: %lf\n", cst * CZ_tot);
        printf("m*g*cos(a): %lf\n", body_axes[0]*g*cos(alpha_1*(pi/180)));
        printf("control: %lf\n", control);
        printf("control2: %lf\n\n", control2);
        printf("Valore interpolato di CZss: %lf\n", CZss);
        printf("Valore interpolato di CZalpha: %lf\n", CZalpha);*/
    }

    double thetaTrim = alphaTrim + CI[2];

    // Componente velocità TAS di Trim
    double uTrim = CI[0] * cos(alphaTrim*(pi/180));
    double wTrim = CI[0] * sin(alphaTrim*(pi/180));
    double hTrim = CI[1];

    // VETTORE DEGLI STATI (DINAMICA LONGITUDINALE)- Condizione di Trim
    double vett_stato[10] = {uTrim, 0, wTrim, 0, 0, 0, 0, thetaTrim, 0, hTrim};

    double CXss = interpolazioneTotale(steady_state_coeff, 1, alphaTrim);
    double CXalpha = interpolazioneTotale(aer_der_x, 1, alphaTrim);
    double CXde = interpolazioneTotale(control_force_der, 1, alphaTrim);

    // ******* TROVARE RPM di Trim *******
    double CX_tot = CXss + CXalpha * alphaTrim * (pi/180) + CXde * deTrim * (pi/180);
    double tTrim = body_axes[0]*g*sin(thetaTrim*(pi/180)) - 0.5*CX_tot*rho_h*CI[0]*CI[0]*body_axes[2];
    printf("\n\ntTrim: %lf\n", tTrim);

    int RPM = 1500;         // RPM minimi

    while (RPM <= 2700){
        double prop[3] = {0, 0, 0};
        double Pal = propel(RPM, rho_h, CI[0], geometry_propeller, propeller_profile, data_propeller, prop);

        if (fabs(tTrim - prop[0]) < 1.0){
            printf("\n*********************RPM di Trim trovato**************************************\n\n");
            printf("---------- RPM: %d\n\n", RPM);
            printf("Efficienza elica: %lf\n\n", prop[2]);
            RPMTrim = RPM;

            if (Pal>Pmax_h) {
                PalTrim = Pmax_h;
                printf("La potenza è stata limitata a quella massima\n");
            } else {
                PalTrim = Pal;
            }
            // printf("Pal: %lf\n", PalTrim);
            // printf("Pal massima: %lf\n", Pmax_h);
            break;
        }
        // printf("RPM: %d\t dT: %lf\n", RPM, fabs(tTrim - prop[0]));
        RPM += 1;
    }
}