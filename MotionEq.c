#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ErrorWarning.h"
#include "Interpolazione.h"
#include "propeller.h"
#include "routh.h"

#define g 9.80665
#define pi 3.14159265

void equation(double *engine, double Pmax_h, double rho_h, double *CI, double ***vett_stato, double *body_axes, double **aer_der_x, double **aer_der_z, double **steady_state_coeff, double **control_force_der, double **control_moment_der, double **pitch_moment_der, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *trim) {
    
    static int stampa = 1;

    // ****** TROVARE ALPHA DI TRIM *******

    double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
    int flag_1 = 0;
    double score_min = 1000;
    double CZtrim, CMtrim;
    
    for (double alpha_1 = -5.0; alpha_1 <= 20.0; alpha_1 += 0.01) {  // cambia la i con alpha_1, tanto la i non viene usata mai
        double CZss = interpolazioneTotale(steady_state_coeff, 3, alpha_1);
        double CMss = interpolazioneTotale(steady_state_coeff, 5, alpha_1);
        double CMalpha = interpolazioneTotale(pitch_moment_der, 1, alpha_1);
        double CMde = interpolazioneTotale(control_moment_der, 3, alpha_1);
        double CZalpha = interpolazioneTotale(aer_der_z, 1, alpha_1);
        double CZde = control_force_der[0][3];
        
        for (double de_1 = -20.0; de_1 <= 20.0; de_1 += 0.01){
            double CZ_tot = CZss + CZalpha * alpha_1 * (pi/180) + CZde * de_1 * (pi/180);
            double control = fabs(body_axes[0]*g*cos(alpha_1*(pi/180) + CI[2]*(pi/180)) + cst * CZ_tot);
            double control2 = fabs(CMss + CMalpha * alpha_1*(pi/180) + CMde * de_1 * (pi/180));

            if (control < 1.0 && control2 < 0.004){

                double score = sqrt(pow(control/10,2) + pow(control2/0.004,2));
                if (score < score_min) {
                    score_min = score;
                    trim[1] = de_1;
                    trim[0] = alpha_1;
                    CZtrim = CZalpha;
                    CMtrim = CMalpha;
                }
                flag_1 = 1;
            }
        }
    }
    if (flag_1 != 0) {
        if(stampa){
            printf("\n*********************Alpha di Trim trovato**************************************\n\n");
            printf("---------- ALPHA: %lf\t\t DE_TRIM: %lf\n\n", trim[0], trim[1]);
        }
    } else ERROR(400);

    double thetaTrim = (trim[0] + CI[2])*(pi/180);  

    // Componente velocità TAS di Trim
    double uTrim = CI[0] * cos(trim[0]*(pi/180));
    double wTrim = CI[0] * sin(trim[0]*(pi/180));
    double hTrim = CI[1];

    // Creo la prima righa della matrice vett_stato (dinamica)
    *vett_stato = malloc(sizeof(double*));
    if (*vett_stato == NULL) ERROR(902, "state");

    (*vett_stato)[0] = calloc(12, sizeof(double));
    if ((*vett_stato)[0] == NULL) ERROR(901, "state");

    // Riempio la prima riga con i valori di Trim
    for (int i = 0; i < 12; ++i){
        switch (i) {
            case 0:
                (*vett_stato)[0][i] = uTrim;
                break;
            case 2:
                (*vett_stato)[0][i] = wTrim;
                break;
            case 7:
                (*vett_stato)[0][i] = thetaTrim;
                break;
            case 9:
                (*vett_stato)[0][i] = hTrim;
                break;
            default:
                (*vett_stato)[0][i] = 0;
                break;
        }
    }

    double CXss = interpolazioneTotale(steady_state_coeff, 1, trim[0]);
    double CXalpha = interpolazioneTotale(aer_der_x, 1, trim[0]);
    double CXde = interpolazioneTotale(control_force_der, 1, trim[0]);

    // ******* TROVARE RPM di Trim *******
    double CX_tot = CXss + CXalpha * trim[0] * (pi/180) + CXde * trim[1] * (pi/180);
    double tTrim = body_axes[0]*g*sin(thetaTrim) - 0.5*CX_tot*rho_h*CI[0]*CI[0]*body_axes[2];

    int RPM = (int) engine[2];
    int RPM_max = (int) engine[3];
    while (RPM <= RPM_max){
        double Pal;
        double prop[3] = {0, 0, 0};
        propel(RPM, Pmax_h, rho_h, CI[0], geometry_propeller, propeller_profile, data_propeller, prop, &Pal);

        if (fabs(tTrim - prop[0]) < 1.0){
            if(stampa){
                printf("\n*********************RPM di Trim trovato**************************************\n\n");
                printf("---------- RPM: %d\n\n", RPM);
                printf("Efficienza elica: %lf\n\n", prop[2]);
                stampa = 0;
            }
            trim[2] = RPM;
            break;
        }
        RPM += 1;
    }
    if(RPM > RPM_max) ERROR(401);

    // Calcolo la stabilità dell'aeromobile
    int a = routh(pitch_moment_der[0][4], body_axes, rho_h, trim[0], CI[0], CXalpha, CZtrim, CMtrim, pitch_moment_der[0][2]);
}