#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "MotionEq.h"
#include "routh.h"
#include "../Error_Warning/ErrorWarning.h"
#include "../Interpolation/Interpolation.h"
#include "../Pre_processing/Variables.h"
#include "../Processing/Propeller.h"
#include "../Processing/Integration.h"

#define g 9.80665
#define pi 3.14159265

void trimEquation(double *CI, double *trim) {

    // ****** TROVARE ALPHA DI TRIM *******

    double cst = 0.5 * rho_h * CI[0] * CI[0] * body_axes[2];
    int flag_1 = 0;
    double score_min = 1000;
    double CZtrim, CMtrim;
    double res1 = 1, res2 = 0.0001;
    
    for (double alpha_1 = -5.0; alpha_1 <= 20.0; alpha_1 += 0.001) {
        double CZss = interpolation(steady_state_coeff, 3, alpha_1);
        double CMss = interpolation(steady_state_coeff, 5, alpha_1);
        double CMalpha = interpolation(pitch_moment_der, 1, alpha_1);
        double CMde = interpolation(control_moment_der, 3, alpha_1);
        double CZalpha = interpolation(aer_der_z, 1, alpha_1);
        double CZde = control_force_der[0][3];
        
        for (double de_1 = -20.0; de_1 <= 20.0; de_1 += 0.001){
            double CZ_tot = CZss + CZalpha * alpha_1 * (pi/180) + CZde * de_1 * (pi/180);
            double control = fabs(body_axes[0]*g*cos(alpha_1*(pi/180) + CI[2]*(pi/180)) + cst * CZ_tot);
            double control2 = fabs(CMss + CMalpha * alpha_1*(pi/180) + CMde * de_1 * (pi/180));

            if (control < res1 && control2 < res2){

                double score = sqrt(pow(control/res1,2) + pow(control2/res2,2));
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
        progressBar(alpha_1, 20.0, "Calcolando le condizioni di Trim!");
    }
    printf("\r\n");
    if (flag_1 != 0) {
        printf("\n*********************Alpha di Trim trovato**************************************\n\n");
        printf("---------- ALPHA: %lf\t\t DE_TRIM: %lf\n\n", trim[0], trim[1]);
    } else MY_ERROR(400);

    double thetaTrim = (trim[0] + CI[2])*(pi/180);  

    // Componente velocità TAS di Trim
    double uTrim = CI[0] * cos(trim[0]*(pi/180));
    double wTrim = CI[0] * sin(trim[0]*(pi/180));
    double hTrim = CI[1];
    
    state = calloc(12, sizeof(double));
    if (state == NULL) MY_ERROR(901, "state");
    state[0] = uTrim;
    state[2] = wTrim;
    state[7] = thetaTrim;
    state[9] = hTrim;
    

    double CXss = interpolation(steady_state_coeff, 1, trim[0]);
    double CXalpha = interpolation(aer_der_x, 1, trim[0]);
    double CXde = interpolation(control_force_der, 1, trim[0]);

    // ******* TROVARE RPM di Trim *******
    double CX_tot = CXss + CXalpha * trim[0] * (pi/180) + CXde * trim[1] * (pi/180);
    double tTrim = body_axes[0]*g*sin(thetaTrim) - 0.5*CX_tot*rho_h*CI[0]*CI[0]*body_axes[2];

    int RPM = RPMmin;
    double prop_hold[3];
    while (RPM <= RPMmax){
        double Pal;
        double prop[3] = {0, 0, 0};
        propel(RPM, CI[0], prop, &Pal);

        if (tTrim - prop[0] < 0.0){            
            memcpy(prop, prop_hold, sizeof(double)*3);
            printf("\n*********************RPM di Trim trovato**************************************\n\n");
            printf("---------- RPM: %d\n\n", RPM-1);
            printf("Efficienza elica: %lf\n\n", prop[2]);
            trim[2] = RPM-1;
            break;
        }
        memcpy(prop_hold, prop, sizeof(double)*3);
        RPM += 1;
    }
    if(RPM > RPMmax) MY_ERROR(401);

    // Calcolo la stabilità dell'aeromobile
    int a = routh(pitch_moment_der[0][4], trim[0], CI[0], CXalpha, CZtrim, CMtrim, pitch_moment_der[0][2]);
}