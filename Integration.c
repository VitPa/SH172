#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Interpolazione.h"
#include "propeller.h"
#include "EstrazioneDati.h"
#include "Variables.h"
#include "MotionEq.h"


#define g 9.80665
#define pi 3.14159265

void eulerEquation(double dt, int i){
    FILE *agg = apriFile("DATI_AGGIUNTIVI.txt", "a");
 
    // Richiamo componenti vettore di stato state
    double u     = state[i][0];
    double v     = state[i][1];
    double w     = state[i][2];
    double p     = state[i][3];
    double q     = state[i][4];
    double r     = state[i][5];
    double phi   = state[i][6];
    double theta = state[i][7];
    double psi   = state[i][8];
    double h     = state[i][9];
    double x_ned = state[i][10];
    double y_ned = state[i][11];

    // Velostatetà totale iniziale (t = 0);
    double V = sqrt(u*u + v*v + w*w);

    // Richiamo componenti dei comandi
    double da, de, dr, manetta;
    if(liv_trim){
        double trim[3] = {0.0, 0.0, 0.0};
        double Ci[] = {V, h};
        equation(Ci, trim);
        da    = 0.0;
        de    = trim[1] *(pi/180);
        dr    = 0.0;
        manetta = trim[2];
    }else{
        da    = command[i][0] *(pi/180);
        de    = command[i][1] *(pi/180);
        dr    = command[i][2] *(pi/180);
        manetta = engine[2] + (engine[3] - engine[2]) * (command[i][3]) / (100);  // Mappatura manetta [0, 100] -> [RPMmin, RPMmax];
    }
    fprintf(agg, "%lf\t%lf\t%lf\t%lf\t%lf\n", i*dt, da, de, dr, manetta);
    fclose(agg);

    // Calcolo Spinta
    double prop[3] =  {0.0, 0.0, 0.0}, Pal = 0.0;

    propel(manetta, V, prop, &Pal);
    double T = prop[0];

    // Calcolo consumo di carburante   ---> Da cambiare dopo che ho il file Variables.c
    if(fuel_mass[0] == 1) body_axes[0] = massConsumption(engine[5], Pal, prop[2], body_axes[0], dt);

    double S = body_axes[2];
    double costante=0.5*rho_h*V*V*S;

    // Definizione alpha e beta iniziali (t = 0):
    double alpha = atan2(w,u);
    double beta = asin(v/V);

    // Calcolo velocità angolari adimensionali
    double q_ad = q*body_axes[3]/(2*V);
    double p_ad = p*body_axes[1]/(2*V);
    double r_ad = r*body_axes[1]/(2*V);
    double alpha_int = alpha*(180/pi);

    // Calcolo coefficienti aerodinamici
    double Cxss = interpolazioneTotale(steady_state_coeff, 1, alpha_int);
    double Cxa = interpolazioneTotale(aer_der_x, 1, alpha_int);
    double Cxde = interpolazioneTotale(control_force_der, 1, alpha_int);
    double Cyb = interpolazioneTotale(aer_der_y, 1, alpha_int);
    double Cyp = interpolazioneTotale(aer_der_y, 3, alpha_int);
    double Cyr = interpolazioneTotale(aer_der_y, 4, alpha_int);
    double Cydr = interpolazioneTotale(control_force_der, 6, alpha_int);
    double Czss = interpolazioneTotale(steady_state_coeff, 3, alpha_int);
    double Cza = interpolazioneTotale(aer_der_z, 1, alpha_int);
    double Czq = interpolazioneTotale(aer_der_z, 4, alpha_int);
    double Czde = interpolazioneTotale(control_force_der, 3, alpha_int);
    double Clb = interpolazioneTotale(rolling_moment_der, 1, alpha_int);
    double Clp = interpolazioneTotale(rolling_moment_der, 3, alpha_int);
    double Clr = interpolazioneTotale(rolling_moment_der, 4, alpha_int);
    double Clda = interpolazioneTotale(control_moment_der, 1, alpha_int);
    double Cldr = interpolazioneTotale(control_moment_der, 2, alpha_int); 
    double Cmss = interpolazioneTotale(steady_state_coeff, 5, alpha_int);
    double Cma = interpolazioneTotale(pitch_moment_der, 1, alpha_int);
    double Cmq = interpolazioneTotale(pitch_moment_der, 4, alpha_int);
    double Cmde = interpolazioneTotale(control_moment_der, 3, alpha_int);
    double Cnss = interpolazioneTotale(steady_state_coeff, 6, alpha_int);
    double Cnb = interpolazioneTotale(yawing_moment_der, 1, alpha_int);
    double Cnp = interpolazioneTotale(yawing_moment_der, 3, alpha_int);
    double Cnr = interpolazioneTotale(yawing_moment_der, 4, alpha_int);
    double Cnda = interpolazioneTotale(control_moment_der, 5, alpha_int);
    double Cndr = interpolazioneTotale(control_moment_der, 6, alpha_int);

    //Calcolo i momenti di Inerzia sui vari assi e la massa
    double Jx = body_axes[13], Jy = body_axes[14], Jz = body_axes[15], m = body_axes[0];

    // Calcolo Forze e Momenti (t = 0);
    double X = costante*(Cxss+Cxa*alpha+Cxde*de);
    double Y = costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
    double Z = costante*(Czss+Cza*alpha+Czq*q_ad+Czde*de);
    double L = costante*body_axes[1]*(Clb*beta+Clp*p_ad+Clr*r_ad+Clda*da+Cldr*dr);
    double M = costante*body_axes[3]*(Cmss+Cma*alpha+Cmq*q_ad+Cmde*de);
    double N = costante*body_axes[1]*(Cnss+Cnb*beta+Cnp*p_ad+Cnr*r_ad+Cnda*da+Cndr*dr);

    // Incrementi tempo t = 0[s]
    double du_     = (r*v-q*w) - g*sin(theta) + X/m + T/m;
    double dv_     = (p*w-r*u) + g*sin(phi)*cos(theta) + Y/m;
    double dw_     = (q*u-p*v) + g*cos(phi)*cos(theta) + Z/m;
    double dp_     = (-(Jz-Jy)*q*r)/Jx + L/Jx;
    double dq_     = (-(Jx-Jz)*p*r)/Jy + M/Jy;
    double dr_     = (-(Jy-Jx)*p*q)/Jz + N/Jz;
    double dphi_   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    double dtheta_ = q*cos(phi) - r*sin(phi);
    double dpsi_   = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));
    double dh_     = -u*sin(theta) + v*cos(theta)*sin(phi) + w*cos(theta)*cos(phi);
    double dx_ned = u*cos(psi)*cos(theta) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    double dy_ned = u*sin(psi)*cos(theta) + v*(sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
    
    // Vettore di stato dopo condizione di trim.
    state[i+1][0]  = u + dt*du_;
    state[i+1][1]  = v + dt*dv_;
    state[i+1][2]  = w + dt*dw_;
    state[i+1][3]  = p + dt*dp_;
    state[i+1][4]  = q + dt*dq_;
    state[i+1][5]  = r + dt*dr_;
    state[i+1][6]  = phi + dt*dphi_;
    state[i+1][7]  = theta + dt*dtheta_;
    state[i+1][8]  = psi + dt*dpsi_;
    state[i+1][9]  = h + dt*dh_;
    state[i+1][10] = x_ned + dt*dx_ned;
    state[i+1][11] = y_ned + dt*dy_ned;
}

void progressBar(double Ts, double deltaT_fs){
    if(fmod(Ts, (deltaT_fs/40.0)) < 0.01){
        int progress = (int)(Ts / (deltaT_fs / 40.0));
        if (Ts > deltaT_fs) progress = 40;
        printf("\r[");
        for(int k = 0; k<40; ++k){
            if(k <= progress){
                printf("*");
            } else {
                printf("-");
            }
        }
        printf("] Simulating");
    }
}