#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "rk4.h"
#include "../Interpolation/Interpolation.h"
#include "../Pre_processing/Variables.h"
#include "../Processing/propeller.h"
#include "../Trim/MotionEq.h"


// Esempio di funzione per calcolare le derivate dello stato
// Adatta questa funzione copiando la logica di eulerEquation, ma scrivi le derivate in dstatedt
void compute_derivatives(const double *state, double *dstatedt, double t, int i) {
    // Variabili di stato
    double u     = state[0];
    double v     = state[1];
    double w     = state[2];
    double p     = state[3];
    double q     = state[4];
    double r     = state[5];
    double phi   = state[6];
    double theta = state[7];
    double psi   = state[8];
    double h     = state[9];
    double x_ned = state[10];
    double y_ned = state[11];

    // Comandi
    double da    = command[i][0] *(pi/180);
    double de    = command[i][1] *(pi/180);
    double dr    = command[i][2] *(pi/180);
    double manetta = engine[2] + (engine[3] - engine[2]) * (command[i][3]) / (100);

    // Calcolo spinta
    double prop[3] =  {0.0, 0.0, 0.0}, Pal = 0.0;
    propel(manetta, sqrt(u*u + v*v + w*w), prop, &Pal);
    double T = prop[0];

    // Calcolo della velocità totale
    double V = sqrt(u*u + v*v + w*w);

    // Definizione alpha e beta iniziali (t = 0):
    double alpha = atan2(w, u);
    double beta = asin(v / (V + 1e-8));

    // Calcolo velocità angolari adimensionali
    double q_ad = q*body_axes[3]/(2*V);
    double p_ad = p*body_axes[1]/(2*V);
    double r_ad = r*body_axes[1]/(2*V);
    double alpha_int = alpha*(180/pi);

    // Calcolo coefficienti aerodinamici
    double Cxss = interpolation(steady_state_coeff, 1, alpha_int);
    double Cxa = interpolation(aer_der_x, 1, alpha_int);
    double Cxde = interpolation(control_force_der, 1, alpha_int);
    double Cyb = interpolation(aer_der_y, 1, alpha_int);
    double Cyp = interpolation(aer_der_y, 3, alpha_int);
    double Cyr = interpolation(aer_der_y, 4, alpha_int);
    double Cydr = interpolation(control_force_der, 6, alpha_int);
    double Czss = interpolation(steady_state_coeff, 3, alpha_int);
    double Cza = interpolation(aer_der_z, 1, alpha_int);
    double Czq = interpolation(aer_der_z, 4, alpha_int);
    double Czde = interpolation(control_force_der, 3, alpha_int);
    double Clb = interpolation(rolling_moment_der, 1, alpha_int);
    double Clp = interpolation(rolling_moment_der, 3, alpha_int);
    double Clr = interpolation(rolling_moment_der, 4, alpha_int);
    double Clda = interpolation(control_moment_der, 1, alpha_int);
    double Cldr = interpolation(control_moment_der, 2, alpha_int); 
    double Cmss = interpolation(steady_state_coeff, 5, alpha_int);
    double Cma = interpolation(pitch_moment_der, 1, alpha_int);
    double Cmq = interpolation(pitch_moment_der, 4, alpha_int);
    double Cmde = interpolation(control_moment_der, 3, alpha_int);
    double Cnss = interpolation(steady_state_coeff, 6, alpha_int);
    double Cnb = interpolation(yawing_moment_der, 1, alpha_int);
    double Cnp = interpolation(yawing_moment_der, 3, alpha_int);
    double Cnr = interpolation(yawing_moment_der, 4, alpha_int);
    double Cnda = interpolation(control_moment_der, 5, alpha_int);
    double Cndr = interpolation(control_moment_der, 6, alpha_int);

    //Calcolo i momenti di Inerzia sui vari assi e la massa
    double Jx = body_axes[13], Jy = body_axes[14], Jz = body_axes[15], m = body_axes[0];

    // Calcolo Forze e Momenti (t = 0);
    double S = body_axes[2];
    double costante=0.5*rho_h*V*V*S;
    double X = costante*(Cxss+Cxa*alpha+Cxde*de);
    double Y = costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
    double Z = costante*(Czss+Cza*alpha+Czq*q_ad+Czde*de);
    double L = costante*body_axes[1]*(Clb*beta+Clp*p_ad+Clr*r_ad+Clda*da+Cldr*dr);
    double M = costante*body_axes[3]*(Cmss+Cma*alpha+Cmq*q_ad+Cmde*de);
    double N = costante*body_axes[1]*(Cnss+Cnb*beta+Cnp*p_ad+Cnr*r_ad+Cnda*da+Cndr*dr);

    // Equazioni del moto traslazionali
    dstatedt[0] = (r*v-q*w) - g*sin(theta) + X/m + T/m;
    dstatedt[1] = (p*w-r*u) + g*sin(phi)*cos(theta) + Y/m;
    dstatedt[2] = (q*u-p*v) + g*cos(phi)*cos(theta) + Z/m;

    // Equazioni del moto rotazionali
    dstatedt[3] = (-(Jz-Jy)*q*r)/Jx + L/Jx;
    dstatedt[4] = (-(Jx-Jz)*p*r)/Jy + M/Jy;
    dstatedt[5] = (-(Jy-Jx)*p*q)/Jz + N/Jz;

    // Cinematica (angoli di Eulero)
    dstatedt[6] = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    dstatedt[7] = q*cos(phi) - r*sin(phi);
    dstatedt[8] = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));

    // Quota e posizione
    dstatedt[9]  = -u*sin(theta) + v*cos(theta)*sin(phi) + w*cos(theta)*cos(phi);
    dstatedt[10] = u*cos(psi)*cos(theta) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    dstatedt[11] = u*sin(psi)*cos(theta) + v*(sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));

    // Calcolo consumo carburante (se serve)
    // (Non modificare variabili globali qui, aggiorna la massa fuori da questa funzione!)
}
