#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Interpolazione.h"
#include "propeller.h"


#define g 9.80665
#define pi 3.14159265

void eulerEquation(double dt, int i, double **state, /*double **command*/ double command[][4], double Pmax_h, double rho, double RPM,double *engine, double *body_axes, double **steady_state_coefficients, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double *geometry_propeller, double *propeller_profile, double **data_propeller){
    //printf("rho: %lf\n", rho);
    
    // Inizializzo le variabili
    double u,v,w,p,q,r,phi,theta,psi,h,x_ned,y_ned;
    double da, de, dr, manetta;
    double S, V, costante;
    double alpha, beta;
    double q_ad, r_ad, p_ad;
    double T, prop[3] =  {0.0, 0.0, 0.0}, Pal = 0.0;
    double X, Y, Z, L, M, N;
    double du, dv, dw, dp, dq, dphi, dtheta, dpsi, dh, dx_ned, dy_ned;
    // Richiamo componenti vettore di stato state:
    u     = state[i][0];
    v     = state[i][1];
    w     = state[i][2];
    p     = state[i][3];
    q     = state[i][4];
    r     = state[i][5];
    phi   = state[i][6];
    theta = state[i][7];
    psi   = state[i][8];
    h     = state[i][9];
    x_ned = state[i][10];
    y_ned = state[i][11];

    //Richiamo componenti dei comandi
    da    = command[i][0];
    de    = command[i][1] *(pi/180);
    dr    = command[i][2];
    manetta = command[i][3];

    // Velostatetà totale iniziale (t = 0);
    V=sqrt(u*u + v*v + w*w);

    // Calcolo Spinta
    propel(RPM, Pmax_h, rho, V, geometry_propeller, propeller_profile, data_propeller, prop, &Pal);
    T = prop[0];
    /*printf("Spinta: %lf\n", T);
    printf("Pal: %lf\n", Pal);*/

    // Calcolo consumo di carburante
    body_axes[0] = massConsumption(engine[5], Pal, prop[2], body_axes[0], dt);
    //printf("Massa: %lf\n", body_axes[0]);
    // if (body_axes[0] < ??)                   CAPIRE COME CALCOLARE LA CONDIZIONE DI ERRORE PER IL CARBURANTE

    S = body_axes[2];
    costante=0.5*rho*V*V*S;

    // Definizione alpha e beta iniziali (t = 0):
    alpha = atan2(w,u);
    beta = asin(v/V);

    //printf("alpha: %lf\n", alpha*(180/pi));

    //Calcolo velocità angolari adimensionali
    q_ad = q*body_axes[3]/(2*V);
    p_ad = p*body_axes[1]/(2*V);
    r_ad = r*body_axes[1]/(2*V);
    double alpha_int = alpha*(180/pi);

    // Calcolo coefficienti aerodinamici
    double Cxss = interpolazioneTotale(steady_state_coefficients, 1, alpha_int);
    double Cxa = interpolazioneTotale(aer_der_x, 1, alpha_int);
    double Cxde = interpolazioneTotale(control_force_der, 1, alpha_int);
    double Cyb = interpolazioneTotale(aer_der_y, 1, alpha_int);
    double Cyp = interpolazioneTotale(aer_der_y, 3, alpha_int);
    double Cyr = interpolazioneTotale(aer_der_y, 4, alpha_int);
    double Cydr = interpolazioneTotale(control_force_der, 6, alpha_int);
    double Czss = interpolazioneTotale(steady_state_coefficients, 3, alpha_int);
    double Cza = interpolazioneTotale(aer_der_z, 1, alpha_int);
    double Czq = interpolazioneTotale(aer_der_z, 4, alpha_int);
    double Czde = interpolazioneTotale(control_force_der, 3, alpha_int);
    double Clb = interpolazioneTotale(rolling_moment_der, 1, alpha_int);
    double Clp = interpolazioneTotale(rolling_moment_der, 3, alpha_int);
    double Clr = interpolazioneTotale(rolling_moment_der, 4, alpha_int);
    double Clda = interpolazioneTotale(control_moment_der, 1, alpha_int);
    double Cldr = interpolazioneTotale(control_moment_der, 2, alpha_int); 
    double Cmss = interpolazioneTotale(steady_state_coefficients, 5, alpha_int);
    double Cma = interpolazioneTotale(pitch_moment_der, 1, alpha_int);
    double Cmq = interpolazioneTotale(pitch_moment_der, 4, alpha_int);
    double Cmde = interpolazioneTotale(control_moment_der, 3, alpha_int);
    double Cnss = interpolazioneTotale(steady_state_coefficients, 6, alpha_int);
    double Cnb = interpolazioneTotale(yawing_moment_der, 1, alpha_int);
    double Cnp = interpolazioneTotale(yawing_moment_der, 3, alpha_int);
    double Cnr = interpolazioneTotale(yawing_moment_der, 4, alpha_int);
    double Cnda = interpolazioneTotale(control_moment_der, 5, alpha_int);
    double Cndr = interpolazioneTotale(control_moment_der, 6, alpha_int);

    //Calcolo i momenti di Inerzia sui vari assi e la massa
    double Jx = body_axes[13], Jy = body_axes[14], Jz = body_axes[15], m = body_axes[0];

    // Calcolo Forze e Momenti (t = 0);
    X = costante*(Cxss+Cxa*alpha+Cxde*de);
    Y = costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
    Z = costante*(Czss+Cza*alpha+Czq*q_ad+Czde*de);
    L = costante*body_axes[2]*(Clb*beta+Clp*p_ad+Clr*r_ad+Clda*da+Cldr*dr);
    M = costante*body_axes[3]*(Cmss+Cma*alpha+Cmq*q_ad+Cmde*de);
    N = costante*body_axes[2]*(Cnss+Cnb*beta+Cnp*p_ad+Cnr*r_ad+Cnda*da+Cndr*dr);

    /*printf("x: %lf\n", X);
    printf("y: %lf\n", Y);
    printf("z: %lf\n", Z);
    printf("L: %lf\n", L);
    printf("M: %lf\n", M);
    printf("N: %lf\n", N);*/

    // Incrementi tempo t = 0[s].
    du     = (r*v-q*w)-g*sin(theta)+X/m+T/m;  
    dv     = (p*w-r*u)+g*sin(phi)*cos(theta)+Y/m;
    dw     = (q*u-p*v)+g*cos(phi)*cos(theta)+Z/m;
    dp     = -(Jz-Jy)*q*r/Jx+L/Jx;
    dq     = -(Jx-Jz)*p*r/Jy+M/Jy;
    dr     = -(Jy-Jx)*p*q/Jz+N/Jz;
    dphi   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    dtheta = q*cos(phi) - r*sin(phi);
    dpsi   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    dh     = -u*sin(theta) + v*cos(theta)*sin(phi) + w*cos(theta)*cos(phi);
    dx_ned = u*cos(psi)*cos(theta) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    dy_ned = u*sin(psi)*cos(theta) + v*(sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));

    /*printf("du: %lf\n", du);
    printf("dv: %lf\n", dv);
    printf("dw: %lf\n", dw);
    printf("dp: %lf\n", dp);
    printf("dq: %lf\n", dq);
    printf("dr: %lf\n", dr);
    printf("dphi: %lf\n", dphi);
    printf("dtheta: %lf\n", dtheta);
    printf("dpsi: %lf\n", dpsi);
    printf("dh: %lf\n", dh);
    printf("dx_ned: %lf\n", dx_ned);
    printf("dy_ned: %lf\n", dy_ned);*/
    
    // Vettore di stato dopo condizione di trim.
    state[i+1][0]  = u + dt*du;
    state[i+1][1]  = v + dt*dv;
    state[i+1][2]  = w + dt*dw;
    state[i+1][3]  = p + dt*dp;
    state[i+1][4]  = q + dt*dq;
    state[i+1][5]  = r + dt*dr;
    state[i+1][6]  = phi + dt*dphi;
    state[i+1][7]  = theta + dt*dtheta;
    state[i+1][8]  = psi + dt*dpsi;
    state[i+1][9]  = h + dt*dh;
    state[i+1][10] = x_ned + dt*dx_ned;
    state[i+1][11] = y_ned + dt*dy_ned;

    /*printf("Posizione x: %f\n", state[i][10]);
    printf("Posizione y: %f\n", state[i][11]);
    printf("Velocità u: %f\n", state[i][0]);
    printf("Velocità w: %f\n", state[i][2]);
    printf("Altezza: %f\n", state[i][9]);
    printf("---------\n");*/
    
}