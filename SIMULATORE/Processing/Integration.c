#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Integration.h"
#include "Propeller.h"
#include "../Interpolation/Interpolation.h"
#include "../Pre_processing/Variables.h"
#include "../Pre_processing/Data.h"

void eulerEquation(double dt, int i){
    // *** Section: Extract state vector components ***
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

    // *** Section: Compute total velocity ***
    double V = sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));

    // *** Section: Extract command inputs for this time step ***
    double da    = command[i][0] *(pi/180);
    double de    = command[i][1] *(pi/180);
    double dr    = command[i][2] *(pi/180);
    double manetta = RPMmin + (RPMmax - RPMmin) * (command[i][3]) / (1);  // Map throttle [0, 1] -> [RPMmin, RPMmax];

    // *** Section: Compute propeller thrust ***
    double prop[3] =  {0.0, 0.0, 0.0}, Pal = 0.0;
    propel(manetta, V, prop, &Pal);
    double T = prop[0];
    

    // *** Section: Fuel consumption update (if enabled) ***
    if(fuel_mass[0] == 1) body_axes[0] = massConsumption(engine[5], Pal, prop[2], body_axes[0], dt);

    // *** Section: Aerodynamic and geometric parameters ***
    double S = body_axes[2];
    double costante=0.5*rho_h*V*V*S;

    double alpha = atan2(w,u);                      // Compute angles
    double beta = asin(v/V);
    
    double q_ad = q*body_axes[3]/(2*V);             // Compute dimensionless angular rates
    double p_ad = p*body_axes[1]/(2*V);
    double r_ad = r*body_axes[1]/(2*V);
    double alpha_int = alpha*(180/pi);

    // *** Section: Interpolate aerodynamic coefficients ***
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

    double Jx = body_axes[13], Jy = body_axes[14], Jz = body_axes[15], m = body_axes[0];    // Inertia and mass extraction

    // *** Section: Compute aerodynamic forces and moments ***
    double X = costante*(Cxss+Cxa*alpha+Cxde*de);
    double Y = costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
    double Z = costante*(Czss+Cza*alpha+Czq*q_ad+Czde*de);
    double L = costante*body_axes[1]*(Clb*beta+Clp*p_ad+Clr*r_ad+Clda*da+Cldr*dr);
    double M = costante*body_axes[3]*(Cmss+Cma*alpha+Cmq*q_ad+Cmde*de);
    double N = costante*body_axes[1]*(Cnss+Cnb*beta+Cnp*p_ad+Cnr*r_ad+Cnda*da+Cndr*dr);

    // *** Section: Compute time derivatives of state variables (equations of motion) ***
    double du_     = (r*v-q*w) - g*sin(theta) + X/m + T/m;
    double dv_     = (p*w-r*u) + g*sin(phi)*cos(theta) + Y/m;
    double dw_     = (q*u-p*v) + g*cos(phi)*cos(theta) + Z/m;
    double dp_     = (-(Jz-Jy)*q*r)/Jx + L/Jx;
    double dq_     = (-(Jx-Jz)*p*r)/Jy + M/Jy;
    double dr_     = (-(Jy-Jx)*p*q)/Jz + N/Jz;
    double dphi_   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    double dtheta_ = q*cos(phi) - r*sin(phi);
    double dpsi_   = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));
    double dh_     = u*sin(theta) - v*cos(theta)*sin(phi) - w*cos(theta)*cos(phi);  // Con questi segni, le manovre (equilibratore) vanno bene ma il trim va verso l'alto
    double dx_ned = u*cos(psi)*cos(theta) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    double dy_ned = u*sin(psi)*cos(theta) + v*(sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));

    double gamma = atan2(dh_, sqrt(dx_ned*dx_ned + dy_ned*dy_ned));
    fprintf(agg, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n", i*dt, alpha_int, beta*(180/pi), gamma, V, T, -Z, body_axes[0]);
    fflush(agg);
    
    // *** Section: Update state vector using Euler integration ***
    state[0]  = u + dt*du_;
    state[1]  = v + dt*dv_;
    state[2]  = w + dt*dw_;
    state[3]  = p + dt*dp_;
    state[4]  = q + dt*dq_;
    state[5]  = r + dt*dr_;
    state[6]  = phi + dt*dphi_;
    state[7]  = theta + dt*dtheta_;
    state[8]  = psi + dt*dpsi_;
    state[9]  = h + dt*dh_;
    state[10] = x_ned + dt*dx_ned;
    state[11] = y_ned + dt*dy_ned;
}

void progressBar(double Ts, double deltaT_fs, const char* context){
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
        printf("] %s", context);
    }
}