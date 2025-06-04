#include "Atmosphere.h"
#include "Interpolazione_new.h"
#include "propeller.h"

void Integration(double *state_trim, double *comandi, double *body_axes, double *CI, double RPM, double **datiengine, double *Pmax_h, double *press0, double *temp0, double *rho0, double *vsuono0, double *press_h, double *temp_h, double *rho_h, double *vsuono_h, int *flagatm, double **steady_state_coefficients, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **geometry_propeller, double **propeller_profile, double **data_propeller){
    float dt=0.01;
    int i=0;
    float deltaT_fs;
    double t_fc; //tempo dato dall'utente di fine comando
    int n=(deltaT_fs/dt); //devo prendere l'intero inferiore.;
    double X,Y,Z,L,M,N, costante;
    double u,v,w,p,q,r,phi,theta,psi,h,x_ned,y_ned;
    double da, de, dr, manetta;
    double V, rho, S, alpha, beta, q_ad, r_ad, p_ad, T;
    double du, dv, dw, dp, dq, dr, dphi, dtheta, dpsi, dh, dx_ned, dy_ned;
    double Jx = body_axes[13], Jy = body_axes[14], Jz = body_axes[15], m = body_axes[0];
    double prop[3], Pal;
    
    #define g 9.80665
    #define pi 3.14159265

    double state[n+1][12];  // la prima colonna indica il tempo, la seconda la condizione di trim.
    double COM[n+1][4]; // La prima colonna indica il tempo, la seconda i comandi.
    
    for (i=0; i<=n; i=i+1){
        if (i==0){
            //Attenzione state_trim deve essere anche lui da 12 elementi
            for (int j = 0; j < 12; j++) {
                state[0][j] = state_trim[j];
            }
            
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
            
            // Richiamo componenti vettore dei comandi (COM) in condizioni di trim:
            for (int j = 0; j < 12; j++) {
                COM[0][j] = comandi[j];
            }
            da = COM[i][0];
            de = COM[i][1];
            dr = COM[i][2];
            manetta = COM[i][3];
            
            // Definizione rho dinamico
            double temp[1] = {V};
            rho = AtmosphereCalc(temp, datiengine, Pmax_h, press0, temp0, rho0, vsuono0, press_h, temp_h, rho_h, vsuono_h, flagatm);
            
            // Velostatetà totale iniziale (t = 0);
            S = body_axes[2];
            V=sqrt(u*u + v*v + w*w);
            costante=0.5*rho*V*V*S;  //Inserire dipendenza dalla quota
            
            // Definizione alpha e beta iniziali (t = 0):
            alpha = atan2(w/u);
            beta = asin(v/V);

            //Calcolo velocità angolari adimensionali
            q_ad = q*body_axes[3]/(2*V);
            p_ad = p*body_axes[1]/(2*V);
            r_ad = r*body_axes[1]/(2*V);

            // Calcolo coefficienti aerodinamici
            double Cxss = interpolazioneTotale(steady_state_coefficients, 1, alpha);
            double Cxa = interpolazioneTotale(aer_der_x, 1, alpha);
            double Cxde = interpolazioneTotale(control_force_der, 1, alpha);
            double Cyb = interpolazioneTotale(aer_der_y, 1, alpha);
            double Cyp = interpolazioneTotale(aer_der_y, 3, alpha);
            double Cyr = interpolazioneTotale(aer_der_y, 4, alpha);
            double Cydr = interpolazioneTotale(control_force_der, 6, alpha);
            double Czss = interpolazioneTotale(steady_state_coefficients, 3, alpha);
            double Cza = interpolazioneTotale(aer_der_z, 1, alpha);
            double Czq = interpolazioneTotale(aer_der_z, 4, alpha);
            double Czde = interpolazioneTotale(control_force_der, 3, alpha);
            double Clb = interpolazioneTotale(rolling_moment_der, 1, alpha);
            double Clp = interpolazioneTotale(rolling_moment_der, 3, alpha);
            double Clr = interpolazioneTotale(rolling_moment_der, 4, alpha);
            double Clda = interpolazioneTotale(control_moment_der, 1, alpha);
            double Cldr = interpolazioneTotale(control_moment_der, 2, alpha); 
            double Cmss = interpolazioneTotale(steady_state_coefficients, 5, alpha);
            double Cma = interpolazioneTotale(pitch_moment_der, 1, alpha);
            double Cmq = interpolazioneTotale(pitch_moment_der, 4, alpha);
            double Cmde = interpolazioneTotale(control_moment_der, 3, alpha);
            double Cnss = interpolazioneTotale(steady_state_coefficients, 6, alpha);
            double Cnb = interpolazioneTotale(yawing_moment_der, 1, alpha);
            double Cnp = interpolazioneTotale(yawing_moment_der, 3, alpha);
            double Cnr = interpolazioneTotale(yawing_moment_der, 4, alpha);
            double Cnda = interpolazioneTotale(control_moment_der, 5, alpha);
            double Cndr = interpolazioneTotale(control_moment_der, 6, alpha);

            // Calcolo Spinta
            T = propel(RPM, rho, V, geometry_propeller, propeller_profile, data_propeller, prop, &Pal);

            // Calcolo Forze e Momenti (t = 0);
            X = costante*(Cxss*alpha+Cxa*alpha+Cxde*de);
            Y = costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
            Z = costante*(Czss*alpha+Cza*alpha+Czq*q_ad+Czde*de);
            L = costante*(Clb*beta+Clp*p+Clr*r+Clda*da+Cldr*dr);
            M = costante*(Cmss*alpha + Cma*alpha + Cmq*q + Cmde*de);
            N = costante*(Cnss*alpha + Cnb*beta + Cnp*p + Cnr*r + Cnda*da + Cndr*dr);

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
            dx_ned = u*cos(psi)*cos(theta) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(theta)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
            dy_ned = u*sin(psi)*cos(theta) - v*(sin(psi)*sin(theta)*sin(phi) - cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
            
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
            state[i+1][9]  = h + dt*h;
            state[i+1][10] = x_ned + dt*dx_ned;
            state[i+1][11] = y_ned + dt*dy_ned;
        }else{
            
            // Richiamo componenti vettore di stato state
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

            // Velostatetà totale iniziale:
            V=sqrt(u*u + v*v + w*w);

            // Definizione rho dinamico
            double temp[1] = {V};
            rho = AtmosphereCalc(temp, datiengine, Pmax_h, press0, temp0, rho0, vsuono0, press_h, temp_h, rho_h, vsuono_h, flagatm);

            //Calcolo velocità angolari adimensionali
            q_ad = q*body_axes[3]/(2*V);
            p_ad = p*body_axes[1]/(2*V);
            r_ad = r*body_axes[1]/(2*V);

            // Definizione alpha e beta:
            alpha = atan2(w/u);
            beta = asin(v/V);

            // Calcolo coefficienti aerodinamici
            double Cxss = interpolazioneTotale(steady_state_coefficients, 1, alpha);
            double Cxa = interpolazioneTotale(aer_der_x, 1, alpha);
            double Cxde = interpolazioneTotale(control_force_der, 1, alpha);
            double Cyb = interpolazioneTotale(aer_der_y, 1, alpha);
            double Cyp = interpolazioneTotale(aer_der_y, 3, alpha);
            double Cyr = interpolazioneTotale(aer_der_y, 4, alpha);
            double Cydr = interpolazioneTotale(control_force_der, 6, alpha);
            double Czss = interpolazioneTotale(steady_state_coefficients, 3, alpha);
            double Cza = interpolazioneTotale(aer_der_z, 1, alpha);
            double Czq = interpolazioneTotale(aer_der_z, 4, alpha);
            double Czde = interpolazioneTotale(control_force_der, 3, alpha);
            double Clb = interpolazioneTotale(rolling_moment_der, 1, alpha);
            double Clp = interpolazioneTotale(rolling_moment_der, 3, alpha);
            double Clr = interpolazioneTotale(rolling_moment_der, 4, alpha);
            double Clda = interpolazioneTotale(control_moment_der, 1, alpha);
            double Cldr = interpolazioneTotale(control_moment_der, 2, alpha); 
            double Cmss = interpolazioneTotale(steady_state_coefficients, 5, alpha);
            double Cma = interpolazioneTotale(pitch_moment_der, 1, alpha);
            double Cmq = interpolazioneTotale(pitch_moment_der, 4, alpha);
            double Cmde = interpolazioneTotale(control_moment_der, 3, alpha);
            double Cnss = interpolazioneTotale(steady_state_coefficients, 6, alpha);
            double Cnb = interpolazioneTotale(yawing_moment_der, 1, alpha);
            double Cnp = interpolazioneTotale(yawing_moment_der, 3, alpha);
            double Cnr = interpolazioneTotale(yawing_moment_der, 4, alpha);
            double Cnda = interpolazioneTotale(control_moment_der, 5, alpha);
            double Cndr = interpolazioneTotale(control_moment_der, 6, alpha);

            // Calcolo Spinta
            T = propel(RPM, rho, V, geometry_propeller, propeller_profile, data_propeller, prop, &Pal);

            // Definizione di una costante semplificativa
            costante=0.5*rho*V*V*S;

            // Calcolo Forze e Momenti:
            X=costante*(Cxss*alpha+Cxa*alpha+Cxde*de);
            Y=costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
            Z=costante*(Czss*alpha+Cza*alpha+Czq*q_ad+Czde*de);
            L=costante*(Clb*beta+Clp*p+Clr*r+Clda*da+Cldr*dr);
            M=costante*(Cmss*alpha + Cma*alpha + Cmq*q + Cmde*de);
            N=costante*(Cnss*alpha + Cnb*beta + Cnp*p + Cnr*r + Cnda*da + Cndr*dr);
            // Incrementi tempo:
            du=(r*v-q*w)-g*sin(theta)+X/m+T/m;  
            dv=(p*w-r*u)+g*sin(phi)*cos(theta)+Y/m;
            dw=(q*u-p*v)+g*cos(phi)*cos(theta)+Z/m;
            dp=-(Jz-Jy)*q*r/Jx+L/Jx;
            dq=-(Jx-Jz)*p*r/Jy+M/Jy;
            dr=-(Jy-Jx)*p*q/Jz+N/Jz;
            dphi = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
            dtheta = q*cos(phi) - r*sin(phi);
            dpsi = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
            dh = -u*sin(theta) + v*cos(theta)*sin(phi) + w*cos(theta)*cos(phi);
            dx_ned = u*cos(psi)*cos(theta) + v*(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(theta)) + w*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
            dy_ned = u*sin(psi)*cos(theta) - v*(sin(psi)*sin(theta)*sin(phi) - cos(psi)*cos(phi)) + w*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
            // Vettore di stato dopo condizione di trim.
            state[i+1][0] = u + dt*du;
            state[i+1][1] = v + dt*dv;
            state[i+1][2] = w + dt*dw;
            state[i+1][3] = p + dt*dp;
            state[i+1][4] = q + dt*dq;
            state[i+1][5] = r + dt*dr;
            state[i+1][6] = phi + dt*dphi;
            state[i+1][7] = theta + dt*dtheta;
            state[i+1][8] = psi + dt*dpsi;
            state[i+1][9] = h + dt*h;
            state[i+1][10] = x_ned + dt*dx_ned;
            state[i+1][11] = y_ned + dt*dy_ned;
        }
    }

}


