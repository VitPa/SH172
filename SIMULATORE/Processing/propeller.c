#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Propeller.h"
#include "../Pre_processing/Variables.h"
#include "../Pre_processing/Data.h"

double propel(double RPM_ref, double Vel, double* prop, double *Pal)
{
    double data_propeller_p[49][4];
    int k = 0, z = 0;
    for(int i=11; i<60; ++i){
        for(int j=0; j<4; ++j){
            data_propeller_p[k][z] = data_propeller[i][j];
            ++z;
        }
        ++k;
        z=0;
    }
    
    float alpha1, theta1;
    float pitch = 0.0;                              // propeller pitch
    float diam=geometry_propeller[0];               // propeller diameter
    float Raggio=diam/2.0;                          // propeller radius
    float tip=data_propeller_p[48][3]*(pi/180);     // pitch angle at tip
    float xt=Raggio;                                // dimensionalized coordinate at tip
    float hub=data_propeller_p[0][3]*(pi/180);      // pitch angle at hub (at 25% of radius)
    float xs=data_propeller_p[0][0]*Raggio;         // dimensionalized coordinate at hub
    float n=RPM_ref/60.0;
    float omega=n*2.0*pi;                           // angular velocity [rad/s]
    float coef1=(tip-hub)/(xt-xs);                  // coefficient #1 for twist angle calculation
    float coef2=hub-coef1*xs+pitch;                 // coefficient #2 for twist angle calculation
    float rstep=(xt-xs)/48;                         // step size for 48 stations        
    float r1[49];                                   // vector of 48 stations (-> CSI in propeller.txt)
    float t2[49], a2[49],b2[49];
    float th, phi1, eff, DtDr,DqDr,cl,cd,CT,CQ, tem1,tem2;
    float a, anew;
    float b, bnew;
    int j=0;
    int finished=0;
    for(j=0;j<49;j++){
        r1[j]=xs+j*rstep;
    }
    float rad;
    float Vlocal,V0,V2;
    float thrust=0.0;
    float torque=0.0;
    for(j=0; j<49; j++){
    rad=r1[j];                                          // coordinate of j-th station
        theta1=data_propeller_p[j][3]*(pi/180) + pitch; // twist angle of j-th station
        t2[j]=theta1;                                   // twist angle of j-th station
        th=theta1;                                      // twist angle [rad]
        a=0.1;                                          // initialize axial inflow factor
        b=0.01;                                         // initialize angular inflow (swirl) factor
        finished=0;
        int sum=1;
        while (finished==0){
            V0=Vel*(1+a);                                           // flow component approximately equal to aircraft speed (Vinf), increased by axial inflow factor
            V2=omega*rad*(1-b);                                     // flow component approximately equal to angular speed of blade section (omega*rad), reduced by angular inflow factor
            phi1=atan2(V0,V2);                                      // angle between the two flow components V0 and V2
            alpha1=th-phi1;                                         // angle of attack relative to j-th blade section
            cl=propeller_profile[1]+propeller_profile[0]*alpha1;    // lift coefficient
            cd=propeller_profile[5]+propeller_profile[4]*alpha1+propeller_profile[3]*alpha1*alpha1; // drag coefficient CD = CD0+CD_alpha*alpha+CD_alpha2*alpha^2
            Vlocal=sqrt(V0*V0+V2*V2);                               // local flow speed
            CT = cl*cos(phi1)-cd*sin(phi1);                         // non-dimensional thrust coefficient
            DtDr=0.5*rho_h*Vlocal*Vlocal*geometry_propeller[2]*data_propeller_p[j][2]*Raggio*CT; // thrust contribution of j-th section
            CQ = cd*cos(phi1)+cl*sin(phi1);                         // non-dimensional torque coefficient
            DqDr=0.5*rho_h*Vlocal*Vlocal*geometry_propeller[2]*data_propeller_p[j][2]*Raggio*Raggio*rad*CQ; // torque contribution of j-th section
            tem1=DtDr/(4.0*pi*rad*rho_h*Vel*Vel*(1+a));             // correction factor for coefficient "a"
            tem2=DqDr/(4.0*pi*rad*rad*rad*rho_h*Vel*(1+a)*omega);   // correction factor for coefficient "b"
            anew=0.5*(a+tem1);                                      // new value for coefficient "a"
            bnew=0.5*(b+tem2);                                      // new value for coefficient "b"

            if (fabs(anew-a)<1/100000){     // iterative process for convergence
                if (fabs(bnew-b)<1/100000){
                    finished=1;
                }
            }
            a=anew;                         // final value for coefficient "a"
            b=bnew;                         // final value for coefficient "b"
            sum=sum+1;
            if (sum>500){
                finished=1;
            }
        }
        a2[j]=a;                            // final value for coefficient "a" for j-th station
        b2[j]=b;                            // final value for coefficient "b" for j-th station
        prop[0] += DtDr*rstep;              // sum of thrust contributions from station 1 to j
        prop[1] += DqDr*rstep;              // sum of torque contributions from station 1 to j
    }

    double t = prop[0]/(rho_h*pow(n,2)*pow(diam,4)); // non-dimensional thrust coefficient
    double q = prop[1]/(rho_h*pow(n,2)*pow(diam,5)); // non-dimensional torque coefficient
    double J = Vel/(n*diam);                // advance ratio

    if (t<0){
        prop[2] = 0.0; // propeller efficiency
    }else{
        prop[2] = (J * t) / (2.0 * pi * q);
    }

    *Pal = (prop[1]*omega)/1000;
    if (*Pal>Pmax_h) {
        *Pal = Pmax_h;
    }
}
double massConsumption(double Kc, double Pa, double np, double mass, double time){
    if (np <= 0.0) return mass;
    return mass - (Kc*Pa*1000/np)*time;
}