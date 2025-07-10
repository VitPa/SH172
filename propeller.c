#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "EstrazioneDati.h"
#include "Variables.h"

#define pi 3.14159265

// Modifica: la funzione ora prende un double* prop come argomento di output
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
    float pitch = 0.0; // pitch del'elica
    float diam=geometry_propeller[0]; //diametro elica
    float Raggio=diam/2.0; //raggio elica
    float tip=data_propeller_p[48][3]*(pi/180); //angolo di beccheggio al tip
    float xt=Raggio; //coordinata dimensionalizzata al tip
    float hub=data_propeller_p[0][3]*(pi/180); //angolo di beccheggio all'hub (al 25% del raggio)
    float xs=data_propeller_p[0][0]*Raggio; //coordinata dimensionalizzata all'hub
    float n=RPM_ref/60.0; //round-per-second
    float omega=n*2.0*pi; //velocià angolare [rad/s]
    float coef1=(tip-hub)/(xt-xs); //coefficiente #1 di supporto al calcolo dell'angolo di svergolamento theta
    float coef2=hub-coef1*xs+pitch; //coefficiente #2 di supporto al calcolo dell'angolo di svergolamento theta
    float rstep=(xt-xs)/48; //calcolo step delle 48 stazioni        
    float r1[49]; //creazione vettore delle 48 stazioni (-> CSI in propeller.txt)
    float t2[49], a2[49],b2[49];
    float th, phi1, eff, DtDr,DqDr,cl,cd,CT,CQ, tem1,tem2;
    float a, anew;
    float b, bnew;
    int j=0;
    int finished=0;
    /*Inizializzazione r1*/
    for(j=0;j<49;j++){
        r1[j]=xs+j*rstep;           //V ricalcolato oppure va preso dal file
    }
    float rad;
    float Vlocal,V0,V2;
    float thrust=0.0; //inizializzazione vettore spinta
    float torque=0.0;//inizializzazione vettore coppia
    for(j=0; j<49; j++){
        rad=r1[j]; //coordinata j-esima stazione (-> CSI in propeller.txt)
        theta1=data_propeller_p[j][3]*(pi/180) + pitch; //calcolo angolo di svergolamento della j-esima stazione        //sostituire con BA(j) + pitch
        t2[j]=theta1; //angolo di svergolamento della j-esima stazione (-> BA su propeller.txt) 
        th=theta1; //angolo di svergolamento [rad]
        a=0.1; //inizializzazione axial inflow factor (vedi pag.4 PROPEL.pdf)
        b=0.01; //inizializzazione angular inflow (swirl) factor (vedi pag.4 PROPEL.pdf)
        finished=0; //inizializzione flag
        int sum=1; //inizializzione variabile di supporto
        while (finished==0){
            V0=Vel*(1+a); //componente del flusso all'incirca uguale alla velocità di avanzamento del velivolo (Vinf), aumentata tramite l'axial inflow factor
            V2=omega*rad*(1-b); //componente del flusso all'incirca uguale alla velocità angolare della sezione della pala (omega*rad), ridotta tramite l'angular inflow factor
            phi1=atan2(V0,V2); //angolo tra le due componenti del flusso V0 e V2
            alpha1=th-phi1; //angolo di attacco raltivo alla j-esima sezione della pala        //CESSNA theta - phi
            cl=propeller_profile[1]+propeller_profile[0]*alpha1; //L coefficiente di portanza
            cd=propeller_profile[5]+propeller_profile[4]*alpha1+propeller_profile[3]*alpha1*alpha1; // CD coefficiente di resistenza CD = CD0+CD1*CL+CD2*CL^2 (NB nel nostro caso, CD = CD0+CD_alpha*alpha+CD_alpha2*alpha^2 -> slide lezione 2)
            Vlocal=sqrt(V0*V0+V2*V2); // velocità locale del flusso
            CT = cl*cos(phi1)-cd*sin(phi1); //CT coefficiente di spinta adimensionale
            DtDr=0.5*rho_h*Vlocal*Vlocal*geometry_propeller[2]*data_propeller_p[j][2]*Raggio*CT; //contributo di spinta della j-esima sezione      //IL NUMERO 2.0 è il numero di pale
            CQ = cd*cos(phi1)+cl*sin(phi1); //CQ coefficiente di coppia adimensionale
            DqDr=0.5*rho_h*Vlocal*Vlocal*geometry_propeller[2]*data_propeller_p[j][2]*Raggio*Raggio*rad*CQ; //contributo di coppia della j-esima sezione  //IL NUMERO 2.0 è il numero di pale
            tem1=DtDr/(4.0*pi*rad*rho_h*Vel*Vel*(1+a)); //fattore correttivo del coefficiente "a"
            tem2=DqDr/(4.0*pi*rad*rad*rad*rho_h*Vel*(1+a)*omega); //fattore correttivo del coefficiente "b"
            anew=0.5*(a+tem1); //nuovo valore coefficiente "a"
            bnew=0.5*(b+tem2); //nuovo valore coefficiente "b"
            //processo iterativo per arrivare a convergenza
            if (fabs(anew-a)<1/100000){
                if (fabs(bnew-b)<1/100000){
                    finished=1;
                }
            }
            a=anew; //definizione valore finale coefficiente "a"
            b=bnew; //definizione valore finale coefficiente "b"
            sum=sum+1;
            if (sum>500){
                finished=1;
            }
        }
        a2[j]=a; //definizione valore finale coefficiente "a" per la j-esima stazione
        b2[j]=b; //definizione valore finale coefficiente "b" per la j-esima stazione
        prop[0] += DtDr*rstep; //sommatoria dei contributi di spinta dalla stazione 1 alla stazione j
        prop[1] += DqDr*rstep; //sommatoria dei contributi di coppia dalla stazione 1 alla stazione j
    }

    double t = prop[0]/(rho_h*pow(n,2)*pow(diam,4)); //coefficiente di spinta adimensionale
    double q = prop[1]/(rho_h*pow(n,2)*pow(diam,5)); //coefficiente di coppia adimensionale
    double J = Vel/(n*diam); //rapporto di avanzamento

    if (t<0){
        prop[2] = 0.0; //efficienza elica
    }else{
        //prop[2] = t/q*(J*diam)/(2.0*pi); //efficienza elica
        prop[2] = (J * t) / (2.0 * pi * q); //CONTROLLARE
        if (prop[2] > 1.0) {
            printf("Efficienza > 1: eff=%.3f, t=%.3e, q=%.3e, J=%.3f, diam=%.3f, Vel=%.3f, n=%.3f, prop[0]=%.3f, prop[1]=%.3f\n",
                prop[2], t, q, J, diam, Vel, n, prop[0], prop[1]);
        }
    }

    *Pal = (prop[1]*omega)/1000;

    if (*Pal>Pmax_h) {
        *Pal = Pmax_h;
    }
}
double massConsumption(double Kc, double Pa, double np, double mass, double time){
    return mass - (Kc*Pa*1000/np)*time;
}