#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "routh.h"
#include "../Error_Warning/ErrorWarning.h"
#include "../Error_Warning/InitialCondition.h"
#include "../Pre_processing/Variables.h"

#define pi 3.14159265

int routh(double Cm_q, double alpha_trim, double V, double Cx_alpha, double Cz_alpha, double Cm_alpha, double Cm_alphaprimo){

    system("cls");
    printf("\n>>>-----------------------------------------------------------------<<<\n");
    printf(  ">    [ PRE-PROCESSING ]  >>  Calcolo condizioni di stabilita' ...     <\n");
    printf(  ">>>-----------------------------------------------------------------<<<\n\n");

    static int stampa = 1;

    //Variabili geometriche
    double massa_adm;
    double inerziaY_adm;
    double omega_ph, omega_sp;

    alpha_trim = alpha_trim*(pi/180);

    //Massa e Momento di inerzia adimensionali
    massa_adm = 2*body_axes[0]/(rho_h*body_axes[2]*body_axes[3]);
    inerziaY_adm = 8*body_axes[14]/(rho_h*body_axes[2]*pow(body_axes[3],3));

    //Variabili Aerodinamiche
    double CL_alpha,CLalpha_primo, CLe,CDe,CDalpha, Cwe,CTu;
    double k = 0.047, Cd0 = 0.0235;

    //Impostiamo il Clalpha nel nuovo (SdR)
    CL_alpha = Cx_alpha*sin(alpha_trim) - Cz_alpha*cos(alpha_trim); // da modificare
    CLalpha_primo = 1.56; // (rad^-1)
    CLe = body_axes[0]*9.81*2/(rho_h*body_axes[2]*V*V);
    CDe = Cd0 + k*CLe*CLe;  //k = 0.047 CD0 = 0.0235
    CDalpha = 2*k*CL_alpha*CLe; //
    Cwe = CLe;
    CTu = -3*CDe; //-0.0841

    if(Cm_alpha>0) MY_ERROR(402);


    //Coefficienti quartica
    double A, B, C, D, E, Delta;

    A = 2*massa_adm*inerziaY_adm*(2*massa_adm + CLalpha_primo);
    B = 2*massa_adm*inerziaY_adm*(CL_alpha+CDe-CTu) - 4*(Cm_q+Cm_alphaprimo)*massa_adm*massa_adm - inerziaY_adm*CTu*CLalpha_primo -2*massa_adm*Cm_q*CLalpha_primo;
    C = 2*massa_adm*(Cm_q*(CTu-CL_alpha-CDe) - 2*massa_adm*Cm_alpha + Cm_alphaprimo*CTu) + inerziaY_adm*(2*Cwe*(Cwe-CDalpha)+CTu*CL_alpha+CDe*CL_alpha) + Cm_q*CTu*CLalpha_primo;
    D = -2*Cm_alphaprimo*Cwe*Cwe + 2*massa_adm*CTu*Cm_alpha + CTu*Cm_q*CL_alpha -2*Cwe*Cm_q*(CLe-CDalpha)+2*CDe*Cm_q*CTu;
    E = -2*Cm_alpha*Cwe*Cwe;
    Delta = B*C*D - A*D*D - B*B*E;

    if(B<0||Delta<0||D<0||E<0) MY_ERROR(403);

    double omegaNph = (Cwe/(sqrt(2)*massa_adm))*2*V/(body_axes[3]);
    double zph = 3*CDe/(2*sqrt(2)*Cwe);

    double omegaNph_adm = -CTu/(4*massa_adm*zph);
    double Reph = -zph*omegaNph;
    double Imph = omegaNph*sqrt(fabs(zph*zph - 1.0));

    double Tph = 2*pi/Imph;
    double T12_ph = fabs(log(0.5)/Reph);

    //double omegaNsp_adm = sqrt(-Cm_alpha/inerziaY_adm);
    double omegaNsp_adm = sqrt(-(2*massa_adm*Cm_alpha+Cm_q*CL_alpha)/(2*massa_adm*inerziaY_adm));  //Modello completo non semplificato  
    //double omegaNsp_adm = sqrt(-Cm_alpha/inerziaY_adm-(Cm_q*CL_alpha)/(2*massa_adm*inerziaY_adm));
    double omegaNsp = (omegaNsp_adm*2*V)/(body_axes[3]);
    double zsp = (inerziaY_adm*CL_alpha-2*massa_adm*(Cm_q+Cm_alphaprimo))/(2*sqrt(-2*massa_adm*inerziaY_adm*(2*massa_adm*Cm_alpha+Cm_q*CL_alpha)));
    //double omegasp = omegaNsp*sqrt(fabs(zsp*zsp - 1));

    double Resp = -zsp*omegaNsp;
    double Imsp = omegaNsp*sqrt(fabs(zsp*zsp - 1.0));

    double Tsp = 2*pi/Imsp;
    double T12_sp = fabs(log(0.5)/Resp);

    printf("Caratteristiche modi:\n\n");
    printf("     ***************************************************************************************\n");
    printf("     * - MODO FUGOIDE:                           * - MODO CORTO PERIODO:                   *\n" );
    printf("     * - omega[rad/s] = %.3f                    * - omega[rad/s]= %.3lf                   *\n",omegaNph,omegaNsp);
    printf("     * - smorzamento[-] = %.3lf                  * - smorzamento[-]= %.3lf                 *\n",zph,zsp);
    printf("     * - periodo[s] = %.3lf                     * - periodo[s]= %.3lf                     *\n",Tph,Tsp);
    printf("     * - tempo di dimezzamento[s] = %.3lf       * - tempo di dimezzamento[s]= %.3lf       *\n",T12_ph,T12_sp);
    printf("     ***************************************************************************************\n\n");
    
    system("PAUSE");
}
