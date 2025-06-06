#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <complex.h>
#include <math.h>

#define MAX_ELEMENTI 30
#define pi 3.14159265

int routh(double Cm_q,double* body_axes, double rho, double alpha_trim, double V, double Cx_alpha, double Cz_alpha, double Cmalpha_dati, double Cmalpha_primo){

    //Variabili geometriche
    double massa_adm;
    double inerziaY_adm;
    double omega_ph, omega_sp;

    //Massa e Momento di inerzia adimensionali
    massa_adm = 2*body_axes[0]/(rho*body_axes[2]*body_axes[3]);
    inerziaY_adm = 8*body_axes[14]/(rho*body_axes[2]*pow(body_axes[3],3));

    //Variabili Aerodinamiche
    double CL_alpha,CLalpha_primo,Cm_alpha,Cm_alphaprimo, CLe,CDe,CDalpha, Cwe,CTu;
    double k = 0.047, Cd0 = 0.0235;

    //Impostiamo il Clalpha nel nuovo (SdR)
    CL_alpha = Cx_alpha*sin(alpha_trim) - Cz_alpha*cos(alpha_trim); // da modificare
    // printf("\nCl_alpha = %lf",CL_alpha); // da modificare
    CLalpha_primo = 1.56; // (rad^-1)
    Cm_alpha = Cmalpha_dati;  //da modificare
    Cm_alphaprimo = Cmalpha_primo; // da modificare
    CLe = body_axes[0]*9.81*2/(rho*body_axes[2]*V*V);
    CDe = Cd0 + k*CLe*CLe;  //k = 0.047 CD0 = 0.0235
    CDalpha = 2*k*CL_alpha*CLe; //
    Cwe = CLe;
    CTu = -3*CDe; //-0.0841


    if(Cm_alpha>0){
        printf("\nERRORE: -11  - Il Cessna e' staticamente instabile.");
        return -11;
    }


    //Coefficienti quartica
    double A, B, C, D, E, Delta;

    A = 2*massa_adm*inerziaY_adm*(2*massa_adm + CLalpha_primo);
    B = 2*massa_adm*inerziaY_adm*(CL_alpha+CDe-CTu) - 4*(Cm_q+Cm_alphaprimo)*massa_adm*massa_adm - inerziaY_adm*CTu*CLalpha_primo -2*massa_adm*Cm_q*CLalpha_primo;
    C = 2*massa_adm*(Cm_q*(CTu-CL_alpha-CDe) - 2*massa_adm*Cm_alpha + Cm_alphaprimo*CTu) + inerziaY_adm*(2*Cwe*(Cwe-CDalpha)+CTu*CL_alpha+CDe*CL_alpha) + Cm_q*CTu*CLalpha_primo;
    D = -2*Cm_alphaprimo*Cwe*Cwe + 2*massa_adm*CTu*Cm_alpha + CTu*Cm_q*CL_alpha -2*Cwe*Cm_q*(CLe-CDalpha)+2*CDe*Cm_q*CTu;
    E = -2*Cm_alpha*Cwe*Cwe;
    Delta = B*C*D - A*D*D - B*B*E;

    // printf("\nA = %lf\nB = %lf\nC = %lf\nD = %lf\nE = %lf\nDelta = %lf",A,B,C,D,E,Delta);


    if(B<0||Delta<0||D<0||E<0){
        printf("\nERRORE: -12  - Il Cessna e' dinamicamente instabile.");
        return -12;
    }

    double omegaNph = (Cwe/(sqrt(2)*massa_adm))*2*V/(body_axes[3]);
    double zph = 3*CDe/(2*sqrt(2)*Cwe);

    double omegaNph_adm = -CTu/(4*massa_adm*zph);
    double Reph = -zph*omegaNph;
    double Imph = omegaNph*sqrt(fabs(zph*zph - 1.0));

    printf("\nReph = %lf\nImph = %lf\n",Reph,Imph);
    printf("\nomegaNph = %lf\nomegaNph_adm = %lf\n",omegaNph,omegaNph_adm);
    printf("\nzph = %lf\n",zph);

    double Tph = 2*pi/Imph;
    double T12_ph = fabs(log(0.5)/Reph);

    printf("\nTph = %lf\nT12_ph = %lf\n",Tph,T12_ph);

    printf("\n-----------------------------------\n");
    
    //double omegaNsp_adm = sqrt(-(2*massa_adm*Cm_alpha+Cm_q*CL_alpha)/(2*massa_adm*inerziaY_adm));
    double omegaNsp_adm = sqrt(-Cm_alpha/inerziaY_adm - (Cm_q*CL_alpha)/(2*massa_adm*inerziaY_adm));
    double omegaNsp = (omegaNsp_adm*2*V)/(body_axes[3]);
    double zsp = (inerziaY_adm*CL_alpha-2*massa_adm*(Cm_q+Cm_alphaprimo))/(2*sqrt(-2*massa_adm*inerziaY_adm*(2*massa_adm*Cm_alpha+Cm_q*CL_alpha)));
    double Resp = -zsp*omegaNsp;
    double Imsp = omegaNsp*sqrt(fabs(zsp*zsp - 1.0));

    printf("\nResp = %lf\nImsp = %lf\n",Resp,Imsp);
    printf("\nomegaNsp_adm = %lf\nomegaNsp = %lf\n",omegaNsp_adm, omegaNsp);
    printf("\nzsp = %lf\n",zsp);

    double Tsp = 2*pi/Imsp;
    double T12_sp = fabs(log(0.5)/Resp);

    printf("\nTsp = %lf\nT12_sp = %lf\n",Tsp,T12_sp);



    /*//Risoluzione quartica
    double _Complex delta0,Q,S,p,q,s;
    double _Complex soluzione1,soluzione2;
    double eta[2],omega[2],zeta[2],omega_n[2];

    q = 12*A*E - 3*B*D + C*C;
    s = 27*A*D*D -72*A*C*E + 27*E*B*B -9*B*C*D +2*C*C*C;
    //printf("%lf + %lfj\n%lf + %lfj\n",creal(q),cimagf(q),creal(s),cimag(s));
    double _Complex base = (s + csqrt(s*s -4*q*q*q))/2.0;
    delta0 = cpow(base, 1.0/3.0);
    p = (8*A*C-3*B*B)/(8*A*A);
    Q = 0.5*csqrt(-2*p/3 + (delta0 + q/delta0)/(3*A));
    S = (8*D*A*A - 4*A*B*C + B*B*B)/(8*A*A*A);

    soluzione1=-B/(4*A) - Q + 0.5*csqrt(-4*Q*Q -2*p + S/Q);
    soluzione2=-B/(4*A) + Q + 0.5*csqrt(-4*Q*Q -2*p - S/Q); 


    eta[0] = fabs(creal(soluzione1));          //ATTENZIONE: Ho messo fabs anzich� abs in quanto numeri double.
    eta[1] = fabs(creal(soluzione2));
    omega[0] = fabs(cimag(soluzione1));
    omega[1] = fabs(cimag(soluzione2));
    omega_n[0] = sqrt(eta[0]*eta[0]+omega[0]*omega[0]);           //OMEGA NATURALE
    omega_n[1] = sqrt(eta[1]*eta[1]+omega[1]*omega[1]);


    //Calcolo del Periodo, del Tempo di dimezzamento e dei coefficienti eta e omega dimensionali
    double periodo[2], t_mezzi[2],t_caratteristico;

    t_caratteristico = body_axes[3]/(2*V);
    zeta[0] = eta[0]/omega_n[0];               //ZETA
    zeta[1] = eta[1]/omega_n[1];               //ZETA
    omega[0] = omega[0]/t_caratteristico;
    omega[1] = omega[1]/t_caratteristico;

    int i = 0;
    for (i=0;i<2;i++){
        periodo[i] = 2*pi/omega[i];
        t_mezzi[i] = (log(2)/eta[i])*t_caratteristico;
    }


    //----------------------------------------% STOP-----------------------------------------------
    printf("\n\n Premi qualsiasi tasto per continuare:");
    //Attesa dell'input dell'utente
    //press_any_key();
    //---------------------------------------------------------------------------------------------

    double omega_pass,zeta_pass,periodo_pass,t_mezzi_pass; // ci appoggiamo a valori di passaggio per invertire omega[0] e omega[1]

    omega_pass = omega[0];
    zeta_pass = zeta[0];
    periodo_pass = periodo[0];
    t_mezzi_pass = t_mezzi[0];

    if(omega[0]>omega[1]){
            omega[0] = omega[1];
            zeta[0] = zeta[1];
            periodo[0] = periodo[1];
            t_mezzi[0] = t_mezzi[1];

            omega[1] = omega_pass;
            zeta[1] = zeta_pass;
            periodo[1] = periodo_pass;
            t_mezzi[1] = t_mezzi_pass;
    }

    printf("\n\n\n\n-----------------------------------------------------------------------------------------------------------------------\n");
    printf("   ANALISI DI STABILITA'\n");
    printf("-----------------------------------------------------------------------------------------------------------------------\n");
    //sleep(1);
    //sleep(1);
    printf("\n L'UAV e' staticamente stabile.");
    printf("\n L'UAV e' dinamicamente stabile.\n");
    printf("\n Caratteristiche modi:");
    printf("\n                  ****************************************************************************************** \n");
    printf("                  * - MODO FUGOIDE:                           * - MODO CORTO PERIODO:                      *\n" );
    printf("                  * - omega[rad/s]= %lf                  * - omega[rad/s]= %lf                  *\n",omega[0],omega[1]);
    printf("                  * - smorzamento[-]= %lf                * - smorzamento[-]= %lf                 *\n",zeta[0],zeta[1]);
    printf("                  * - periodo[s]= %lf                    * - periodo[s]= %lf                     *\n",periodo[0],periodo[1]);
    printf("                  * - tempo di dimezzamento[s]= %lf     * - tempo di dimezzamento[s]= %lf       *\n",t_mezzi[0],t_mezzi[1]);
    printf("                  ****************************************************************************************** \n ");



      //APERTURA FILE
    const char *file_path = "OUTPUT/output_STABILITA.txt";
    FILE *file = fopen(file_path, "w");
    if (file == NULL) {
        printf("\nERRORE: -13 - Errore nell'apertura del file di scrittura %s",  "OUTPUT/output_STABILITA.txt" );
        return -13;
    }
    // Scrittura delle variabili nel file
    fprintf(file,"MODO FUGOIDE:\n -omega[rad/s]=%lf\n -smorzamento[-]=%lf\n -periodo[s]=%lf\n -dimezzamento[s]=%lf", omega[1], zeta[1],periodo[1],t_mezzi[1]);
    fprintf(file,"\n\nMODO CORTO PERIODO:\n -omega[rad/s]=%lf\n -smorzamento[-]=%lf\n -periodo[s]=%lf\n -dimezzamento[s]=%lf", omega[0], zeta[0],periodo[0],t_mezzi[0]);

    fclose(file);


    //----------------------------------------% STOP-----------------------------------------------
    printf("\n\n Premi qualsiasi tasto per continuare:");
    //Attesa dell'input dell'utente
    //press_any_key();
    //---------------------------------------------------------------------------------------------
    return 0;
    */
}
