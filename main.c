#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "Atmosphere.h"
#include "EstrazioneDati.h"
#include "Interpolazione.h"
#include "MotionEq.h"
#include "Integration.h"
#include "InitialCondition.h"
#include "Command.h"

int main(){

    /*
    Implementare all'inzio un parte di codice che fa decidere (oppure si passa come argomento) 
    se far partire il simulatore in modalità utente o modalità test. Quindi verbosa o no. In più 
    si può fare una terza modalità in cui si stampano (magari chiedendo funzione per funzione) 
    tutti i calcoli intermedi per un debug ancora più avanzato
    */
    /*
    Implementae una struct o una nuova funzione per la gestione degli errori. Ti deve permettere di
    fare (system("PAUSE"); exit(0);) se è un'errore oppure gestire il warning, magari passando il 
    messaggio da stampare e distinguere tra warning silenzioso o warning non silenzioso (cioè se 
    l'utente deve intervenire oppure no).
    In ogni caso tutti i warning ed errori li deve stampare anche su un file di log
    */
    /*
    Inserire la scritta "Premere invio per i messaggi di default" in tutti i messaggi per le condizioni
    */

    double *engine = NULL;
    double *geometry_propeller = NULL, *propeller_profile = NULL, **data_propeller = NULL;
    double *body_axes = NULL, *deflection_limits = NULL, *fuel_mass = NULL;
    double **steady_state_coeff = NULL, **aer_der_x = NULL, **aer_der_y = NULL,**aer_der_z = NULL;
    double **rolling_moment_der = NULL, **pitch_moment_der = NULL, **yawing_moment_der = NULL;
    double **control_force_der = NULL, **control_moment_der = NULL, **rotary_der = NULL;
    double **state = NULL;
    double trim[3];
    double **command = NULL;

    // Load variables from file .txt
    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    // Load initial conditions
    int flagatm;
    double Pmax_h, press_h, temp_h, rho_h, vsuono_h;
    double CI[3];

    loadCI(CI);
    checkVelAlt(&CI[0], &CI[1], &CI[2]);

    // Compute atmospheric variables
    AtmosphereChoice(&press_h, &temp_h, &rho_h, &vsuono_h, &flagatm);
    AtmosphereCalc(CI[1], engine, &Pmax_h, &press_h, &temp_h, &rho_h, &vsuono_h, flagatm);

    // Trim condition and Routh stability
    equation(engine, Pmax_h, rho_h, CI, &state, body_axes, aer_der_x, aer_der_z, steady_state_coeff, control_force_der, 
        control_moment_der, pitch_moment_der, geometry_propeller, propeller_profile, data_propeller, trim);

    // Load commands matrix
    double dt = 0.01, deltaT_fs;
    printf("Inserire il tempo di simulazione: ");
    scanf("%lf", &deltaT_fs);

    command = load_command(dt, deltaT_fs, trim[2], trim[1]);
    
    // Integration and simulation
    int i = 0;
    FILE *fp = fopen("DATI_ANALISI.txt", "w");
    if (fp == NULL) {
        printf("Errore nell'apertura del file DATI_ANALISI.txt\n");
        return 1;
    }
    FILE *cm = fopen("DATI_COMANDI.txt", "w");
    if (cm == NULL) {
        printf("Errore nell'apertura del file DATI_ANALISI.txt\n");
        return 1;
    }
    FILE *f = fopen("DATI_AGGIUNTIVI.txt", "w");
    if (f) fclose(f);
    for(double Ts = 0.00; Ts <=deltaT_fs; Ts += dt){
        AtmosphereCalc(state[i][9], engine, &Pmax_h, &press_h, &temp_h, &rho_h, &vsuono_h, flagatm);

        state = reallocState(state, 12);

        if (eulerEquation(dt, i, state, command, Pmax_h, rho_h, engine, body_axes, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, 
            rolling_moment_der, pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, geometry_propeller, 
            propeller_profile, data_propeller)){
                break;
            }

        physicalCheck(fabs(sqrt(pow(state[i][0], 2)+pow(state[i][1], 2)+pow(state[i][2], 2))), state[i][9], body_axes[4], vsuono_h);
        
        //printf("%.2lf", Ts);

        /*for(int j = 0; j < 12; j++){
            printf("   -   %.4lf", state[i][j]);
        }*/
        //printf("\n");

        fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n",Ts,state[i][0],state[i][1],state[i][2],state[i][3],state[i][4],state[i][5],state[i][6],state[i][7],state[i][8],state[i][9],state[i][10],state[i][11]);
        fprintf(cm, "%lf\t%lf\t%lf\t%lf\t%lf\t\n", Ts, command[i][0], command[i][1], command[i][2], command[i][3]);
        ++i;
    }
    fclose(fp);
    fclose(cm);

    // Free dynamic memory
    liberaTuttiIDati(engine, geometry_propeller, propeller_profile, data_propeller, body_axes, deflection_limits, 
        fuel_mass, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, pitch_moment_der, 
        yawing_moment_der, control_force_der, control_moment_der, rotary_der, state, command);

    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_AGGIUNTIVI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_AGGIUNTIVI.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_ANALISI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_ANALISI.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_COMANDI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_COMANDI.txt\"");

    printf("SIMULAZIONE TERMINATA. GRAZIE PER AVER VOLATO CON NOI, \tA PRESTO!\n");
    system("PAUSE");
    return 0;
}