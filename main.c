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
#include "ErrorWarning.h"
#include "Variables.h"
#include "rk4.h"

int main(){
    /*
    In ogni caso tutti i warning ed errori li deve stampare anche su un file di log
    */
    ew_log = apriFile("log.txt", "w");
    
    double trim[3];

    printf("\n\nSimulatore di volo per il Cessna 172\n Inserire i dati iniziali\n");
    printf("--------------------------------------------\n");
    printf("Per i valori di default premere Invio . . .\n");
    printf("--------------------------------------------\n\n");

    // Load variables from file .txt
    caricaTuttiIDati();

    RPMmin = (int)engine[2];
    RPMmax = (int)engine[3];

    // Load initial conditions
    double CI[3];

    loadCI(CI);
    checkVelAlt(&CI[0], &CI[1], &CI[2]);

    // Compute atmospheric variables
    AtmosphereChoice();
    AtmosphereCalc(CI[1]);

    // Trim condition and Routh stability
    equation(CI, trim);

    // Load commands matrix
    double dt = 0.01, deltaT_fs;
    printf("Inserire il tempo di simulazione [s]: ");
    do{
        scanf("%lf", &deltaT_fs);
        if(deltaT_fs <= 0){
            WARNING(501);
        }
    }while(deltaT_fs <= 0);

    command = load_command(dt, deltaT_fs, trim[2], trim[1]);
    
    // Integration and simulation
    int i = 0;
    FILE *fp = apriFile("DATI_ANALISI.txt", "w");
    FILE *cm = apriFile("DATI_COMANDI.txt", "w");
    FILE *f = apriFile("DATI_AGGIUNTIVI.txt", "w");
    if (f) fclose(f);
    for(double Ts = 0.00; Ts <=deltaT_fs; Ts += dt){

        //AtmosphereCalc(state[9]);

        //state = reallocState(state, 12);

        eulerEquation(dt, i);
        //rk4_step(state, 12, dt, Ts, i, compute_derivatives);

        physicalCheck(sqrt(pow(state[0], 2) + pow(state[1], 2) + pow(state[2], 2)), state[9], body_axes[0],body_axes[4], vsuono_h);
        
        //printf("%.2lf", Ts);
        /*for(int j = 0; j < 12; j++){
            printf("   -   %.4lf", state[i][j]);
        }*/
        //printf("\n");

        fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n",Ts,state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7],state[8],state[9],state[10],state[11]);
        fprintf(cm, "%lf\t%lf\t%lf\t%lf\t%lf\t\n", Ts, command[i][0], command[i][1], command[i][2], command[i][3]);

        progressBar(Ts, deltaT_fs);
        ++i;
    }
    printf("\n");
    fclose(fp);
    fclose(cm);

    // Free dynamic memory
    liberaTuttiIDati();

    // DA ELIMINARE NEL CODICE FINALE
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_AGGIUNTIVI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_AGGIUNTIVI.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_ANALISI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_ANALISI.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_COMANDI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_COMANDI.txt\"");

    fclose(ew_log);
    printf("SIMULAZIONE TERMINATA. GRAZIE PER AVER VOLATO CON NOI, \tA PRESTO!\n");
    system("PAUSE");
    return 0;
}