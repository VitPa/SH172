#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "../Atmosphere/Atmosphere.h"
#include "../Error_Warning/ErrorWarning.h"
#include "../Error_Warning/InitialCondition.h"
#include "../Interpolation/Interpolation.h"
#include "../Pre_processing/EstrazioneDati.h"
#include "../Pre_processing/Variables.h"
#include "../Processing/Command.h"
#include "../Processing/Integration.h"
#include "../Trim/MotionEq.h"

int main(){    
    double trim[3];

    openFiles();

    printf("Simulatore di volo per il Cessna 172\n Inserire i dati iniziali\n");
    printf("--------------------------------------------\n");
    printf("Per i valori di default premere Invio . . .\n");
    printf("--------------------------------------------\n\n");

    // Load variables from file .txt
    caricaTuttiIDati();

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
    for(double Ts = 0.00; Ts <=deltaT_fs; Ts += dt){

        AtmosphereCalc(state[9]);

        eulerEquation(dt, i);

        physicalCheck(sqrt(pow(state[0], 2) + pow(state[1], 2) + pow(state[2], 2)), state[9], body_axes[0],body_axes[4], vsuono_h);

        fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n",Ts,state[0],state[1],state[2],state[3]*(180/pi),state[4]*(180/pi),state[5]*(180/pi),state[6]*(180/pi),state[7]*(180/pi),state[8]*(180/pi),state[9],state[10],state[11]);
        fflush(fp);
        fprintf(cm, "%lf\t%lf\t%lf\t%lf\t%lf\t\n", Ts, command[i][0], command[i][1], command[i][2], command[i][3]);
        fflush(cm);

        progressBar(Ts, deltaT_fs);
        ++i;
    }
    printf("\n");

    // Free dynamic memory
    liberaTuttiIDati();

    // DA ELIMINARE NEL CODICE FINALE
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_AGGIUNTIVI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_AGGIUNTIVI.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_ANALISI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_ANALISI.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\DATI_COMANDI.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATI_COMANDI.txt\"");

    closeFiles();
    printf("SIMULAZIONE TERMINATA. GRAZIE PER AVER VOLATO CON NOI, \tA PRESTO!\n");
    system("PAUSE");
    return 0;
}