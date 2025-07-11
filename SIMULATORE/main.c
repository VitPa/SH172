#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "Atmosphere/Atmosphere.h"
#include "Error_Warning/ErrorWarning.h"
#include "Error_Warning/InitialCondition.h"
#include "Interpolation/Interpolation.h"
#include "Pre_processing/Data.h"
#include "Pre_processing/Variables.h"
#include "Processing/Command.h"
#include "Processing/Integration.h"
#include "Trim/MotionEq.h"

int main(){    
    double trim[3];

    openFiles();

    printf("Simulatore di volo per il Cessna 172\n Inserire i dati iniziali\n");
    printf("--------------------------------------------\n");
    printf("Per i valori di default premere Invio . . .\n");
    printf("--------------------------------------------\n\n");

    // Load variables from file .txt
    loadData();

    // Load initial conditions
    double CI[3];

    loadCI(CI);
    checkVelAlt(&CI[0], &CI[1], &CI[2]);

    // Compute atmospheric variables
    AtmosphereChoice();
    AtmosphereCalc(CI[1]);

    // Trim condition and Routh stability
    trimEquation(CI, trim);

    // Load commands matrix
    double dt, deltaT_fs;
    printf("Inserire il passo di simulazione:\n(1) -> 0.01s\n(2) -> 0.02s\nScegliere tra [1 e 2]: ");
    do{
        scanf("%lf", &dt);
        if(dt==1) dt = 0.01;
        else if(dt==2) dt = 0.02;
        if(dt != 0.01 && dt != 0.02){
            WARNING(500, 0, 1.0);
        }
    }while(dt != 0.01 && dt != 0.02);
    printf("Inserire il tempo di simulazione [s]: ");
    do{
        scanf("%lf", &deltaT_fs);
        if(deltaT_fs <= 0){
            WARNING(501);
        }
    }while(deltaT_fs <= 0);

    load_command(dt, deltaT_fs, trim[2], trim[1]);
    
    // Integration and simulation
    int i = 0;
    for(double Ts = 0.00; Ts <=deltaT_fs; Ts += dt){

        AtmosphereCalc(state[9]);

        eulerEquation(dt, i);

        physicalCheck(sqrt(pow(state[0], 2) + pow(state[1], 2) + pow(state[2], 2)), state[9], body_axes[0],body_axes[4], vsuono_h);

        fprintf(data, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n",Ts,state[0],state[1],state[2],state[3]*(180/pi),state[4]*(180/pi),state[5]*(180/pi),state[6]*(180/pi),state[7]*(180/pi),state[8]*(180/pi),state[9],state[10],state[11]);
        fflush(data);
        fprintf(com, "%lf\t%lf\t%lf\t%lf\t%lf\t\n", Ts, command[i][0], command[i][1], command[i][2], command[i][3]);
        fflush(com);

        progressBar(Ts, deltaT_fs, "Volo in corso!");
        ++i;
    }
    printf("\n");
    printf("Massa finale %g\n", body_axes[0]);

    // Free dynamic memory
    freeData();

    // DA ELIMINARE NEL CODICE FINALE
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\SIMULATORE\\_output_files\\DATA.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\DATA.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\SIMULATORE\\_output_files\\COMMAND.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\COMMAND.txt\"");
    system("copy /Y \"C:\\Users\\vitop\\OneDrive - Politecnico di Torino\\Computer\\Universita\\PoliTo\\2 anno\\Mod-Sim\\Simulazione\\Progetti\\Simulatore\\FILE_PROGETTO\\GIT\\SIMULATORE\\_output_files\\EXTRA.txt\" \"C:\\Users\\vitop\\Downloads\\CODICE_GRUPPO\\CODICE_GRUPPO\\EXTRA.txt\"");

    closeFiles();
    printf("SIMULAZIONE TERMINATA. GRAZIE PER AVER VOLATO CON NOI, \tA PRESTO!\n");
    system("PAUSE");
    return 0;
}