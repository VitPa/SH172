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
    double trim[3] = {0.0, 0.0, 0.0};

    // *** Section: Open output/input files ***
    openFiles();
    
    printf("Simulatore di volo per il Cessna 172\n\t\tGruppo 01\n\t    A.A. (24-25)\n");
    printf("Casali Filippo   -   Dimola Alessio\n");
    printf("Godoy Gabriel    -   Guarino Enrica\n");
    printf("Piazzolla Vito   -   Ruggieri Elena\n");
    printf("Scarso Giovanni\n\n");

    system("PAUSE");

    // *** Section: Load data from file ***
    loadData();

    // *** Section: Load and check initial conditions ***
    double CI[3];
    loadCI(CI);
    checkVelAlt(&CI[0], &CI[1], &CI[2]);

    // *** Section: Compute atmospheric variables at initial altitude ***
    AtmosphereChoice();
    AtmosphereCalc(CI[1]);

    // *** Section: Compute trim condition and check stability ***
    trimEquation(CI, trim);

    // *** Section: Maneuver selection and simulation setup ***
    startSection(5);
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
    printf("\nInserire il tempo di simulazione [s]: ");
    do{
        scanf("%lf", &deltaT_fs);
        if(deltaT_fs <= 0){
            WARNING(501);
        }
    }while(deltaT_fs <= 0);

    load_command(dt, deltaT_fs, trim[2], trim[1]);          // Load the command matrix for the simulation

    
    // *** Section: Integration and simulation loop ***
    startSection(6);

    int i = 0;
    for(double Ts = 0.00; Ts <=deltaT_fs; Ts += dt){

        AtmosphereCalc(state[9]);                   // Update atmospheric variables at current altitude

        eulerEquation(dt, i);                       // Integrate equations of motion for this time step

        // Check physical constraints (e.g., velocity, altitude, etc.)
        physicalCheck(sqrt(pow(state[0], 2) + pow(state[1], 2) + pow(state[2], 2)), state[9], body_axes[0],body_axes[4], vsuono_h);

        // Write data to output files
        fprintf(data, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n",Ts,state[0],state[1],state[2],state[3]*(180/pi),state[4]*(180/pi),state[5]*(180/pi),state[6]*(180/pi),state[7]*(180/pi),state[8]*(180/pi),state[9],state[10],state[11]);
        fflush(data);
        fprintf(com, "%lf\t%lf\t%lf\t%lf\t%lf\t\n", Ts, command[i][0], command[i][1], command[i][2], command[i][3]);
        fflush(com);

        progressBar(Ts, deltaT_fs, "Volo in corso!");
        ++i;
    }

    // *** Section: Free memory and close files ***
    freeData();

    closeFiles();
    system("cls");
    printf("\nSIMULAZIONE TERMINATA. GRAZIE PER AVER VOLATO CON NOI, \tA PRESTO!\n");
    system("PAUSE");
    return 0;
}