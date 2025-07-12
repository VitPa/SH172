#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include "ErrorWarning.h"
#include "../Pre_processing/Variables.h"
#include "../Pre_processing/Data.h"

static double mFuelMin = -1.0;

void endSection(){
    printf("                                                             =");
    Sleep(1000);
    printf("=");
    Sleep(1000);
    printf(">>");
    Sleep(1000);
}

void startSection(int option){
    system("cls");
    printf("\n>>>-----------------------------------------------------------------<<<\n");
    switch(option){
        case 1:
            printf(">     [ PRE-PROCESSING ]  >>  Inserimento condizioni iniziali ...     <\n");
            printf(">                    --------------------------                       <\n");
            printf(">             Per i valori di default premere Invio . . .             <\n");
            break;
        case 2:
            printf(">             [ PRE-PROCESSING ]  >>  Atmosfera ISA ...               <\n");
            break;
        case 3:
            printf(">       [ PRE-PROCESSING ]  >>  Calcolo condizioni di Trim ...        <\n");
            break;
        case 4:
            printf(">    [ PRE-PROCESSING ]  >>  Calcolo condizioni di stabilita' ...     <\n");
            break;
        case 5:
            printf(">               [ PRE-PROCESSING ]  >>  Scelta Manovra ...            <\n");
            break;
        case 6:
            printf(">           [ PROCESSING ]  >>  Integrazione e Simulazione ...        <\n");
            break;
    }
    printf(">>>-----------------------------------------------------------------<<<\n\n");
}

void checkVelAlt(double *V, double *h, double *gamma) {
    if (*V < 30){
        WARNING(200, 30.0);
        *V = 30;
    }
    else if (*V > 75){
        WARNING(202, 75.0);
        *V = 75;
    }
    if (*h<0){
        WARNING(203, 0);
        *h = 0;
    }
    else if(*h>4116){
        WARNING(204, 4116.0);
        *h = 4116;
    }
    if (*gamma<-5){
        WARNING(205, -5.0);
        *gamma = -5;
    }
    if (*gamma>10){
        WARNING(206, 10.0);
        *gamma = 10;
    }
}

void physicalCheck(double V, double h, double m, double Mdg, double vsuono_h) {
    enum {COND_VMIN, COND_MACH, COND_HMIN, COND_HMAX, COND_MASS, N_COND};
    static int counters[N_COND] = {0};
    const int threshold = 3;

    if(mFuelMin < 0) mFuelMin = body_axes[0] * (1 - fuel_mass[1]);              // Compute minimum fuel mass only once

    int triggered[N_COND] = {
        V < 30,                 // Airspeed below minimum
        (V/vsuono_h) > Mdg,     // Mach number exceeds limit
        h < 0,                  // Altitude below ground
        h > 4116,               // Altitude above maximum
        m < mFuelMin            // Mass below minimum allowed (fuel exhausted)
    };

    int error_codes[N_COND] = {200, 201, 203, 204, 205};                        // Error codes for each condition

    for (int i = 0; i < N_COND; ++i) {                                          // For each condition, increment counter if triggered, reset if not
        if (triggered[i]) {
            if (++counters[i] > threshold) MY_ERROR(error_codes[i]);            // If the violation persists for more than 'threshold' cycles, raise error
        } else if (counters[i] > 0) {
            --counters[i];
        }
    }
}

void loadCI(double *CI) {
    char input[100];
    
    startSection(1);

    printf("Inserire la velocità inziale [m/s] (default: 52): ");
    do{
        fgets(input, 100, stdin);
        if(input[0]=='\n') {
            CI[0] = 52; 
            WARNING(0, 52.0); 
            break;
        }
        if(sscanf(input,"%lf",&CI[0])!=0) break;
        WARNING(504);
    }while(1);
    printf("\nInserire l'altitudine inziale [m] (default: 1000): ");
    do{
        fgets(input, 100, stdin);
        if(input[0]=='\n') {
            CI[1] = 1000; 
            WARNING(0, 1000.0); 
            break;
        }
        if(sscanf(input,"%lf",&CI[1])!=0) break;
        WARNING(504);
    }while(1);
    printf("\nInserire l'angolo di attacco inziale [deg] (default: 0): ");
    do{
        fgets(input, 100, stdin);
        if(input[0]=='\n') {
            CI[2] = 0; 
            WARNING(0, 0); 
            break;
        }
        if(sscanf(input,"%lf",&CI[2])!=0) break;
        WARNING(504);
    }while(1);
    printf("\n");

    endSection();
}

void openFiles(){
    ew_log = openFile(path_log, "w");
    data = openFile(path_data, "w");
    com = openFile(path_com, "w");
    agg = openFile(path_agg, "w");
}
void closeFiles(){
    fclose(ew_log);
    fclose(data);
    fclose(com);
    fclose(agg);
}