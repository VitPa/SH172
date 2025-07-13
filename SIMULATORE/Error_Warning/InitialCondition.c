#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include "ErrorWarning.h"
#include "../Pre_processing/Variables.h"
#include "../Pre_processing/Data.h"

static double mFuelMin = -1.0;
static double CI[3];
static double trim[3];
static int hop = 0;

void endSection(double *opt){
    if(opt!=NULL && hop==0){CI[0] = opt[0]; CI[1] = opt[1]; CI[2] = opt[2]; hop = 1;}
    if(opt!=NULL && hop==1){trim[0] = opt[0]; trim[1] = opt[1]; trim[2] = (opt[2] - RPMmin) / (RPMmax - RPMmin);}
    
    printf("\n                                                             =");
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
            printf(">                    --------------------------                       <\n");
            printf("       V = %.2lf m/s  -  h = %.2lf m  -  gamma = %.2lf deg           \n", CI[0], CI[1], CI[2]);
            break;
        case 3:
            printf(">       [ PRE-PROCESSING ]  >>  Calcolo condizioni di Trim ...        <\n");
            printf(">                    --------------------------                       <\n");
            printf("         V = %.2lf m/s  -  h = %.2lf m  -  gamma = %.2lf deg           \n", CI[0], CI[1], CI[2]);
            break;
        case 4:
            printf(">    [ PRE-PROCESSING ]  >>  Calcolo condizioni di stabilita' ...     <\n");
            printf(">                    --------------------------                       <\n");
            printf("         V = %.2lf m/s  -  h = %.2lf m  -  gamma = %.2lf deg           \n", CI[0], CI[1], CI[2]);
            break;
        case 5:
            printf(">               [ PRE-PROCESSING ]  >>  Scelta Manovra ...            <\n");
            printf(">                    --------------------------                       <\n");
            printf("         V = %.2lf m/s  -  h = %.2lf m  -  gamma = %.2lf deg           \n", CI[0], CI[1], CI[2]);
            printf(">                    --------------------------                       <\n");
            printf("    alphaTrim = %.2lf deg - deTrim = %.2lf deg  -  manettaTrim = %.2lf \n", trim[0], trim[1], trim[2]);
            break;
        case 6:
            printf(">           [ PROCESSING ]  >>  Integrazione e Simulazione ...        <\n");
            break;
    }
    printf(">>>-----------------------------------------------------------------<<<\n\n");
}

void checkVelAlt(double *V, double *h, double *gamma) {
    if (*V < Vmin){
        WARNING(200, Vmin);
        *V = Vmin;
    }
    else if (*V > Vmax){
        WARNING(202, Vmax);
        *V = Vmax;
    }
    if (*h<Hmin){
        WARNING(203, Hmin);
        *h = Hmin;
    }
    else if(*h>Hmax){
        WARNING(204, Hmax);
        *h = Hmax;
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
        V < Vmin,                 // Airspeed below minimum
        (V/vsuono_h) > Mdg,     // Mach number exceeds limit
        h < Hmin,                  // Altitude below ground
        h > Hmax,               // Altitude above maximum
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