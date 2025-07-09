#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ErrorWarning.h"

void checkVelAlt(double *V, double *h, double *gamma) {
    if (*V < 30){
        printf("[~]WARNING: La velocità inserita è inferiore di quella di stallo.\tInserita velocità minima: 30 m/s\n");
        *V = 30;
    }
    else if (*V > 75){
        printf("[~]WARNING: La velocità inserita è maggiore di quella massima.\tInserita velocità massima: 75 m/s\n");
        *V = 75;
    }
    if (*h<0){
        printf("[~]WARNING: I valori inseriti corrispondono ad una quota minore di zero.\nInserita quota minima: 0 m\n");
        *h = 0;
    }
    else if(*h>4116){
        printf("[~]WARNING: I valori inseriti corrispondono ad una quota maggiore di quella di tangenza.\nInserita quota massima: 4116 m\n");
        *h = 4116;
    }
    if (*gamma<-5){
        printf("[~]WARNING: L'angolo di rampa è inferiore di quello minimo consentito.\tInserito angolo di rampa minimo: -5 deg\n");
        *gamma = -5;
    }
    if (*gamma>10){
        printf("[~]WARNING: L'angolo di rampa è maggiore di quello massimo consentito.\tInserito angolo di rampa massimo: 10 deg\n");
        *gamma = 10;
    }
}

void physicalCheck(double V, double h, double m, double Mdg, double vsuono_h) {
    enum {COND_VMIN, COND_MACH, COND_VMAX, COND_HMIN, COND_HMAX, COND_MASS, N_COND};
    static int counters[N_COND] = {0};
    const int threshold = 3;
    double mFuelMin; //Eliminare quando si mette il controllo della massa nel main

    int triggered[N_COND] = {
        V < 30,
        V/vsuono_h > Mdg,
        V > 75,
        h < 0,
        h > 4116,
        m < mFuelMin
    };

    int error_codes[N_COND] = {200, 201, 202, 203, 204};

    for (int i = 0; i < N_COND; ++i) {
        if (triggered[i]) {
            if (++counters[i] > threshold) Error(error_codes[i], NULL);
        } else if (counters[i] > 0) {
            --counters[i];
        }
    }
}

void loadCI(double *CI) {
    char input[100];
    printf("\n\nSimulatore di volo per il Cessna 172\n Inserire i dati iniziali\n");
    printf("--------------------------------------------\n");
    printf("Per i valori di default premere Invio . . .\n");
    printf("--------------------------------------------\n\n");
    
    printf("Inserire la velocità inziale [m/s] (default: 52): ");
    do{
        fgets(input, 100, stdin);
        if(input[0]=='\n') {
            CI[0] = 52; 
            printf("[-]DEFAULT: Valore di default inserito: 52 m/s\n"); 
            break;
        }
        if(sscanf(input,"%lf",&CI[0])!=0) break;
        printf("[~]WARNING: Immettere un valore numerico --> ");
    }while(1);
    printf("\nInserire l'altitudine inziale [m] (default: 1000): ");
    do{
        fgets(input, 100, stdin);
        if(input[0]=='\n') {
            CI[1] = 1000; 
            printf("[-]DEFAULT: Valore di default inserito: 1000 m\n"); 
            break;
        }
        if(sscanf(input,"%lf",&CI[1])!=0) break;
        printf("[~]WARNING: Immettere un valore numerico --> ");
    }while(1);
    printf("\nInserire l'angolo di attacco inziale [deg] (default: 0): ");
    do{
        fgets(input, 100, stdin);
        if(input[0]=='\n') {
            CI[2] = 0; 
            printf("[-]DEFAULT: Valore di default inserito: 0 deg\n"); 
            break;
        }
        if(sscanf(input,"%lf",&CI[2])!=0) break;
        printf("[~]WARNING: Immettere un valore numerico --> ");
    }while(1);
    printf("\n");
}