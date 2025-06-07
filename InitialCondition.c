#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Atmosphere.h"

void checkVelAlt(double *V, double h, double Mdg, double temp_h) {
    if (*V < 30){
        printf("[~]WARNING: La velocità inserita è inferiore di quella di stallo.\tInserita velocità minima: 30 m/s\n");
        *V = 30;
    }
    if (*V > 75){
        printf("[~]WARNING: La velocità inserita è maggiore di quella massima.\tInserita velocità massima: 75 m/s\n");
        *V = 75;
    }
    if (*V/(sqrt(1.4 * 287.05 * temp_h)) > Mdg){
        printf("[!]ERROR: La velocità è maggiore del match di drag rise.\nSIMULAZIONE TERMINATA!");
        system("PAUSE");
        exit(0);
    }
    if(h<0){
        printf("[!]ERROR: I valori inseriti corrispondono ad una quota minore di zero.\nSIMULAZIONE TERMINATA!");
        system("PAUSE");
        exit(0);
    }
    else if(h>4116){
        printf("[!]ERROR: I valori inseriti corrispondono ad una quota maggiore di quella di tangenza.\nSIMULAZIONE TERMINATA!\n");
        system("PAUSE");
        exit(0);
    }
}

void loadCI(double *CI) {
    char input[100];
    printf("\n\nSimulatore di volo per il Cessna 172\n Inserire i dati iniziali\n");
    printf("--------------------------------------------\n");
    printf("Per i valori di default premere Invio . . .\n");
    printf("--------------------------------------------\n\n");
    
    printf("Inserire la velocità inziale [m/s]: ");
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
    printf("Inserire l'altitudine inziale [m]: ");
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
    printf("Inserire l'angolo di attacco inziale [deg]: ");
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