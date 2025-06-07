#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Atmosphere.h"

void checkMdgAlt(double V, double h, double Mdg, double temp_h) {
    if (V/(sqrt(1.4 * 287.05 * temp_h)) > Mdg){
        printf("Warning: il match è maggiore del match di drag rise, inserire un valore di velocità inferiore.\n");
        system("PAUSE");
        exit(0);
    }
    if(h<0)
    {
        printf("[!]ERRORE: I valori inseriti corrispondono ad una quota minore di zero\nSIMULAZIONE TERMINATA!");
        system("PAUSE");
        exit(14);
    }
    else if(h>4116)
    {
        printf("[!]ERRORE: I valori inseriti corrispondono ad una quota maggiore di quella di tangenza\nSIMULAZIONE TERMINATA!\n");
        system("PAUSE");
        exit(14);
    }
}

void loadCI(double *CI) {
    printf("\n\nSimulatore di volo per il Cessna 172\n Inserire i dati iniziali\n");
    printf("--------------------------------------------\n\n");
    
    printf("Inserire la velocità inziale: ");
    scanf("%lf", &CI[0]);
    printf("Inserire l'altitudine inziale: ");
    scanf("%lf", &CI[1]);
    printf("Inserire l'angolo di attacco inziale: ");
    scanf("%lf", &CI[2]);
    printf("\n");
}