#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "EstrazioneDati.h"
#include "Interpolazione.h"

void main() {
    int trovato = 1, k = 0, mat;

    double *engine = NULL;
    double *geometry_propeller = NULL;
    double *propeller_profile = NULL;
    double **data_propeller = NULL;
    double *body_axes = NULL;
    double *deflection_limits = NULL;
    double *fuel_mass = NULL;
    double **steady_state_coeff = NULL;
    double **aer_der_x = NULL;
    double **aer_der_y = NULL;
    double **aer_der_z = NULL;
    double **rolling_moment_der = NULL;
    double **pitch_moment_der = NULL;
    double **yawing_moment_der = NULL;
    double **control_force_der = NULL;
    double **control_moment_der = NULL;
    double **rotary_der = NULL;

    /*              VALDAZIONE LETTURA DATI                     */
    printf("Inizio lettura dati -> ");
    system("PAUSE");

    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);
    stampaTuttiIDati(engine, geometry_propeller, propeller_profile, data_propeller, body_axes, deflection_limits,
        fuel_mass, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, 
        pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, rotary_der);
    
    printf("\nInizio interpolazione dati -> ");
    system("PAUSE");

    /*              VALDAZIONE INTERPOLAZIONE DATI                     */
    const char *legende[][8] = {
        // steady_state_coeff
        {"ALPHA", "CX", "CY", "CZ", "Cl", "Cm", "Cn", NULL},
        // aer_der_x
        {"ALPHA", "CXA", "CXAP", "CXU", "CXQ", "CXB", "CXP", "CXR"},
        // aer_der_y
        {"ALPHA", "CYB", "CYBP", "CYP", "CYR", "CYA", "CYQ", NULL},
        // aer_der_z
        {"ALPHA", "CZALPHA", "CZAP", "CZU", "CZQ", "CZB", "CZP", "CZR"},
        // rolling_moment_der
        {"ALPHA", "ClB", "ClBP", "ClP", "ClR", "ClA", "ClQ", NULL},
        // pitch_moment_der
        {"ALPHA", "CmA", "CmAP", "CmU", "CmQ", "CmB", "CmP", "CmR"},
        // yawing_moment_der
        {"ALPHA", "CnB", "CnBP", "CnP", "CnR", "CnA", "CnQ", NULL},
        // control_force_der
        {"ALPHA", "CXde", "CXdle", "CZde", "CZdle", "CYda", "CYdr", NULL},
        // control_moment_der
        {"ALPHA", "Clda", "Cldr", "Cmde", "Cmdle", "Cnda", "Cndr", NULL},
        // rotary_der
        {"ALPHA", "CXom", "CYom", "CZom", "Clom", "Cmom", "Cnom", NULL}
    };

    const char *nomi[] = {
        "steady_state_coeff", "aer_der_x", "aer_der_y", "aer_der_z",
        "rolling_moment_der", "pitch_moment_der", "yawing_moment_der",
        "control_force_der", "control_moment_der", "rotary_der"
    };

    double** matrici[] = {
        steady_state_coeff, aer_der_x, aer_der_y, aer_der_z,
        rolling_moment_der, pitch_moment_der, yawing_moment_der,
        control_force_der, control_moment_der, rotary_der
    };

    int numMatrici = sizeof(nomi) / sizeof(nomi[0]);

    printf("\nScegli la matrice da interpolare:\n");
    for (int i = 0; i < numMatrici; ++i) {
        printf("(%d) %s\n", i, nomi[i]);
    }
    int sceltaMat;
    printf("Inserisci il numero della matrice: ");
    do{
        scanf("%d", &sceltaMat);
        if(sceltaMat < 0 || sceltaMat >= numMatrici){
            printf("Scelta non valida. Riprovare: ");
            continue;
        }
        break;
    }while(1);

    printf("\nLegenda colonne per '%s':\n", nomi[sceltaMat]);
    for (int j = 0; j < 8; ++j) {
        if(legende[sceltaMat][j] == NULL){
            break;
        }
        printf("(%d) %s\n", j, legende[sceltaMat][j]);
    }
    int sceltaCol;
    printf("Inserisci il codice della colonna da interpolare: ");
    do{
        scanf("%d", &sceltaCol);
        if(sceltaCol < 0 || sceltaCol >= (legende[sceltaMat][7] == NULL) ? 7 : 8){
            printf("Scelta non valida. Riprovare: ");
            continue;
        }
        break;
    }while(1);

    double alpha;
    printf("Inserisci il valore di alpha per l'interpolazione[-5, 20]: ");
    do{
        scanf("%lf", &alpha);
        if(alpha < -5 || alpha > 20){
            printf("Scelta non valida. Riprovare: ");
            continue;
        }
        break;
    }while(1);

    double InterpVet = interpolazioneTotale(matrici[sceltaMat], sceltaCol, alpha);

    while(matrici[sceltaMat][++k][0] < alpha);
    double x0 = matrici[sceltaMat][k-1][0];
    double x1 = matrici[sceltaMat][k][0];

    printf("\nTest interpolazione su matrice '%s'\n", nomi[sceltaMat]);
    printf("Alpha di riferimento: %g\n", alpha);
    printf("Colonna di riferimento: %s\n", legende[sceltaMat][sceltaCol]);
    printf("Valori di alpha di riferimento: %g, %g \n", x0, x1);

    x0 = matrici[sceltaMat][k-1][sceltaCol];
    x1 = matrici[sceltaMat][k][sceltaCol];

    double min = (x0 < x1) ? x0 : x1;
    double max = (x0 > x1) ? x0 : x1;
    int bar_len = 40;
    int pos = (int)(((InterpVet - min) / (max - min)) * bar_len);
    if (pos < 0) pos = 0;
    if (pos > bar_len) pos = bar_len;

    printf("\nVisualizzazione grafica:\n");
    printf("Estremo1: %g", x0);
    for (int i = 0; i < bar_len + 2; ++i) printf(" ");
    printf("Estremo2: %g\n", x1);

    printf("[");
    for (int i = 0; i < bar_len; ++i) {
        if (i == pos)
            printf("|");
        else
            printf("-");
    }
    printf("]\n");

    for (int i = 0; i < ((pos==0) ? bar_len/2 : pos + 1); ++i) {
        printf(" ");
    }
    printf("^\n");
    printf("\t\tValore interpolato: %g\n", InterpVet);

    printf("Inizio controllo propel -> ");
    system("PAUSE");

    
}