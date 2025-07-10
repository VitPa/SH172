#include <stdio.h>
#include <stdlib.h>
#include "EstrazioneDati.h"
#include "Interpolazione.h"
#include "propeller.h"
#include "ErrorWarning.h"

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
    
    printf("\n- Valori correttamente scritti nel file \"VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt\"\n\n");

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
            WARNING(500, 0, (double)numMatrici);
            continue;
        }
        break;
    }while(1);

    printf("\nLegenda colonne per '%s':\n", nomi[sceltaMat]);
    for (int j = 0; j < 8; ++j) {
        if(legende[sceltaMat][j] == NULL) {
            break;
        }
        printf("(%d) %s %s\n", j, legende[sceltaMat][j], matrici[sceltaMat][0][j]==matrici[sceltaMat][10][j] ? 
            matrici[sceltaMat][0][j]==0 ? "\t(costanti a zero)" : "\t(costanti)" : "");
    }
    int sceltaCol;
    printf("\nInserisci il codice della colonna da interpolare: ");
    do{
        scanf("%d", &sceltaCol);
        if(sceltaCol < 0 || sceltaCol > ((legende[sceltaMat][7] == NULL) ? 7 : 8)) {
            WARNING(500, 0, ((legende[sceltaMat][7] == NULL) ? 7.0 : 8.0));
            continue;
        }
        break;
    }while(1);

    double alpha;
    printf("Inserisci il valore di alpha per l'interpolazione[-5, 20]: ");
    do{
        scanf("%lf", &alpha);
        if(alpha < -5 || alpha > 20){
            WARNING(500, -5.0, 20.0);
            continue;
        }
        break;
    }while(1);

    double InterpVet = interpolazioneTotale(matrici[sceltaMat], sceltaCol, alpha);

    while(matrici[sceltaMat][++k][0] < alpha){};
    double x0 = matrici[sceltaMat][k-1][0];
    double x1 = matrici[sceltaMat][k][0];

    FILE *val1 = apriFile("Validazione_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt", "a");
    fprintf(val1, "\n------------VALIDAZIONE_INTERPOLAZIONE------------\n");

    fprintf(val1, "\nTest interpolazione su matrice '%s'\n", nomi[sceltaMat]);
    fprintf(val1, "Alpha di riferimento: %g\n", alpha);
    fprintf(val1, "Colonna di riferimento: %s\n", legende[sceltaMat][sceltaCol]);
    fprintf(val1, "Valori di alpha di riferimento: %g, %g \n", x0, x1);

    x0 = matrici[sceltaMat][k-1][sceltaCol];
    x1 = matrici[sceltaMat][k][sceltaCol];

    double min = (x0 < x1) ? x0 : x1;
    double max = (x0 > x1) ? x0 : x1;
    int bar_len = 40;
    int pos = (int)(((InterpVet - min) / (max - min)) * bar_len);
    if (pos < 0) pos = 0;
    if (pos > bar_len) pos = bar_len;

    fprintf(val1, "\nVisualizzazione grafica:\n");
    fprintf(val1, "Estremo1: %g", x0);
    for (int i = 0; i < bar_len + 2; ++i) fprintf(val1, " ");
    fprintf(val1, "Estremo2: %g\n", x1);
    fprintf(val1, "[");
    for (int i = 0; i < bar_len; ++i) {
        if (i == pos)
            fprintf(val1, "|");
        else
            fprintf(val1, "-");
    }
    fprintf(val1, "]\n");
    for (int i = 0; i < ((pos==0) ? bar_len/2 : pos + 1); ++i) {
        fprintf(val1, " ");
    }
    fprintf(val1, "^\n");
    fprintf(val1, "\t\tValore interpolato: %g\n", InterpVet);

    fclose(val1);

    printf("\n- Valori correttamente scritti nel file \"VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt\"\n\n");

    printf("Validazione terminata con successo!\n");
    system("PAUSE");
}