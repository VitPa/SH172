#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "EstrazioneDati.h"
#include "Interpolazione.h"

void main() {
    int trovato = 1, k = 0;

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

    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);
    stampaTuttiIDati(engine, geometry_propeller, propeller_profile, data_propeller, body_axes, deflection_limits,
        fuel_mass, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, 
        pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, rotary_der);

    double** matrici[] = {
    steady_state_coeff, aer_der_x, aer_der_y, aer_der_z,
    rolling_moment_der, pitch_moment_der, yawing_moment_der,
    control_force_der, control_moment_der, rotary_der
    };
    const char *nomi[] = {
        "steady_state_coeff", "aer_der_x", "aer_der_y", "aer_der_z",
        "rolling_moment_der", "pitch_moment_der", "yawing_moment_der",
        "control_force_der", "control_moment_der", "rotary_der"
    };

    srand(time(NULL));
    double alpha_rand = -5.0 + 25.0 * ((double)rand() / RAND_MAX);
    int i =(rand() % sizeof(matrici) / sizeof(matrici[0]));
    if(alpha_rand < -5) alpha_rand = -5;
    else if(alpha_rand > 20) alpha_rand = 20;
    while(matrici[i][++k][0] < alpha_rand);

    int colonna = (i==2 || i==4 || i==6) ? rand()%8 : rand()%7;
    colonna = 0;

    double InterpVet = interpolazioneTotale(matrici[i], colonna, alpha_rand);

    double x0 = matrici[i][k-1][0];
    double x1 = matrici[i][k][0];

    // Stampa formattata e grafica
    printf("\nTest interpolazione su matrice '%s':\n", nomi[i]);
    printf("Alpha di riferimento (random): %g\n", alpha_rand);
    printf("Colonna di riferimento (random): %d\n", colonna);
    printf("Valori di alpha di riferimento: %g (riga %d), %g (riga %d)\n", x0, k, x1, k+1);

    x0 = matrici[i][k-1][colonna];
    x1 = matrici[i][k][colonna];

    // Visualizzazione grafica
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

    system("PAUSE");
}