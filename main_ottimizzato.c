// main_ottimizzato.c
// Versione ottimizzata del main per il simulatore Cessna 172
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Atmosphere.h"
#include "EstrazioneDati.h"
#include "Interpolazione.h"
#include "MotionEq.h"

#define N_ALPHA 126

// Funzione per acquisire i dati iniziali dall'utente
void acquisisciInputIniziale(double *CI) {
    printf("\nSimulatore di volo per il Cessna 172\nInserire i dati iniziali\n--------------------------------------------\n\n");
    printf("Inserire la velocità iniziale: ");
    scanf("%lf", &CI[0]);
    printf("Inserire l'altitudine iniziale: ");
    scanf("%lf", &CI[1]);
    printf("Inserire l'angolo di attacco iniziale: ");
    scanf("%lf", &CI[2]);
    printf("\n");
}

int main() {
    // Dati principali
    double CI[3];
    acquisisciInputIniziale(CI);

    // Puntatori ai dati
    double *engine = NULL, *geometry_propeller = NULL, *propeller_profile = NULL;
    double **data_propeller = NULL, *body_axes = NULL, *deflection_limits = NULL, *fuel_mass = NULL;
    double **steady_state_coeff = NULL, **aer_der_x = NULL, **aer_der_y = NULL, **aer_der_z = NULL;
    double **rolling_moment_der = NULL, **pitch_moment_der = NULL, **yawing_moment_der = NULL;
    double **control_force_der = NULL, **control_moment_der = NULL, **rotary_der = NULL;

    // Caricamento dati da file
    if (!caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits, &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der)) {
        fprintf(stderr, "Errore nel caricamento dei dati.\n");
        return 1;
    }

    // Atmosfera
    double Pmax_h = 0, press0 = 0, temp0 = 0, rho0 = 0, vsuono0 = 0, press_h = 0, temp_h = 0, rho_h = 0, vsuono_h = 0;
    int flagatm;
    AtmosphereChoice(&press0, &temp0, &rho0, &vsuono0, &press_h, &temp_h, &rho_h, &vsuono_h, CI, &flagatm);
    AtmosphereCalc(CI, &engine, &Pmax_h, &press0, &temp0, &rho0, &vsuono0, &press_h, &temp_h, &rho_h, &vsuono_h, &flagatm);

    // Esecuzione equazione del moto (esempio)
    equation(5.0, 1.1116, CI, body_axes, aer_der_x, aer_der_y, aer_der_z, steady_state_coeff, control_force_der, control_moment_der, rotary_der);

    // Libera la memoria
    liberaTuttiIDati(engine, geometry_propeller, propeller_profile, data_propeller, body_axes, deflection_limits, fuel_mass, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, rotary_der);

    return 0;
}
