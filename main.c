// Provare ad usare gnuplot per i grafici



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "Atmosphere.h"
#include "EstrazioneDati.h"
#include "Interpolazione.h"
#include "MotionEq.h"

int main(){

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
    double Int_ssc[6], Int_adx[7], Int_ady[6], Int_adz[7], Int_rmd[6], Int_pmd[7], Int_ymd[6], Int_cfd[6], Int_cmd[6], Int_rd[6];
    double CI[3];
    double Pmax_h = 0, press0 = 0, temp0 = 0, rho0 = 0, vsuono0 = 0, press_h = 0, temp_h = 0, rho_h = 0, vsuono_h = 0;
    int flagatm;

    printf("\n\nSimulatore di volo per il Cessna 172\n Inserire i dati iniziali\n --------------------------------------------\n\nInserire la velocità inziale: ");
    scanf("%lf", &CI[0]);
    printf("Inserire l'altitudine inziale: ");
    scanf("%lf", &CI[1]);
    printf("Inserire l'angolo di attacco inziale: ");
    scanf("%lf", &CI[2]);
    printf("\n");

    datiFiles(0, &engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    AtmosphereChoice(&press0, &temp0, &rho0, &vsuono0, &press_h, &temp_h, &rho_h, &vsuono_h, CI, &flagatm);

    AtmosphereCalc(CI, &engine, &Pmax_h, &press0, &temp0, &rho0, &vsuono0, &press_h, &temp_h, &rho_h, &vsuono_h, &flagatm);

    equation(5.0, Pmax_h, rho_h, CI, body_axes, aer_der_x, aer_der_y, aer_der_z, steady_state_coeff, control_force_der, 
        control_moment_der, rotary_der, pitch_moment_der, geometry_propeller, propeller_profile, data_propeller, fuel_mass);
    /*do {
        // inizializzazione - Equazioni della dinamica

        // inizializzazione - CONDIZIONI DI TRIM (){ autovalori e autovettori, angoli di Trim, numero di giri, coefficienti aer tot}


        // Interpoliamo per calcolare i dati aerodinamici corretti
        InterpolazioneCoeff(0, &body_axes, CI, &temp_h, 2.0, Int_ssc, Int_adx, Int_ady, Int_adz, Int_rmd, 
        Int_pmd, Int_ymd, Int_cfd, Int_cmd, Int_rd, &steady_state_coeff, &aer_der_x, &aer_der_y, 
        &aer_der_z, &rolling_moment_der, &pitch_moment_der, &yawing_moment_der, &control_force_der, 
        &control_moment_der, &rotary_der);



    } while (1);*/

    return 0;

    

    // inizializzazione - COMANDI DI VOLO (comando desiderato semplice) {calcolo e creazione del vetore comando}

    // loop - EQUAZIONI DEL MOTO (Tutti i dati aerodinamici e di potenza, comandi input)  { studiare la stabilità statica e dinamica, 
    //                                                                                      calcolare i dati aggiornati cambiati in base alle manovre}
}