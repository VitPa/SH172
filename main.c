// Provare ad usare gnuplot per i grafici
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "Atmosphere.h"
#include "EstrazioneDati.h"
#include "EstrazioneDati_ottimizzato.h"
#include "Interpolazione_new.h"
#include "MotionEq.h"
#include "Integration.h"
#include "InitialCondition.h"

int main(){

    /*
    Implementare all'inzio un parte di codice che fa decidere (oppure si passa come argomento) 
    se far partire il simulatore in modalità utente o modalità test. Quindi verbosa o no. In più 
    si può fare una terza modalità in cui si stampano (magari chiedendo funzione per funzione) 
    tutti i calcoli intermedi per un debug ancora più avanzato
    */

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
    double **state = NULL;
    double Int_ssc[6], Int_adx[7], Int_ady[6], Int_adz[7], Int_rmd[6], Int_pmd[7], Int_ymd[6], Int_cfd[6], Int_cmd[6], Int_rd[6];
    double CI[3], trim[4], command[10000][4];
    double Pmax_h, press_h, temp_h, rho_h, vsuono_h;
    double dt = 0.01, deltaT_fs;
    int flagatm, state_rows = 1;

    // Caricamento variabili da file .txt
    datiFiles(0, &engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    // Caricamento condizioni iniziali
    loadCI(CI);
    AtmosphereChoice(&press_h, &temp_h, &rho_h, &vsuono_h, &flagatm);
    AtmosphereCalc(CI[1], engine, &Pmax_h, &press_h, &temp_h, &rho_h, &vsuono_h, flagatm);
    checkMdgAlt(CI[0], CI[1], body_axes[4], temp_h);

    // Calcolo del trim e stabilità con Routh
    equation(Pmax_h, rho_h, CI, &state, body_axes, aer_der_x, aer_der_z, steady_state_coeff, control_force_der, 
        control_moment_der, pitch_moment_der, geometry_propeller, propeller_profile, data_propeller, trim);

    // Inserimento manovra
    printf("Inserire il tempo di simulazione: ");
    scanf("%lf", &deltaT_fs);

    for (int i = 0; i < deltaT_fs/dt; ++i){
        command[i][0] = 0;
        command[i][1] = trim[1];
        command[i][2] = 0;
        command[i][3] = 0;
    }
    
    // Ciclo di simulazione
    int i = 0;
    for(double Ts = 0.00; Ts <= deltaT_fs; Ts += dt){
        
        // Ricalcolo variabili atmoferiche
        AtmosphereCalc(state[i][9], engine, &Pmax_h, &press_h, &temp_h, &rho_h, &vsuono_h, flagatm);

        // Aumento la dimensione del vettore di stato di una riga
        state = reallocState(state, &state_rows, 12);

        // Integro con Eulero le equazioni del moto
        eulerEquation(i, state, command, rho_h, trim[2], body_axes, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, 
            rolling_moment_der, pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, geometry_propeller, 
            propeller_profile, data_propeller);

        /* 
        Arrivati all'ultima iterazione, in base a come è scritta la funzione, calcola comunque i valori di state al 
        passo successvo anche se al passo successivo lasimulazione sarà terminata. È corretto?
        */
        ++i;
    }

    // Libera la memoria
    liberaTuttiIDati(engine, geometry_propeller, propeller_profile, data_propeller, body_axes, deflection_limits, 
        fuel_mass, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, pitch_moment_der, 
        yawing_moment_der, control_force_der, control_moment_der, rotary_der, state);

    return 0;
}