#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "Atmosphere.h"
#include "EstrazioneDati.h"
#include "Interpolazione.h"
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
   /*
   Implementae una struct o una nuova funzione per la gestione degli errori. Ti deve permettere di
   fare (system("PAUSE"); exit(0);) se è un'errore oppure gestire il warning, magari passando il 
   messaggio da stampare e distinguere tra warning silenzioso o warning non silenzioso (cioè se 
   l'utente deve intervenire oppure no).
   In ogni caso tutti i warning ed errori li deve stampare anche su un file di log
   */

    double *engine = NULL;
    double *geometry_propeller = NULL, *propeller_profile = NULL, **data_propeller = NULL;
    double *body_axes = NULL, *deflection_limits = NULL, *fuel_mass = NULL;
    double **steady_state_coeff = NULL, **aer_der_x = NULL, **aer_der_y = NULL,**aer_der_z = NULL;
    double **rolling_moment_der = NULL, **pitch_moment_der = NULL, **yawing_moment_der = NULL;
    double **control_force_der = NULL, **control_moment_der = NULL, **rotary_der = NULL;
    double **state = NULL;
    double trim[4];
    double (*command)[4] = malloc(1000000 * sizeof(*command));
    if (!command) {
        printf("Errore allocazione memoria per command\n");
        return 1;
    }

    // Caricamento variabili da file .txt
    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);
    /*stampaTuttiIDati(engine, geometry_propeller, propeller_profile, data_propeller, body_axes, deflection_limits,
        fuel_mass, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, 
        pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, rotary_der);*/

    // Caricamento condizioni iniziali
    int flagatm;
    double Pmax_h, press_h, temp_h, rho_h, vsuono_h;
    double CI[3];

    loadCI(CI);
    AtmosphereChoice(&press_h, &temp_h, &rho_h, &vsuono_h, &flagatm);
    AtmosphereCalc(CI[1], engine, &Pmax_h, &press_h, &temp_h, &rho_h, &vsuono_h, flagatm);

    checkVelAlt(&CI[0], CI[1], body_axes[4], temp_h);

    // Calcolo del trim e stabilità con Routh
    equation(engine, Pmax_h, rho_h, CI, &state, body_axes, aer_der_x, aer_der_z, steady_state_coeff, control_force_der, 
        control_moment_der, pitch_moment_der, geometry_propeller, propeller_profile, data_propeller, trim);

    // Inserimento manovra
    double dt = 0.01, deltaT_fs;
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
        state = reallocState(state, 12);

        // Integro con Eulero le equazioni del moto
        eulerEquation(dt, i, state, command, Pmax_h, rho_h, trim[2], engine, body_axes, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, 
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
    free(command);
    return 0;
}