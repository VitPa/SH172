#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "EstrazioneDati.h"
#include "Interpolazione.h"

void main() {
    //Programma utilizzato per validare i dati dei file in input e per validare l'interpolazione effettuata

    double alpha = 1;  // inseriamo un valore di alpha per validare i dati sui file
    double CL, CD, CL0 = 0.3832, CLa = 3.9849, aplha_0 = -0.09616302, CD0 = 0.0235, CDa = 0.154, CDa2 = 1.0476;
    double CX, CZ;
    int trovato = 1, i = 0;

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

    datiFiles(0, &engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    CL = CL0 + CLa * alpha;
    CD = CD0 + CDa * alpha + CDa2 * alpha * alpha;
    CX = - (CD * cos(alpha) + CL * sin(alpha));
    CZ = - (CD * sin(alpha) + CL * cos(alpha));

    while (trovato) {
        if(steady_state_coeff[i][0] == alpha){
            trovato = 0;
        } else {
            i++;
        }
    }

    printf("Il valore di CX sperimentale: %lf, il valore validato: %lf\n", steady_state_coeff[i][1], CX);
    printf("Il valore di CZ sperimentale: %lf, il valore validato: %lf\n", steady_state_coeff[i][3], CZ);
}