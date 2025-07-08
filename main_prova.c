#include <stdio.h>
#include <stdlib.h>
#include "Command.h"
#include "EstrazioneDati.h"

int main(){
    double *engine = NULL;
    double *geometry_propeller = NULL, *propeller_profile = NULL, **data_propeller = NULL;
    double *body_axes = NULL, *deflection_limits = NULL, *fuel_mass = NULL;
    double **steady_state_coeff = NULL, **aer_der_x = NULL, **aer_der_y = NULL,**aer_der_z = NULL;
    double **rolling_moment_der = NULL, **pitch_moment_der = NULL, **yawing_moment_der = NULL;
    double **control_force_der = NULL, **control_moment_der = NULL, **rotary_der = NULL;

    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    
    double **command = load_command(0.01, 2, 0.15, 0.1);
    int n = 2/0.01;
    for (int i = 0; i < n; ++i) {
        printf("%.2lf  -  ", i*0.01);
        for (int j = 0; j < 4; ++j) {
            printf("%g ", command[i][j]);
        }
        printf("\n");
    }
    system("PAUSE");
    return 0;
}