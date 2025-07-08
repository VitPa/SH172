#include <stdio.h>
#include <stdlib.h>
#include "EstrazioneDati.h"
#include "propeller.h"

void main() {
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

    printf("Inizio controllo propel -> ");
    system("PAUSE");

    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    FILE *val2 = apriFile("Validazione_output/VALIDAZIONE_PROPEL.txt", "w");

    fprintf(val2, "RPM\t\t Thrust\t\t Torque\n");
    for(int RPM = 1500; RPM <= 2700; RPM += 50) {
        double prop[3] = {0.0, 0.0, 0.0};
        double Pal;
        propel(RPM, 106.801832, 1.111648, 52, geometry_propeller, propeller_profile, data_propeller, prop, &Pal);
        fprintf(val2, "%d\t|%.2lf \t|%.2lf\n", RPM, prop[0], prop[1]);
    }
    fclose(val2);

    printf("\n- Valori correttamente scritti nel file \"VALIDAZIONE_PROPEL.txt\"\n\n");

    printf("Validazione terminata con successo!\n");
    system("PAUSE");
}