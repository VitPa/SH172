#include <stdio.h>
#include <stdlib.h>
#include "EstrazioneDati.h"
#include "propeller.h"
#include "InitialCondition.h"
#include "Atmosphere.h"
#include "ErrorWarning.h"

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
    double Pmax_h, press_h, temp_h, rho_h, vsuono_h;

    printf("Inizio controllo propel -> ");
    system("PAUSE");

    caricaTuttiIDati(&engine, &geometry_propeller, &propeller_profile, &data_propeller, &body_axes, &deflection_limits,
        &fuel_mass, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, 
        &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der);

    FILE *val2 = apriFile("Validazione_output/VALIDAZIONE_PROPEL.txt", "w");

    double CI[2];
    printf("Inserire la velocità inziale [m/s]: ");
    do{
        if(scanf("%lf",&CI[0])!=0) break;
        WARNING(504);
    }while(1);
    printf("\nInserire l'altitudine inziale [m]: ");
    do{
        if(scanf("%lf",&CI[1])!=0) break;
        WARNING(504);
    }while(1);

    system("cls");
    AtmosphereChoice(&press_h,&temp_h,&rho_h,&vsuono_h);
    AtmosphereCalc(CI[1], engine, &Pmax_h, &press_h, &temp_h, &rho_h, &vsuono_h);

    system("cls");
    printf("\nVelocità iniziale: %g", CI[0]);
    printf("\nAltitudine iniziale: %g", CI[1]);

    fprintf(val2, "RPM\t\t Thrust\t\t Torque\n");
    for(int RPM = 1500; RPM <= 2700; RPM += 50) {
        double prop[3] = {0.0, 0.0, 0.0};
        double Pal;
        propel(RPM, Pmax_h, rho_h, CI[0], geometry_propeller, propeller_profile, data_propeller, prop, &Pal);
        fprintf(val2, "%d\t|%.2lf \t|%.2lf\n", RPM, prop[0], prop[1]);
    }
    fclose(val2);

    printf("\n\n- Valori correttamente scritti nel file \"VALIDAZIONE_PROPEL.txt\"\n\n");

    printf("Validazione terminata con successo!\n");
    system("PAUSE");
}