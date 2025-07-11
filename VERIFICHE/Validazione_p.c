#include <stdio.h>
#include <stdlib.h>
#include "../SIMULATORE/Atmosphere/Atmosphere.h"
#include "../SIMULATORE/Error_Warning/ErrorWarning.h"
#include "../SIMULATORE/Error_Warning/InitialCondition.h"
#include "../SIMULATORE/Pre_processing/Data.h"
#include "../SIMULATORE/Pre_processing/Variables.h"
#include "../SIMULATORE/Processing/Propeller.h"

void main() {
    printf("Inizio controllo propel -> ");
    system("PAUSE");

    loadData();

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
    AtmosphereChoice();
    AtmosphereCalc(CI[1]);

    system("cls");
    printf("\nVelocità iniziale: %g", CI[0]);
    printf("\nAltitudine iniziale: %g", CI[1]);

    fprintf(val2, "RPM\t\t Thrust\t\t Torque\n");
    for(int RPM = 1500; RPM <= 2700; RPM += 50) {
        double prop[3] = {0.0, 0.0, 0.0};
        double Pal;
        propel(RPM, CI[0], prop, &Pal);
        fprintf(val2, "%d\t|%.2lf \t|%.2lf\n", RPM, prop[0], prop[1]);
    }
    fclose(val2);

    printf("\n\n- Valori correttamente scritti nel file \"VALIDAZIONE_PROPEL.txt\"\n\n");

    printf("Validazione terminata con successo!\n");
    system("PAUSE");
}