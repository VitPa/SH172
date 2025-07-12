#include <stdio.h>
#include <stdlib.h>
#include "../SIMULATORE/Atmosphere/Atmosphere.h"
#include "../SIMULATORE/Error_Warning/ErrorWarning.h"
#include "../SIMULATORE/Error_Warning/InitialCondition.h"
#include "../SIMULATORE/Pre_processing/Data.h"
#include "../SIMULATORE/Pre_processing/Variables.h"
#include "../SIMULATORE/Processing/Propeller.h"

void main() {
    path_engine = "../SIMULATORE/_input_files/engine.txt";
    path_propeller = "../SIMULATORE/_input_files/propeller.txt";
    path_dba = "../SIMULATORE/_input_files/DBA.txt";

    printf("Inizio controllo propel -> ");
    system("PAUSE");

    loadData();

    val2 = openFile(path_v_p, "w");

    // *** Section: Load initial conditions ***
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

    AtmosphereChoice();
    AtmosphereCalc(CI[1]);

    // *** Print data in the file ***
    system("cls");
    printf("Velocita' iniziale: %g", CI[0]);
    printf("\nAltitudine iniziale: %g", CI[1]);

    fprintf(val2, "Velocità iniziale: %g", CI[0]);
    fprintf(val2, "\nAltitudine iniziale: %g\n\n", CI[1]);

    fprintf(val2, "RPM\t\t Thrust\t\t Torque\n");
    for(int RPM = 1500; RPM <= 2700; RPM += 50) {           // Compute and write propeller data for each RPM
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