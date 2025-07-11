#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "ErrorWarning.h"
#include "Variables.h"

static double press0 = 101325;    // Pa
static double temp0 = 15;         // C
static double rho0 = 1.225;       // kg/m^3
static double vsuono0 = 340;      // m/s
static int flagatm;

void AtmosphereChoice ()
{
    int input;

    printf("\n\\\\\\\\\\\\\\\\\\\\\\\n\\\\\\\\ATMOSFERA ISA\n\\\\\\\\\\\\\\\\\\\\\\\n");
    printf("\nLa simulazione fa riferimento al modello atmosferico ISA, avente i segeunti valori per quota h=0m (sea level):\n\n");
    printf("\tPressione: \t\tP = %g Pa\n",press0);
    printf("\tTemperatura: \t\tT = %g C\n",temp0);
    printf("\tDensita': \t\trho = %g kg/m^3\n",rho0);
    printf("\tVelocita' del Suono: \ta = %g m/s\n",vsuono0);

    printf("\nSe non si desidera procedere con i suddetti parametri e' possibile modificarli, reinserendoli manualmente o scegliendo una quota differente.\n");
    do {
        printf("\nPremere:\n");
        printf("\t[1] se si desidera procedere con i valori iniziali precedentemente indicati\n"); // mantiene i valori iniziali di default
        printf("\t[2] se si desidera modificare i valori iniziali a h=0\n"); //modifica i valori iniziali di default
        printf("\t[3] se di desidera immettere i valori manualmente a una quota specifica (richiesta in seguito)\n");// immette i valori manualmente a una data quota senza richiederla poi

        scanf("%d", &input);
        if(input!=1 && input!=2 && input!=3)
        {
            scanf("%*[^\n]"); //svuota il buffer di scanf se immetto un carattere invece che un numero
            WARNING(500, 1.0, 3.0);
        }
        switch(input)
        {
            case 1: // mantiene i valori iniziali di default
                flagatm = 1;
                break;
            case 2: //modifica i valori iniziali di default
                printf("Inserire un valore di pressione (h=0) [Pa]:");    
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &press0) != 0 && press0>0) break;
                    WARNING(501);
                } while (1);

                printf("Inserire un valore di temperatura (h=0) [C]:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &temp0) != 0 && temp0>-273.15) break;
                    WARNING(502, -273.15);
                } while (1);

                printf("Inserire un valore di densita' (h=0) [kg/m^3]:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &rho0) != 0 && rho0>0) break;
                    WARNING(501);
                } while(1);

                printf("Inserire un valore di velocita' (h=0) del suono in m/s:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &vsuono0) != 0 && vsuono0>0) break;
                    WARNING(501);
                } while (1);

                flagatm = 1;
                break;
            case 3: // immette i valori manualmente a una data quota senza richiederla poi
                printf("Inserire un valore di pressione [Pa]:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &press_h) != 0 && press_h>0) break;
                    WARNING(501);
                } while (1);

                printf("Inserire un valore di temperatura [C]:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &temp_h) != 0 && temp_h>-273.15) break;
                    WARNING(502, -273.15);
                } while (1);

                printf("Inserire un valore di densita' [kg/m^3]:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &rho_h) != 0 && rho_h>0) break;
                    WARNING(501);
                } while (1);

                printf("Inserire un valore di velocita' del suono [m/s]:");
                do {
                    scanf("%*[^\n]");
                    if(scanf("%lf", &vsuono_h) != 0 && vsuono_h>0) break;
                    WARNING(501);
                } while (1);
                flagatm = 2;
                break;
        }
    }
    while(input!=1 && input!=2 && input!=3);
}

void AtmosphereCalc (double h)
{
    double R=287.05, gamma=1.3954;
    static int stampa = 0;
    switch(flagatm)
    {
        case 1: // calcola le condizioni atmosferiche e di pot per i valori a SL di default o inseriti dall'utente
            double temp = temp0 + 273.15;
            temp_h = temp - 0.0065*h;
            press_h = press0*(pow((temp_h/(temp)),5.2561));
            rho_h = press_h/(R*temp_h);
            Pmax_h = engine[0] * pow(rho_h/(rho0),engine[1]);
            vsuono_h = sqrt(gamma*R*(temp_h));
            if (stampa == 0){
                printf("\nI dati atmosferici e di potenza per la quota di %d m sono:\n\n", (int) h);
                printf("Temperatura: \t\t%g [K]\n",temp_h);
                printf("Pressione: \t\t%g [Pa]\n",press_h);
                printf("Desita': \t\t%g [kg/m^3]\n",rho_h);
                printf("Potenza: \t\t%g [kW]\n",Pmax_h);
                stampa = 1;
            }
            break;
        case 2:// usa i valori scelti dall'utente
            temp_h=temp_h+273.15;
            Pmax_h= engine[0] * pow(rho_h/(rho0),engine[1]);

            if (stampa == 0){
                printf("\nI dati atmosferici e di potenza per la quota di %f m sono:\n\n",h);
                printf("Temperatura: \t\t%g [K]\n",temp_h);
                printf("Pressione: \t\t%g [Pa]\n",press_h);
                printf("Densita': \t\t%g [kg/m^3]\n",rho_h);
                printf("Potenza: \t\t%g [kW]\n",Pmax_h);
                stampa = 1;
            }
            break;
    }
}