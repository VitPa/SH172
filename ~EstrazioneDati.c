#include <stdio.h>
#include <stdlib.h>
#include "EstrazioneDati.h"

void datiFiles(int stampa, double **engine, double **geometry_propeller, double **propeller_profile, double ***data_propeller, double **body_axes, double **deflection_limits,
    double **fuel_mass, double ***steady_state_coeff, double ***aer_der_x, double ***aer_der_y, double ***aer_der_z, double ***rolling_moment_der, 
    double ***pitch_moment_der, double ***yawing_moment_der, double ***control_force_der, double ***control_moment_der, double ***rotary_der){

    int eng_num_valori = 0;
    int prop_num_val = 0;
    int geometry_num;
    int count = 0;
    int dba_num = 0;
    int count0 = 0;
    int body_num, deflection_num, count1, count2, count3, count4, count5, count6, count7, count8, count9;

    FILE *file1 = fopen("dati/engine.txt", "r");
    if (file1 == NULL) {
        printf("Impossibile aprire il file engine.txt\n");
    } else {

        char eng_riga[1024];
        while (fgets(eng_riga, 1024, file1) != NULL) {
            
            if (eng_riga[0] == '*') {
                continue;
            }

            double eng_valore;
            if (sscanf(eng_riga, "%lf", &eng_valore) == 1) {
                
                *engine = realloc(*engine, (eng_num_valori + 1) * sizeof(double));
                (*engine)[eng_num_valori] = eng_valore;
                eng_num_valori++;
            }
        }

        fclose(file1);
    }

    FILE *file2 = fopen("dati/propeller.txt", "r");
    if (file2 == NULL) {
        printf("Impossibile aprire il file propeller.txt\n");
    } else {
        int section = 0;
        int check = 0;

        char prop_riga[1024];
        while (fgets(prop_riga, 1024, file2) != NULL) {
            
            if (prop_riga[0] == '*') {
                if (check == 1){
                    switch (section){
                        case 0:
                            geometry_num = prop_num_val;
                            prop_num_val = 0;
                            section = 1;
                            check = 0;
                            break;
                        case 1:
                            section = 2;
                            check = 0;
                            break;
                        case 2:
                            section = 3;
                            check = 0;
                            break;      
                    }
                }
                continue;
            }

            double prop_value;
            double data_value[4];     //vettore temporaneo per salvataggio dati da file
            switch (section){
                case 0:
                    if (sscanf(prop_riga, "%lf", &prop_value) == 0){
                        break;
                    } else {
                        *geometry_propeller = realloc(*geometry_propeller, (prop_num_val + 1) * sizeof(double));
                        (*geometry_propeller)[prop_num_val] = prop_value;
                        prop_num_val++;
                        check = 1;
                        break;
                    }
                case 1:
                    if (sscanf(prop_riga, "%lf", &prop_value) == 0){
                        break;
                    } else {
                        *propeller_profile = realloc(*propeller_profile, (prop_num_val + 1) * sizeof(double));
                        (*propeller_profile)[prop_num_val] = prop_value;
                        prop_num_val++;
                        check = 1;
                        break;
                    }
                case 2:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf", &data_value[0], &data_value[1], &data_value[2], &data_value[3]) == 0){
                        break;
                    } else {
                        if (data_value[0] <= 0.19){
                            break;
                        }

                        *data_propeller = realloc(*data_propeller, (count + 1) * sizeof(double *));
                        (*data_propeller)[count] = malloc(4 * sizeof(double));
                        for (int j = 0; j < 4; j++){
                            (*data_propeller)[count][j] = data_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
            }
        }

        fclose(file2);
    }

    FILE *file3 = fopen("dati/dba.txt", "r");
    if (file3 == NULL) {
        printf("Impossibile aprire il file dba.txt\n");
    } else {
        int section = 0;
        int check = 0;

        char prop_riga[1024];
        while (fgets(prop_riga, 1024, file2) != NULL) {
            
            if (prop_riga[0] == '*' || prop_riga[1] == '*') {
                if (check == 1){
                    switch (section){
                        case 0:
                            body_num = dba_num;
                            dba_num = 0;
                            section = 1;
                            check = 0;
                            break;
                        case 1:
                            deflection_num = dba_num;
                            dba_num = 0;
                            section = 2;
                            check = 0;
                            break;
                        case 2:
                            section = 3;
                            check = 0;
                            break;
                        case 3:
                            section = 4;
                            check = 0;
                            count1 = count0;
                            count0 = 0;
                            break;
                        case 4:
                            section = 5;
                            check = 0;
                            count2 = count0;
                            count0 = 0;
                            break;
                        case 5:
                            section = 6;
                            check = 0;
                            count3 = count0;
                            count0 = 0;
                            break;
                        case 6:
                            section = 7;
                            check = 0;
                            count4 = count0;
                            count0 = 0;
                            break;
                        case 7:
                            section = 8;
                            check = 0;
                            count5 = count0;
                            count0 = 0;
                            break;
                        case 8:
                            section = 9;
                            check = 0;
                            count6 = count0;
                            count0 = 0;
                            break;
                        case 9:
                            section = 10;
                            check = 0;
                            count7 = count0;
                            count0 = 0;
                            break;
                        case 10:
                            section = 11;
                            check = 0;
                            count8 = count0;
                            count0 = 0;
                            break;
                        case 11:
                            section = 12;
                            check = 0;
                            count9 = count0;
                            count0 = 0;
                            break;
                    }
                }
                continue;
            }

            double dba_value;
            double data_dba_value[8];     //vettore temporaneo per salvataggio dati da file
            switch (section){
                case 0:
                    if (sscanf(prop_riga, "%lf", &dba_value) == 0){
                        break;
                    } else {
                        *body_axes = realloc(*body_axes, (dba_num + 1) * sizeof(double));
                        (*body_axes)[dba_num] = dba_value;
                        dba_num++;
                        check = 1;
                        break;
                    }
                case 1:
                    if (sscanf(prop_riga, "%lf", &dba_value) == 0){
                        break;
                    } else {
                        *deflection_limits = realloc(*deflection_limits, (dba_num + 1) * sizeof(double));
                        (*deflection_limits)[dba_num] = dba_value;
                        dba_num++;
                        check = 1;
                        break;
                    }
                case 2:
                    if (sscanf(prop_riga, "%lf", &dba_value) == 0){
                        break;
                    } else {
                        *fuel_mass = realloc(*fuel_mass, (dba_num + 1) * sizeof(double));
                        (*fuel_mass)[dba_num] = dba_value;
                        dba_num++;
                        check = 1;
                        break;
                    }
                case 3:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *steady_state_coeff = realloc(*steady_state_coeff, (count0 + 1) * sizeof(double *));
                        (*steady_state_coeff)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*steady_state_coeff)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 4:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6], &data_dba_value[7]) == 0){
                        break;
                    } else {
                        *aer_der_x = realloc(*aer_der_x, (count0 + 1) * sizeof(double *));
                        (*aer_der_x)[count0] = malloc(8 * sizeof(double));
                        for (int j = 0; j < 8; j++){
                            (*aer_der_x)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 5:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *aer_der_y = realloc(*aer_der_y, (count0 + 1) * sizeof(double *));
                        (*aer_der_y)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*aer_der_y)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 6:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6], &data_dba_value[7]) == 0){
                        break;
                    } else {
                        *aer_der_z = realloc(*aer_der_z, (count0 + 1) * sizeof(double *));
                        (*aer_der_z)[count0] = malloc(8 * sizeof(double));
                        for (int j = 0; j < 8; j++){
                            (*aer_der_z)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 7:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *rolling_moment_der = realloc(*rolling_moment_der, (count0 + 1) * sizeof(double *));
                        (*rolling_moment_der)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*rolling_moment_der)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 8:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6], &data_dba_value[7]) == 0){
                        break;
                    } else {
                        *pitch_moment_der = realloc(*pitch_moment_der, (count0 + 1) * sizeof(double *));
                        (*pitch_moment_der)[count0] = malloc(8 * sizeof(double));
                        for (int j = 0; j < 8; j++){
                            (*pitch_moment_der)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 9:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *yawing_moment_der = realloc(*yawing_moment_der, (count0 + 1) * sizeof(double *));
                        (*yawing_moment_der)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*yawing_moment_der)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 10:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *control_force_der = realloc(*control_force_der, (count0 + 1) * sizeof(double *));
                        (*control_force_der)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*control_force_der)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 11:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *control_moment_der = realloc(*control_moment_der, (count0 + 1) * sizeof(double *));
                        (*control_moment_der)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*control_moment_der)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
                case 12:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        *rotary_der = realloc(*rotary_der, (count0 + 1) * sizeof(double *));
                        (*rotary_der)[count0] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            (*rotary_der)[count0][j] = data_dba_value[j];
                        }
                        count0++;
                        check = 1;
                        break;
                    }
            }
        }

        fclose(file3);
    }

    if(stampa == 1){
        printf("Engine: \n");
        for (int i = 0; i < eng_num_valori; i++) {
            printf("%.10lf\n", (*engine)[i]);
        }

        printf("\n\n\n");

        printf("Propeller: \n");
        for (int i = 0; i < geometry_num; i++) {
            printf("%.10lf\n", (*geometry_propeller)[i]);
        }
        printf("\n\n\n");

        for (int i = 0; i < prop_num_val; i++) {
            printf("%.10lf\n", (*propeller_profile)[i]);
        }

        printf("\n\n\n");

        for (int i = 0; i < count; i++){
            for (int j = 0; j < 4; j++) {
                printf("%.4lf\t", (*data_propeller)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        printf("DBA: \n");

        for (int i = 0; i < body_num; i++) {
            printf("%.3lf\n", (*body_axes)[i]);
        }
        printf("\n\n\n");

        for (int i = 0; i < deflection_num; i++) {
            printf("%.3lf\n", (*deflection_limits)[i]);
        }

        printf("\n\n\n");

        for (int i = 0; i < dba_num; i++) {
            printf("%.3lf\n", (*fuel_mass)[i]);
        }

        printf("\n\n\n");

        for (int i = 0; i < count1; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*steady_state_coeff)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count2; i++){
            for (int j = 0; j < 8; j++) {
                printf("%.4lf\t", (*aer_der_x)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count3; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*aer_der_y)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count4; i++){
            for (int j = 0; j < 8; j++) {
                printf("%.4lf\t", (*aer_der_z)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count5; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*rolling_moment_der)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count6; i++){
            for (int j = 0; j < 8; j++) {
                printf("%.4lf\t", (*pitch_moment_der)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count7; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*yawing_moment_der)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count8; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*control_force_der)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count9; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*control_moment_der)[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count0; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", (*rotary_der)[i][j]);
            }
            printf("\n");
        }
    }
}