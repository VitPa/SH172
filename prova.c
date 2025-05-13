#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void stampaInterpolazione1(double * Int_ssc, double * Int_adx, double * Int_ady, double * Int_adz, double * Int_rmd, 
    double * Int_pmd, double * Int_ymd, double * Int_cfd, double * Int_cmd, double * Int_rd){
    for(int j = 0; j < 7; j++){
        if(j == 6){
            printf("Int_adx[%d] = %lf\n", j, Int_adx[j]);
            printf("------------------------------\n");
            printf("Int_adz[%d] = %lf\n", j, Int_adz[j]);
            printf("------------------------------\n");
            printf("Int_pmd[%d] = %lf\n", j, Int_pmd[j]);
            printf("------------------------------\n");
        }else{
            printf("Int_ssc[%d] = %lf\n", j, Int_ssc[j]);
            printf("------------------------------\n");
            printf("Int_adx[%d] = %lf\n", j, Int_adx[j]);
            printf("------------------------------\n");
            printf("Int_ady[%d] = %lf\n", j, Int_ady[j]);
            printf("------------------------------\n");
            printf("Int_adz[%d] = %lf\n", j, Int_adz[j]);
            printf("------------------------------\n");
            printf("Int_rmd[%d] = %lf\n", j, Int_rmd[j]);
            printf("------------------------------\n");
            printf("Int_pmd[%d] = %lf\n", j, Int_pmd[j]);
            printf("------------------------------\n");
            printf("Int_ymd[%d] = %lf\n", j, Int_ymd[j]);
            printf("------------------------------\n");
            printf("Int_cfd[%d] = %lf\n", j, Int_cfd[j]);
            printf("------------------------------\n");
            printf("Int_cmd[%d] = %lf\n", j, Int_cmd[j]);
            printf("------------------------------\n");
            printf("Int_rd[%d] = %lf\n", j, Int_rd[j]);
        }
    }
}

void main(){

    // ENGINE
        // 0 -> Potenza massima sl                              [KW]
        // 1 -> Legge variazionale della potenza con la quota   [-]
        // 2 -> Numero di giri motore minimo                    [rpm]
        // 3 -> Numero di giri motore massimo                   [rpm]
        // 4 -> Rendimento meccanico trasmissione               [-]
        // 5 -> Consumo specifico                               [Kg/s]

    // PROPELLER
        /*      geometry_propeller      */
        // 0 -> Diametro            [m]
        // 1 -> Diametro ogiva      [m]
        // 2 -> Numero di pale      [-]
        // 3 -> Inerzia             [kgm^2]
        // 4 -> Numero di stazioni  [-]

        /*      propeller_profile       */
        // 0 -> Clalfa  [rad^-1]
        // 1 -> Cl0     [-]
        // 2 -> a0      [rad]
        // 3 -> Cdalfa2 [rad^-2]
        // 4 -> Cdalfa  [rad^-1]
        // 5 -> Cd0     [-]

        /*      data_propeller          */
        // x , 0 -> CSI     [-]
        // x , 1 -> RD      [m]
        // x , 2 -> CH AD   [-]
        // x , 3 -> BA      [deg]
    
    // DBA
        /*      body_axes        */
        // 0 -> Massa                                                       [Kg]
        // 1 -> Apertura alare                                              [m]
        // 2 -> Superficie alare                                            [m^2]
        // 3 -> Corda                                                       [m]
        // 4 -> Mach drag rise                                              [-]
        // 5 -> Distanza tra asse di spinta e asse baricentrico X lungo Z   [m]
        // 6 -> Distanza tra asse di spinta e asse baricentrico X lungo Y   [m]
        // 7 -> Distanza tra asse di spinta e asse baricentrico X lungo Z   [m]
        // 8 -> Angolo tra asse di spinta e asse X nel piano X-Y (dtz)      [deg](>0 verso DX)
        // 9 -> Angolo tra asse di spinta e asse X nel piano X-Z (dtY)      [deg](>0 verso l'alto)
        // 10 -> Numero di incidenze per cui sono disponibili i dati        [-]
        // 11 -> Rotary derivatives (1-->presenti)                          [-]
        // 12 -> Center of gravity reference location                       [m]
            // 13 -> Jx                                                     [kgm^2]
            // 14 -> Jy                                                     [kgm^2]
            // 15 -> Jz                                                     [kgm^2]
            // 16 -> Jxz                                                    [kgm^2]
            // 17 -> Opzione posizione baricentro                           [-]
        // 18 -> Nuova posizione del baricentro                             [m]
            // 19 -> Posizione pilota lungo X (rispetto al C.G.)            [m]
            // 20 -> Posizione pilota lungo Y (rispetto al C.G.)            [m]
            // 21 -> Posizione pilota lungo Z (rispetto al C.G.)            [m]

        /*      deflection_limits       */
        // 0 -> Elevator (max)              [deg]
        // 1 -> Elevator (min)              [deg]
        // 2 -> Ailerons (symmetrical)      [deg]
        // 3 -> Rudder   (symmetrical)      [deg]
        // 4 -> Flap     (min deflection)   [deg]
        // 5 -> Flap     (max deflection)   [deg]
        
        /*      mass_data               */
        // 0 -> Option mass (0-->constant, 1-->variable)    [-]
        // 1 -> Fraction of mass (% MTOW)                   [-]

        /*      steady_state_coefficients       */
        // x , 0 -> ALPHA
        // x , 1 -> CX
        // x , 2 -> CY
        // x , 3 -> CZ
        // x , 4 -> Cl
        // x , 5 -> Cm
        // x , 6 -> Cn

        /*      aerodynamic_derivatives      */
        /*      x force derivatives          */
        // x , 0 -> ALPHA
        // x , 1 -> CXA
        // x , 2 -> CXAP
        // x , 3 -> CXU
        // x , 4 -> CXQ
        // x , 5 -> CXB
        // x , 6 -> CXP
        // x , 7 -> CXR

        /*      y force derivatives          */
        // x , 0 -> ALPHA
        // x , 1 -> CYB
        // x , 2 -> CYBP
        // x , 3 -> CYP
        // x , 4 -> CYR
        // x , 5 -> CYA
        // x , 6 -> CYQ
        
        /*      z force derivatives          */
        // x , 0 -> ALPHA
        // x , 1 -> CZALPHA
        // x , 2 -> CZAP
        // x , 3 -> CZU
        // x , 4 -> CZQ
        // x , 5 -> CZB
        // x , 6 -> CZP
        // x , 7 -> CZR

        /*      rolling_moment_derivatives           */
        // x , 0 -> ALPHA
        // x , 1 -> ClB
        // x , 2 -> ClBP
        // x , 3 -> ClP
        // x , 4 -> ClR
        // x , 5 -> ClA
        // x , 6 -> ClQ

        /*      pitching_moment_derivatives          */
        // x , 0 -> APLHA
        // x , 1 -> CmA
        // x , 2 -> CmAP
        // x , 3 -> CmU
        // x , 4 -> CmQ
        // x , 5 -> CmB
        // x , 6 -> CmP
        // x , 7 -> CmR

        /*      yawing_moment_derivatives            */
        // x , 0 -> ALPHA
        // x , 1 -> CnB
        // x , 2 -> CnBP
        // x , 3 -> CnP
        // x , 4 -> CnR
        // x , 5 -> CnA
        // x , 6 -> CnQ

        /*      control_force_derivates             */
        // x , 0 -> ALPHA
        // x , 1 -> CXde
        // x , 2 -> CXdle
        // x , 3 -> CZde
        // x , 4 -> CZdle
        // x , 5 -> CYda
        // x , 6 -> CYdr

        /*      control_moment_derivates            */
        // x , 0 -> ALPHA
        // x , 1 -> Clda
        // x , 2 -> Cldr
        // x , 3 -> Cmde
        // x , 4 -> Cmdle
        // x , 5 -> Cnda
        // x , 6 -> Cndr

        /*      rotary_derivates            */
        // x , 0 -> ALPHA
        // x , 1 -> CXom
        // x , 2 -> CYom
        // x , 3 -> CZom
        // x , 4 -> Clom
        // x , 5 -> Cmom
        // x , 6 -> Cnom

    FILE *file1 = fopen("dati/engine.txt", "r");
    if (file1 == NULL) {
        printf("Impossibile aprire il file engine.txt\n");
    } else {
        double *engine = NULL;
        int eng_num_valori = 0;

        char eng_riga[1024];
        while (fgets(eng_riga, 1024, file1) != NULL) {
            
            if (eng_riga[0] == '*') {
                continue;
            }

            double eng_valore;
            if (sscanf(eng_riga, "%lf", &eng_valore) == 1) {
                
                engine = realloc(engine, (eng_num_valori + 1) * sizeof(double));
                engine[eng_num_valori] = eng_valore;
                eng_num_valori++;
            }
        }

        /*printf("Engine: \n");
        for (int i = 0; i < eng_num_valori; i++) {
            printf("%.10lf\n", engine[i]);
        }*/

        fclose(file1);
    }

    FILE *file2 = fopen("dati/propeller.txt", "r");
    if (file2 == NULL) {
        printf("Impossibile aprire il file propeller.txt\n");
    } else {
        double *geometry_propeller = NULL;
        double *propeller_profile = NULL;
        double **data_propeller = NULL;
        int prop_num_val = 0;
        int count = 0;
        int section = 0;
        int check = 0;
        int geometry_num;

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
                        geometry_propeller = realloc(geometry_propeller, (prop_num_val + 1) * sizeof(double));
                        geometry_propeller[prop_num_val] = prop_value;
                        prop_num_val++;
                        check = 1;
                        break;
                    }
                case 1:
                    if (sscanf(prop_riga, "%lf", &prop_value) == 0){
                        break;
                    } else {
                        propeller_profile = realloc(propeller_profile, (prop_num_val + 1) * sizeof(double));
                        propeller_profile[prop_num_val] = prop_value;
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

                        data_propeller = realloc(data_propeller, (count + 1) * sizeof(double *));
                        data_propeller[count] = malloc(4 * sizeof(double));
                        for (int j = 0; j < 4; j++){
                            data_propeller[count][j] = data_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
            }
        }

        /*printf("\n\n\n");
        printf("Propeller: \n");

        for (int i = 0; i < geometry_num; i++) {
            printf("%.10lf\n", geometry_propeller[i]);
        }
        printf("\n\n\n");

        for (int i = 0; i < prop_num_val; i++) {
            printf("%.10lf\n", propeller_profile[i]);
        }

        printf("\n\n\n");

        for (int i = 0; i < count; i++){
            for (int j = 0; j < 4; j++) {
                printf("%.4lf\t", data_propeller[i][j]);
            }
            printf("\n");
        }*/
        fclose(file2);
    }

    FILE *file3 = fopen("dati/dba.txt", "r");
    if (file3 == NULL) {
        printf("Impossibile aprire il file dba.txt\n");
    } else {
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
        int dba_num = 0;
        int count = 0;
        int section = 0;
        int check = 0;
        int body_num, deflection_num, count1, count2, count3, count4, count5, count6, count7, count8, count9;

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
                            count1 = count;
                            count = 0;
                            break;
                        case 4:
                            section = 5;
                            check = 0;
                            count2 = count;
                            count = 0;
                            break;
                        case 5:
                            section = 6;
                            check = 0;
                            count3 = count;
                            count = 0;
                            break;
                        case 6:
                            section = 7;
                            check = 0;
                            count4 = count;
                            count = 0;
                            break;
                        case 7:
                            section = 8;
                            check = 0;
                            count5 = count;
                            count = 0;
                            break;
                        case 8:
                            section = 9;
                            check = 0;
                            count6 = count;
                            count = 0;
                            break;
                        case 9:
                            section = 10;
                            check = 0;
                            count7 = count;
                            count = 0;
                            break;
                        case 10:
                            section = 11;
                            check = 0;
                            count8 = count;
                            count = 0;
                            break;
                        case 11:
                            section = 12;
                            check = 0;
                            count9 = count;
                            count = 0;
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
                        body_axes = realloc(body_axes, (dba_num + 1) * sizeof(double));
                        body_axes[dba_num] = dba_value;
                        dba_num++;
                        check = 1;
                        break;
                    }
                case 1:
                    if (sscanf(prop_riga, "%lf", &dba_value) == 0){
                        break;
                    } else {
                        deflection_limits = realloc(deflection_limits, (dba_num + 1) * sizeof(double));
                        deflection_limits[dba_num] = dba_value;
                        dba_num++;
                        check = 1;
                        break;
                    }
                case 2:
                    if (sscanf(prop_riga, "%lf", &dba_value) == 0){
                        break;
                    } else {
                        fuel_mass = realloc(fuel_mass, (dba_num + 1) * sizeof(double));
                        fuel_mass[dba_num] = dba_value;
                        dba_num++;
                        check = 1;
                        break;
                    }
                case 3:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        steady_state_coeff = realloc(steady_state_coeff, (count + 1) * sizeof(double *));
                        steady_state_coeff[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            steady_state_coeff[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 4:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6], &data_dba_value[7]) == 0){
                        break;
                    } else {
                        aer_der_x = realloc(aer_der_x, (count + 1) * sizeof(double *));
                        aer_der_x[count] = malloc(8 * sizeof(double));
                        for (int j = 0; j < 8; j++){
                            aer_der_x[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 5:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        aer_der_y = realloc(aer_der_y, (count + 1) * sizeof(double *));
                        aer_der_y[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            aer_der_y[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 6:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6], &data_dba_value[7]) == 0){
                        break;
                    } else {
                        aer_der_z = realloc(aer_der_z, (count + 1) * sizeof(double *));
                        aer_der_z[count] = malloc(8 * sizeof(double));
                        for (int j = 0; j < 8; j++){
                            aer_der_z[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 7:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        rolling_moment_der = realloc(rolling_moment_der, (count + 1) * sizeof(double *));
                        rolling_moment_der[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            rolling_moment_der[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 8:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6], &data_dba_value[7]) == 0){
                        break;
                    } else {
                        pitch_moment_der = realloc(pitch_moment_der, (count + 1) * sizeof(double *));
                        pitch_moment_der[count] = malloc(8 * sizeof(double));
                        for (int j = 0; j < 8; j++){
                            pitch_moment_der[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 9:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        yawing_moment_der = realloc(yawing_moment_der, (count + 1) * sizeof(double *));
                        yawing_moment_der[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            yawing_moment_der[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 10:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        control_force_der = realloc(control_force_der, (count + 1) * sizeof(double *));
                        control_force_der[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            control_force_der[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 11:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        control_moment_der = realloc(control_moment_der, (count + 1) * sizeof(double *));
                        control_moment_der[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            control_moment_der[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
                case 12:
                    if (sscanf(prop_riga, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &data_dba_value[0], &data_dba_value[1], &data_dba_value[2], &data_dba_value[3], &data_dba_value[4], &data_dba_value[5], &data_dba_value[6]) == 0){
                        break;
                    } else {
                        rotary_der = realloc(rotary_der, (count + 1) * sizeof(double *));
                        rotary_der[count] = malloc(7 * sizeof(double));
                        for (int j = 0; j < 7; j++){
                            rotary_der[count][j] = data_dba_value[j];
                        }
                        count++;
                        check = 1;
                        break;
                    }
            }
        }

        /*printf("\n\n\n");
        printf("DBA: \n");

        for (int i = 0; i < body_num; i++) {
            printf("%.3lf\n", body_axes[i]);
        }
        printf("\n\n\n");

        for (int i = 0; i < deflection_num; i++) {
            printf("%.3lf\n", deflection_limits[i]);
        }

        printf("\n\n\n");

        for (int i = 0; i < dba_num; i++) {
            printf("%.3lf\n", fuel_mass[i]);
        }

        printf("\n\n\n");

        for (int i = 0; i < count1; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", steady_state_coeff[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count2; i++){
            for (int j = 0; j < 8; j++) {
                printf("%.4lf\t", aer_der_x[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count3; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", aer_der_y[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count4; i++){
            for (int j = 0; j < 8; j++) {
                printf("%.4lf\t", aer_der_z[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count5; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", rolling_moment_der[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count6; i++){
            for (int j = 0; j < 8; j++) {
                printf("%.4lf\t", pitch_moment_der[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count7; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", yawing_moment_der[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count8; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", control_force_der[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count9; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", control_moment_der[i][j]);
            }
            printf("\n");
        }

        printf("\n\n\n");

        for (int i = 0; i < count; i++){
            for (int j = 0; j < 7; j++) {
                printf("%.4lf\t", rotary_der[i][j]);
            }
            printf("\n");
        }*/

        fclose(file3);

        double mio_alpha = 25.0, R = 287.05, a, b, a1, a2;
        int trovato = 0, i = 0;
        double Int_ssc[6], Int_adx[7], Int_ady[6], Int_adz[7], Int_rmd[6], Int_pmd[7], Int_ymd[6], Int_cfd[6], Int_cmd[6], Int_rd[6];
        
        double CI[3] = {100, 0, 0};
        double temp_h = 15 + 273.15;

        double M = CI[0]/(sqrt(1.4 * R * temp_h));

        if (M > body_axes[4]){
            printf("La velocità è maggiore di quella di drag rise!");
            exit(1);
        }

        if (mio_alpha > 20.0){
            mio_alpha = 20;
            printf("Il valore di aplha è superiore del valore massimo consentito. \nIl valore impostato è: 20°\n");
        } else if(mio_alpha < -5){
            mio_alpha = -5;
            printf("Il valore di aplha è inferiore del valore minimo consentito. \nIl valore impostato è: -5°\n");
        }

        while (trovato == 0){

            if (mio_alpha == steady_state_coeff[i][0]){
                
                for(int j = 1; j < 8; j++){
                    if (j == 7){
                        Int_adx[j-1] = aer_der_x[i][j];
                        Int_adz[j-1] = aer_der_z[i][j];
                        Int_pmd[j-1] = pitch_moment_der[i][j];
                    } else {
                        Int_ssc[j-1] = steady_state_coeff[i][j];
                        Int_ady[j-1] = aer_der_y[i][j];
                        Int_rmd[j-1] = rolling_moment_der[i][j];
                        Int_ymd[j-1] = yawing_moment_der[i][j];
                        Int_cfd[j-1] = control_force_der[i][j];
                        Int_cmd[j-1] = control_moment_der[i][j];
                        Int_rd[j-1] = rotary_der[i][j];
                        Int_adx[j-1] = aer_der_x[i][j];
                        Int_adz[j-1] = aer_der_z[i][j];
                        Int_pmd[j-1] = pitch_moment_der[i][j];
                    }
                    
                }
                stampaInterpolazione1(Int_ssc, Int_adx, Int_ady, Int_adz, Int_rmd, Int_pmd, Int_ymd, Int_cfd, Int_cmd, Int_rd);
                trovato = 1;

            } else if (mio_alpha < steady_state_coeff[i][0]){
                a2 = steady_state_coeff[i][0];
                a1 = steady_state_coeff[i-1][0];

                for(int j = 0; j < 7; j++){
                    if (j == 6) {
                        a = (aer_der_x[i-1][j+1] - aer_der_x[i][j+1])/(a1 - a2);
                        b = (aer_der_x[i][j+1] - a * a2);
                        Int_adx[j] = a * mio_alpha + b;

                        a = (aer_der_z[i-1][j+1] - aer_der_z[i][j+1])/(a1 - a2);
                        b = (aer_der_z[i][j+1] - a * a2);
                        Int_adz[j] = a * mio_alpha + b;
                        
                        a = (pitch_moment_der[i-1][j+1] - pitch_moment_der[i][j+1])/(a1 - a2);
                        b = (pitch_moment_der[i][j+1] - a * a2);
                        Int_pmd[j] = a * mio_alpha + b;
                    } else {
                        a = (steady_state_coeff[i-1][j+1] - steady_state_coeff[i][j+1])/(a1 - a2);
                        b = (steady_state_coeff[i][j+1] - a * a2);
                        Int_ssc[j] = a * mio_alpha + b;

                        a = (aer_der_x[i-1][j+1] - aer_der_x[i][j+1])/(a1 - a2);
                        b = (aer_der_x[i][j+1] - a * a2);
                        Int_adx[j] = a * mio_alpha + b;

                        a = (aer_der_y[i-1][j+1] - aer_der_y[i][j+1])/(a1 - a2);
                        b = (aer_der_y[i][j+1] - a * a2);
                        Int_ady[j] = a * mio_alpha + b;

                        a = (aer_der_z[i-1][j+1] - aer_der_z[i][j+1])/(a1 - a2);
                        b = (aer_der_z[i][j+1] - a * a2);
                        Int_adz[j] = a * mio_alpha + b;

                        a = (rolling_moment_der[i-1][j+1] - rolling_moment_der[i][j+1])/(a1 - a2);
                        b = (rolling_moment_der[i][j+1] - a * a2);
                        Int_rmd[j] = a * mio_alpha + b;

                        a = (pitch_moment_der[i-1][j+1] - pitch_moment_der[i][j+1])/(a1 - a2);
                        b = (pitch_moment_der[i][j+1] - a * a2);
                        Int_pmd[j] = a * mio_alpha + b;

                        a = (yawing_moment_der[i-1][j+1] - yawing_moment_der[i][j+1])/(a1 - a2);
                        b = (yawing_moment_der[i][j+1] - a * a2);
                        Int_ymd[j] = a * mio_alpha + b;

                        a = (control_force_der[i-1][j+1] - control_force_der[i][j+1])/(a1 - a2);
                        b = (control_force_der[i][j+1] - a * a2);
                        Int_cfd[j] = a * mio_alpha + b;

                        a = (control_moment_der[i-1][j+1] - control_moment_der[i][j+1])/(a1 - a2);
                        b = (control_moment_der[i][j+1] - a * a2);
                        Int_cmd[j] = a * mio_alpha + b;

                        a = (rotary_der[i-1][j+1] - rotary_der[i][j+1])/(a1 - a2);
                        b = (rotary_der[i][j+1] - a * a2);
                        Int_rd[j] = a * mio_alpha + b;
                    }
                }
                stampaInterpolazione1(Int_ssc, Int_adx, Int_ady, Int_adz, Int_rmd, Int_pmd, Int_ymd, Int_cfd, Int_cmd, Int_rd);
                trovato = 1;
            }
            
            i++;
        }
    }
}