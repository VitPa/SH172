#include <stdio.h>
#include <stdlib.h>

static int dimVett[6];
int dimMat[13];
double RPMmax, RPMmin;

FILE* apriFile(const char *path, const char *mode) {
    FILE* f = fopen(path, mode);
    if (!f) fprintf(stderr, "Errore apertura file: %s\n", path);
    return f;
}

double* caricaVettoreDouble(const char *path, int checkSection, int *outSize) {
    FILE* f = apriFile(path, "r");
    if (!f) return NULL;
    double* arr = NULL;
    int n = 0, section = 0, flag = 1;
    double val;
    char riga[256];
    while (fgets(riga, sizeof(riga), f)) {
        if (riga[0] == '*' || riga[1] == '*') {
            if (flag == 1) {flag = 0; ++section;}
            continue;
        }
        if (sscanf(riga, "%lf", &val) == 1) {
            if (section == checkSection){
                arr = realloc(arr, (n+1)*sizeof(double));
                arr[n++] = val;
            }
            flag = 1;
        }
    }
    fclose(f);
    if (outSize) *outSize = n;
    return arr;
}

double** caricaMatriceDouble(const char *path, int colonne, int checkSection, int *outRighe) {
    FILE* f = apriFile(path, "r");
    if (!f) return NULL;
    double **mat = NULL, t;
    int n = 0, section = 0, flag = 1;
    char riga[512];
    while (fgets(riga, sizeof(riga), f)) {
        if (riga[0] == '*' || riga[1] == '*') {
            if (flag == 1) {flag = 0; ++section;}
            continue;
        }
        if (sscanf(riga, "%lf", &t) == 1) {
            if (section == checkSection){
                double *temp = malloc(colonne * sizeof(double));
                int letti = 0;
                char *ptr = riga;
                for (int i = 0; i < colonne; ++i) {
                    while (*ptr == '\t' || *ptr == ' ') ++ptr;
                    if (sscanf(ptr, "%lf", &temp[i]) == 1) {
                        // Avanza ptr alla prossima tabulazione/spazio
                        char *next = ptr;
                        while (*next && *next != '\t' && *next != ' ' && *next != '\n') ++next;
                        ptr = next;
                        letti++;
                    } else {
                        break;
                    }
                }
                if (letti == colonne) {
                    mat = realloc(mat, (n+1)*sizeof(double*));
                    mat[n++] = temp;
                } else {
                    free(temp);
                }
            }
            flag = 1;   
        }
    }
    fclose(f);
    if (outRighe) *outRighe = n;
    return mat;
}

// Rialloca la matrice state aggiungendo una riga
double** reallocState(double **state, int n_colonne) {
    int new_rows = dimMat[11] + 1;
    double **tmp = realloc(state, new_rows * sizeof(double*));
    if (!tmp) {
        printf("[!]ERROR: Errore realloc state\n");
        system("PAUSE"); 
        exit(0);
    }
    tmp[new_rows - 1] = calloc(n_colonne, sizeof(double));
    if (!tmp[new_rows - 1]) {
        printf("[!]ERROR: Errore calloc nuova riga state\n");
        system("PAUSE"); 
        exit(0);
    }
    dimMat[11] = new_rows;
    return tmp;
}

// Rialloca la matrice command aggiungendo una riga
double** reallocCommand(double **command, int n_colonne) {
    int new_rows = dimMat[12] + 1; // Usa un indice libero per command
    double **tmp = realloc(command, new_rows * sizeof(double*));
    if (!tmp) {
        printf("[!]ERROR: Errore realloc command\n");
        system("PAUSE");
        exit(0);
    }
    tmp[new_rows - 1] = calloc(n_colonne, sizeof(double));
    if (!tmp[new_rows - 1]) {
        printf("[!]ERROR: Errore calloc nuova riga command\n");
        system("PAUSE");
        exit(0);
    }
    dimMat[12] = new_rows;
    return tmp;
}

void stampaVettoreFile(const char* nome, double* v, int n) {
    FILE *val1 = apriFile("Validazione_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt", "a");
    fprintf(val1, "\n\n--- %s (vettore, %d elementi) ---\n", nome, n);
    for (int i = 0; i < n; ++i) {
        fprintf(val1, "%.2lf\t", v[i]);
    }
    fclose(val1);
}

void stampaMatriceFile(const char* nome, double** m, int righe, int colonne) {
    FILE *val1 = apriFile("Validazione_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt", "a");
    fprintf(val1, "\n\n--- %s (matrice, %d x %d) ---\n", nome, righe, colonne);
    for (int i = 0; i < righe; ++i) {
        for (int j = 0; j < colonne; ++j) {
            fprintf(val1, "%s%s%.4lf\t", m[i][j]<0 ? "" : " ", m[i][j]<10 ? " " : "", m[i][j]);
        }
        fprintf(val1, "\n");
    }
    fclose(val1);
}

// Funzione wrapper per caricare tutti i dati richiesti dal simulatore
void caricaTuttiIDati(double **engine, double **geometry_propeller, double **propeller_profile, double ***data_propeller, double **body_axes, double **deflection_limits, double **fuel_mass, double ***steady_state_coeff, double ***aer_der_x, double ***aer_der_y, double ***aer_der_z, double ***rolling_moment_der, double ***pitch_moment_der, double ***yawing_moment_der, double ***control_force_der, double ***control_moment_der, double ***rotary_der) {
    int ok = 1;
    dimMat[11]=1;
    dimMat[12]=0;
    
    // VETTORI
    *engine = caricaVettoreDouble("dati/engine.txt", 1, &dimVett[0]); 
    if (!*engine) {printf("[!]ERROR: Errore caricamento vettore engine\n"); system("PAUSE"); exit(0);} //Error(101, "engine");
    RPMmin = (*engine)[2];
    RPMmax = (*engine)[3];
    *geometry_propeller = caricaVettoreDouble("dati/propeller.txt", 1, &dimVett[1]); 
    if (!*geometry_propeller) {printf("[!]ERROR: Errore caricamento vettore geometry_propeller\n"); system("PAUSE"); exit(0);}
    *propeller_profile = caricaVettoreDouble("dati/propeller.txt", 2, &dimVett[2]); 
    if(!*propeller_profile) {printf("[!]ERROR: Errore caricamento vettore propeller_profile\n"); system("PAUSE"); exit(0);}
    *body_axes = caricaVettoreDouble("dati/dba.txt", 1, &dimVett[3]); 
    if (!*body_axes) {printf("[!]ERROR: Errore caricamento vettore body_axes\n"); system("PAUSE"); exit(0);}
    *deflection_limits = caricaVettoreDouble("dati/dba.txt", 2, &dimVett[4]); 
    if (!*deflection_limits) {printf("[!]ERROR: Errore caricamento vettore deflection_limits\n"); system("PAUSE"); exit(0);}
    *fuel_mass = caricaVettoreDouble("dati/dba.txt", 3, &dimVett[5]); 
    if (!*fuel_mass) {printf("[!]ERROR: Errore caricamento vettore fuel_mass\n"); system("PAUSE"); exit(0);}
    
    // MATRICI
    *data_propeller = caricaMatriceDouble("dati/propeller.txt", 4, 3, &dimMat[0]);
    *steady_state_coeff = caricaMatriceDouble("dati/dba.txt", 7, 4, &dimMat[1]);
    *aer_der_x = caricaMatriceDouble("dati/dba.txt", 8, 5, &dimMat[2]);
    *aer_der_y = caricaMatriceDouble("dati/dba.txt", 7, 6, &dimMat[3]);
    *aer_der_z = caricaMatriceDouble("dati/dba.txt", 8, 7, &dimMat[4]);
    *rolling_moment_der = caricaMatriceDouble("dati/dba.txt", 7, 8, &dimMat[5]);
    *pitch_moment_der = caricaMatriceDouble("dati/dba.txt", 8, 9, &dimMat[6]);
    *yawing_moment_der = caricaMatriceDouble("dati/dba.txt", 7, 10, &dimMat[7]);
    *control_force_der = caricaMatriceDouble("dati/dba.txt", 7, 11, &dimMat[8]);
    *control_moment_der = caricaMatriceDouble("dati/dba.txt", 7, 12, &dimMat[9]);
    *rotary_der = caricaMatriceDouble("dati/dba.txt", 7, 13, &dimMat[10]);
    // Controlla che tutte le allocazioni siano andate a buon fine
    if (!*data_propeller || !*steady_state_coeff || !*aer_der_x || !*aer_der_y || !*aer_der_z || !*rolling_moment_der || !*pitch_moment_der || !*yawing_moment_der || !*control_force_der || !*control_moment_der || !*rotary_der) {printf("[!]ERROR: Errore caricamento matrici\n"); system("PAUSE"); exit(0);}
}

void stampaTuttiIDati(double *engine, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *body_axes, double *deflection_limits, double *fuel_mass, double **steady_state_coeff, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **rotary_der) {
    const char *name_vec[] = {"engine", "geometry_propeller", "propeller_profile", "body_axes", "deflection_limits", "fuel_mass"};
    double *vec[] = {engine, geometry_propeller,propeller_profile, body_axes, deflection_limits, fuel_mass};

    FILE *val1 = fopen("Validazione_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt", "w");
    fclose(val1);
    val1 = fopen("Validazione_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt", "a");
    fprintf(val1, "\n\n**********   STAMPA VETTORI   **********");
    fclose(val1);
    for(int i=0; i<sizeof(vec)/sizeof(vec[0]); ++i){
        stampaVettoreFile(name_vec[i], vec[i], dimVett[i]);
    }

    const char *name_mat[] = {"data_propeller", "steady_state_coeff", "aer_der_x", "aer_der_y", "aer_der_z", "rolling_moment_der", "pitch_moment_der", "yawing_moment_der", "control_force_der", "control_moment_der", "rotary_der"};
    double **mat[] = {data_propeller, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, rolling_moment_der, pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, rotary_der};

    val1 = fopen("Validazione_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt", "a");
    fprintf(val1, "\n\n\n**********   STAMPA MATRICI   **********");
    fclose(val1);
    for(int i=0; i<sizeof(mat)/sizeof(mat[0]); ++i){
        stampaMatriceFile(name_mat[i], mat[i], dimMat[i], (i==0) ? 4 : (i==2 || i==4 || i==6) ? 8 : 7);
    }
}

// Funzione per liberare la memoria di tutti i dati caricati
void liberaTuttiIDati(double *engine, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *body_axes, double *deflection_limits, double *fuel_mass, double **steady_state_coeff, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **rotary_der, double **state, double **command) {
    free(engine); free(geometry_propeller); free(propeller_profile); free(body_axes); free(deflection_limits); free(fuel_mass);
    // Libera matrici dinamiche
    double** matrici[] = {
        data_propeller, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z,
        rolling_moment_der, pitch_moment_der, yawing_moment_der,
        control_force_der, control_moment_der, rotary_der, state, command
    };
    int numMatrici = sizeof(matrici)/sizeof(matrici[0]);
    for (int m = 0; m < numMatrici; ++m) {
        if (matrici[m]) {
            int nRighe = dimMat[m];
            for (int i = 0; i < nRighe; ++i) {
                free(matrici[m][i]);
            }
            free(matrici[m]);
        }
    }
}