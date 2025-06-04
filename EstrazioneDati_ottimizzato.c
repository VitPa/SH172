// EstrazioneDati_ottimizzato.c
// Versione ottimizzata delle funzioni di estrazione dati
#include <stdio.h>
#include <stdlib.h>
#include "EstrazioneDati.h"

// Funzione di utilità per apertura file
static FILE* apriFile(const char* path, const char* mode) {
    FILE* f = fopen(path, mode);
    if (!f) fprintf(stderr, "Errore apertura file: %s\n", path);
    return f;
}

// Esempio di funzione modulare per caricare un vettore di double
double* caricaVettoreDouble(const char* path, int* outSize) {
    FILE* f = apriFile(path, "r");
    if (!f) return NULL;
    double* arr = NULL;
    int n = 0;
    double val;
    char riga[256];
    while (fgets(riga, sizeof(riga), f)) {
        if (riga[0] == '*') continue;
        if (sscanf(riga, "%lf", &val) == 1) {
            arr = realloc(arr, (n+1)*sizeof(double));
            arr[n++] = val;
        }
    }
    fclose(f);
    if (outSize) *outSize = n;
    return arr;
}

// Funzione per caricare una matrice di double da file (righe x colonne)
double** caricaMatriceDouble(const char* path, int colonne, int* outRighe) {
    FILE* f = apriFile(path, "r");
    if (!f) return NULL;
    double** mat = NULL;
    int n = 0;
    char riga[512];
    while (fgets(riga, sizeof(riga), f)) {
        if (riga[0] == '*') continue;
        double* temp = malloc(colonne * sizeof(double));
        int letti = 0;
        char* ptr = riga;
        for (int i = 0; i < colonne; ++i) {
            while (*ptr == '\t' || *ptr == ' ') ++ptr;
            if (sscanf(ptr, "%lf", &temp[i]) == 1) {
                // Avanza ptr alla prossima tabulazione/spazio
                char* next = ptr;
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
    fclose(f);
    if (outRighe) *outRighe = n;
    return mat;
}

// Rialloca la matrice state aggiungendo una riga
double** reallocState(double** state, int* n_righe, int n_colonne) {
    // Aumenta il numero di righe
    int new_rows = *n_righe + 1;
    // Rialloca il vettore di puntatori alle righe
    double** tmp = realloc(state, new_rows * sizeof(double*));
    if (!tmp) {
        fprintf(stderr, "Errore realloc state\n");
        return state; // ritorna la vecchia matrice se fallisce
    }
    // Alloca la nuova riga
    tmp[new_rows - 1] = calloc(n_colonne, sizeof(double));
    if (!tmp[new_rows - 1]) {
        fprintf(stderr, "Errore calloc nuova riga state\n");
        return tmp;
    }
    *n_righe = new_rows;
    return tmp;
}

// Funzione wrapper per caricare tutti i dati richiesti dal simulatore
int caricaTuttiIDati(double **engine, double **geometry_propeller, double **propeller_profile, double ***data_propeller, double **body_axes, double **deflection_limits, double **fuel_mass, double ***steady_state_coeff, double ***aer_der_x, double ***aer_der_y, double ***aer_der_z, double ***rolling_moment_der, double ***pitch_moment_der, double ***yawing_moment_der, double ***control_force_der, double ***control_moment_der, double ***rotary_der) {
    int ok = 1;
    int dummy;
    *engine = caricaVettoreDouble("dati/engine.txt", NULL); if (!*engine) ok = 0;
    *geometry_propeller = caricaVettoreDouble("dati/propeller.txt", NULL); if (!*geometry_propeller) ok = 0;
    *propeller_profile = caricaVettoreDouble("dati/propeller.txt", NULL); // Da separare se necessario
    *body_axes = caricaVettoreDouble("dati/dba.txt", NULL); if (!*body_axes) ok = 0;
    *deflection_limits = caricaVettoreDouble("dati/dba.txt", NULL); // Da separare se necessario
    *fuel_mass = caricaVettoreDouble("dati/dba.txt", NULL); // Da separare se necessario
    // Matrici: i numeri di colonne sono da adattare ai tuoi file
    *data_propeller = caricaMatriceDouble("dati/propeller.txt", 4, &dummy);
    *steady_state_coeff = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    *aer_der_x = caricaMatriceDouble("dati/dba.txt", 8, &dummy);
    *aer_der_y = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    *aer_der_z = caricaMatriceDouble("dati/dba.txt", 8, &dummy);
    *rolling_moment_der = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    *pitch_moment_der = caricaMatriceDouble("dati/dba.txt", 8, &dummy);
    *yawing_moment_der = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    *control_force_der = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    *control_moment_der = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    *rotary_der = caricaMatriceDouble("dati/dba.txt", 7, &dummy);
    // Controlla che tutte le allocazioni siano andate a buon fine
    if (!*data_propeller || !*steady_state_coeff || !*aer_der_x || !*aer_der_y || !*aer_der_z || !*rolling_moment_der || !*pitch_moment_der || !*yawing_moment_der || !*control_force_der || !*control_moment_der || !*rotary_der) ok = 0;
    return ok;
}

// Funzione per liberare la memoria di tutti i dati caricati
void liberaTuttiIDati(double *engine, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *body_axes, double *deflection_limits, double *fuel_mass, double **steady_state_coeff, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **rotary_der, double **state) {
    free(engine); free(geometry_propeller); free(propeller_profile); free(body_axes); free(deflection_limits); free(fuel_mass);
    // Libera matrici dinamiche
    double*** matrici[] = {&data_propeller, &steady_state_coeff, &aer_der_x, &aer_der_y, &aer_der_z, &rolling_moment_der, &pitch_moment_der, &yawing_moment_der, &control_force_der, &control_moment_der, &rotary_der, &state};
    int numMatrici = sizeof(matrici)/sizeof(matrici[0]);
    for (int m = 0; m < numMatrici; ++m) {
        if (*matrici[m]) {
            // Libera ogni riga
            int i = 0;
            while ((*matrici[m])[i]) {
                free((*matrici[m])[i]);
                ++i;
            }
            free(*matrici[m]);
        }
    }
}
