#include <stdio.h>
#include <stdlib.h>
#include "Data.h"
#include "../Error_Warning/ErrorWarning.h"
#include "../Pre_processing/Variables.h"

static int dimVett[6];
int dimMat[13];

FILE* openFile(const char *path, const char *mode) {
    FILE* f = fopen(path, mode);
    if (!f) MY_ERROR(100, path);
    return f;
}

double* loadVector(const char *path, int checkSection, int *outSize) {
    char *name_p[] = {"geometry_propeller", "propeller_profile"};
    char *name_d[] = {"body_axes", "deflection_limits", "fuel_mass"};
    FILE* f = openFile(path, "r");
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
                if(!arr) MY_ERROR(901, path!=path_engine?
                    path!=path_propeller?name_d[checkSection-1]:name_p[checkSection-1]:"engine");
                arr[n++] = val;
            }
            flag = 1;
        }
    }
    fclose(f);
    if (outSize) *outSize = n;
    if(!arr) MY_ERROR(101, path!=path_engine?
                    path!=path_propeller?name_d[checkSection-1]:name_p[checkSection-1]:"engine");
    return arr;
}

double** loadMatrix(const char *path, int colonne, int checkSection, int *outRighe) {
    char *name[] = {"steady_state_coeff", "aer_der_x", "aer_der_y", "aer_der_z", 
        "rolling_moment_der", "pitch_moment_der", "yawing_moment_der", "control_force_der", "control_moment_der", "rotary_der"};
    FILE* f = openFile(path, "r");
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
                    if(!mat) MY_ERROR(901, path!=path_propeller?name[checkSection-4]:"data_propeller");
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
    if(!mat) MY_ERROR(102, path!=path_propeller?name[checkSection-4]:"data_propeller");
    return mat;
}

// Rialloca la matrice command aggiungendo una riga
double** reallocCommand(int n_colonne) {
    int new_rows = dimMat[12] + 1;
    
    double **tmp = realloc(command, new_rows * sizeof(double*));
    if (!tmp) MY_ERROR(901, "command");
    
    tmp[new_rows - 1] = calloc(n_colonne, sizeof(double));
    if (!tmp[new_rows - 1]) MY_ERROR(900, "command");
    
    dimMat[12] = new_rows;

    return tmp;
}

void printVector(const char* nome, double* v, int n) {
    fprintf(val1, "\n\n--- %s (vettore, %d elementi) ---\n", nome, n);
    for (int i = 0; i < n; ++i) {
        fprintf(val1, "%.2lf\t", v[i]);
    }
}

void printMatrix(const char* nome, double** m, int righe, int colonne) {
    fprintf(val1, "\n\n--- %s (matrice, %d x %d) ---\n", nome, righe, colonne);
    for (int i = 0; i < righe; ++i) {
        for (int j = 0; j < colonne; ++j) {
            fprintf(val1, "%s%s%.4lf\t", m[i][j]<0 ? "" : " ", m[i][j]<10 ? " " : "", m[i][j]);
        }
        fprintf(val1, "\n");
    }
}

// Funzione wrapper per caricare tutti i dati richiesti dal simulatore
void loadData() {
    dimMat[11]=1;
    dimMat[12]=0;
    
    // *** Section: Load vector from file ***
    engine = loadVector(path_engine, 1, &dimVett[0]); 
    geometry_propeller = loadVector(path_propeller, 1, &dimVett[1]);
    propeller_profile = loadVector(path_propeller, 2, &dimVett[2]); 
    body_axes = loadVector(path_dba, 1, &dimVett[3]); 
    deflection_limits = loadVector(path_dba, 2, &dimVett[4]); 
    fuel_mass = loadVector(path_dba, 3, &dimVett[5]); 
    
    // *** Section: Load matrix from file ***
    data_propeller = loadMatrix(path_propeller, 4, 3, &dimMat[0]);
    steady_state_coeff = loadMatrix(path_dba, 7, 4, &dimMat[1]);
    aer_der_x = loadMatrix(path_dba, 8, 5, &dimMat[2]);
    aer_der_y = loadMatrix(path_dba, 7, 6, &dimMat[3]);
    aer_der_z = loadMatrix(path_dba, 8, 7, &dimMat[4]);
    rolling_moment_der = loadMatrix(path_dba, 7, 8, &dimMat[5]);
    pitch_moment_der = loadMatrix(path_dba, 8, 9, &dimMat[6]);
    yawing_moment_der = loadMatrix(path_dba, 7, 10, &dimMat[7]);
    control_force_der = loadMatrix(path_dba, 7, 11, &dimMat[8]);
    control_moment_der = loadMatrix(path_dba, 7, 12, &dimMat[9]);
    rotary_der = loadMatrix(path_dba, 7, 13, &dimMat[10]);

    // *** Section: Set RPM limits ***
    RPMmin = (int)engine[2];
    RPMmax = (int)engine[3];
}

void printData() {    
    const char *name_vec[] = {"engine", "geometry_propeller", "propeller_profile", "body_axes", "deflection_limits", "fuel_mass"};
    double *vec[] = {engine, geometry_propeller,propeller_profile, body_axes, deflection_limits, fuel_mass};

    fprintf(val1, "**********   STAMPA VETTORI   **********");
    for(int i=0; i<sizeof(vec)/sizeof(vec[0]); ++i){
        printVector(name_vec[i], vec[i], dimVett[i]);
    }

    const char *name_mat[] = {"data_propeller", "steady_state_coeff", "aer_der_x", "aer_der_y", "aer_der_z", 
        "rolling_moment_der", "pitch_moment_der", "yawing_moment_der", "control_force_der", "control_moment_der", "rotary_der"};
    double **mat[] = {data_propeller, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z, 
        rolling_moment_der, pitch_moment_der, yawing_moment_der, control_force_der, control_moment_der, rotary_der};

    fprintf(val1, "\n\n\n**********   STAMPA MATRICI   **********");
    for(int i=0; i<sizeof(mat)/sizeof(mat[0]); ++i){
        printMatrix(name_mat[i], mat[i], dimMat[i], (i==0) ? 4 : (i==2 || i==4 || i==6) ? 8 : 7);
    }
    fflush(val1);
}

// Funzione per liberare la memoria di tutti i dati caricati
void freeData() {
    free(engine); free(geometry_propeller); free(propeller_profile); free(body_axes); free(deflection_limits); free(fuel_mass); free(state);
    double** matrici[] = {
        data_propeller, steady_state_coeff, aer_der_x, aer_der_y, aer_der_z,
        rolling_moment_der, pitch_moment_der, yawing_moment_der,
        control_force_der, control_moment_der, rotary_der, command
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