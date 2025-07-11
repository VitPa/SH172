#ifndef ESTRAZIONE_DATI_H
#define ESTRAZIONE_DATI_H

FILE* openFile(const char *path, const char *mode);

double* loadVector(const char *path, int checkSection, int *outSize);
double** loadMatrix(const char *path, int colonne, int checkSection, int *outRighe);

double** reallocCommand(double **command, int n_colonne);

void printVector(const char* nome, double* v, int n);
void printMatrix(const char* nome, double** m, int righe, int colonne);

void loadData();
void printData();
void freeData();

#endif