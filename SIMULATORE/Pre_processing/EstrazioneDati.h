#ifndef ESTRAZIONE_DATI_H
#define ESTRAZIONE_DATI_H

FILE* apriFile(const char *path, const char *mode);

double* caricaVettoreDouble(const char *path, int checkSection, int *outSize);
double** caricaMatriceDouble(const char *path, int colonne, int checkSection, int *outRighe);

double** reallocCommand(double **command, int n_colonne);

void stampaVettoreFile(const char* nome, double* v, int n);
void stampaMatriceFile(const char* nome, double** m, int righe, int colonne);

void caricaTuttiIDati();
void stampaTuttiIDati();
void liberaTuttiIDati();

#endif