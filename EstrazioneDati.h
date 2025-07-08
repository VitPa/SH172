#ifndef ESTRAZIONE_DATI_H
#define ESTRAZIONE_DATI_H

extern double RPMmax, RPMmin;

FILE* apriFile(const char *path, const char *mode);

double* caricaVettoreDouble(const char *path, int checkSection, int *outSize);

double** caricaMatriceDouble(const char *path, int colonne, int checkSection, int *outRighe);

double** reallocState(double **state, int n_colonne);

double** reallocCommand(double **command, int n_colonne);

void stampaVettoreFile(const char* nome, double* v, int n);

void stampaMatriceFile(const char* nome, double** m, int righe, int colonne);

int caricaTuttiIDati(double **engine, double **geometry_propeller, double **propeller_profile, double ***data_propeller, double **body_axes, double **deflection_limits, double **fuel_mass, double ***steady_state_coeff, double ***aer_der_x, double ***aer_der_y, double ***aer_der_z, double ***rolling_moment_der, double ***pitch_moment_der, double ***yawing_moment_der, double ***control_force_der, double ***control_moment_der, double ***rotary_der);

void stampaTuttiIDati(double *engine, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *body_axes, double *deflection_limits, double *fuel_mass, double **steady_state_coeff, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **rotary_der);

void liberaTuttiIDati(double *engine, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *body_axes, double *deflection_limits, double *fuel_mass, double **steady_state_coeff, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **rotary_der, double **state, double **command);

#endif