#ifndef COMMAND_H
#define COMMAND_H

double** load_command(double dt, double Tfs, double RPMtrim, double eTrim);
void defaultManeuver(double dt, double Tfs, double **command);
void customManeuver(double dt, double Tfs, double **command);

void zero(double dt, double Tfs, double **command, int column);
void impulse(double A, double start_command, double dt, double Tfs, double **command, int column, int l);
void symmetricImpulse(double A, double start_command, double dt, double Tfs, double **command, int column, int l);
void ramp(double A0, double A1, double start_command, double duration_command, double dt, double Tfs, double **command, int column, int l);
void step(double A, double start_command, double duration_command, double dt, double Tfs, double **command, int column, int l);

static inline double apply_trim(double val, int column);
double ask_double(double min, double max);
char check_choice();

#endif