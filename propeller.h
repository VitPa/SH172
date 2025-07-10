#ifndef PROP_H
#define PROP_H

double propel(double RPM_ref, double Vel, double* prop, double *Pal);

double massConsumption(double Kc, double Pa, double np, double mass, double time);

#endif