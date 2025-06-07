#ifndef PROP_H
#define PROP_H

double propel(double RPM_ref, double Pmax_h, double rho1, double Vel, double* geometry_propeller, double* propeller_profile, double** data_propeller, double* prop, double *Pal);

double massConsumption(double Kc, double Pa, double np, double mass, double time);

#endif