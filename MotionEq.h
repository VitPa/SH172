#ifndef MOTIONEQ_H
#define MOTIONEQ_H

void equation(double *engine, double Pmax_h, double rho_h, double *CI, double ***vett_stato, double *body_axes, double **aer_der_x, double **aer_der_z, double **steady_state_coeff, double **control_force_der, double **control_moment_der, double **pitch_moment_der, double *geometry_propeller, double *propeller_profile, double **data_propeller, double *trim);

#endif