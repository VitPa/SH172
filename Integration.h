#ifndef INTEGRAZIONE_H_
#define INTEGRAZIONE_H_

void eulerEquation(int i, double **state, double **command, double rho, double RPM, double *body_axes, double **steady_state_coefficients, double **aer_der_x, double **aer_der_y, double **aer_der_z, double **rolling_moment_der, double **pitch_moment_der, double **yawing_moment_der, double **control_force_der, double **control_moment_der, double **geometry_propeller, double **propeller_profile, double **data_propeller);

#endif