#ifndef RK4_H
#define RK4_H

void rk4_step(double *state, int N, double dt, double t, int i, void (*derivs)(const double *, double *, double, int));

void compute_derivatives(const double *state, double *dstatedt, double t, int i);

#endif // RK4_H
