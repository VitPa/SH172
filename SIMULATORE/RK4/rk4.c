#include <stdlib.h>
#include "rk4.h"

void rk4_step(double *state, int N, double dt, double t, int i, void (*derivs)(const double *, double *, double, int)) {
    double *k1 = (double*)malloc(N * sizeof(double));
    double *k2 = (double*)malloc(N * sizeof(double));
    double *k3 = (double*)malloc(N * sizeof(double));
    double *k4 = (double*)malloc(N * sizeof(double));
    double *temp = (double*)malloc(N * sizeof(double));

    derivs(state, k1, t, i);
    for (int j = 0; j < N; ++j)
        temp[j] = state[j] + 0.5 * dt * k1[j];
    derivs(temp, k2, t + 0.5 * dt, i);
    for (int j = 0; j < N; ++j)
        temp[j] = state[j] + 0.5 * dt * k2[j];
    derivs(temp, k3, t + 0.5 * dt, i);
    for (int j = 0; j < N; ++j)
        temp[j] = state[j] + dt * k3[j];
    derivs(temp, k4, t + dt, i);
    for (int j = 0; j < N; ++j)
        state[j] += (dt / 6.0) * (k1[j] + 2*k2[j] + 2*k3[j] + k4[j]);

    free(k1); free(k2); free(k3); free(k4); free(temp);
}
