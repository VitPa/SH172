#ifndef SIMULATORE_ATMOSPHERE_H
#define SIMULATORE_ATMOSPHERE_H

int AtmosphereChoice (double *press0,double *temp0,double *rho0,double *vsuono0,
        double *press_h,double *temp_h,double *rho_h,double *vsuono_h, double *CI, int *flagatm);

int AtmosphereCalc (double *CI, double **datiengine, double *Pmax_h,double *press0,double *temp0,double *rho0,double *vsuono0,
        double *press_h,double *temp_h,double *rho_h,double *vsuono_h, int *flagatm);

#endif //SIMULATORE_ATMOSPHERE_H
