#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

void AtmosphereChoice (double *press_h,double *temp_h,double *rho_h,double *vsuono_h, int *flagatm);

void AtmosphereCalc (double h, double *datiengine, double *Pmax_h,  double *press_h, double *temp_h, double *rho_h, double *vsuono_h, int flagatm);

#endif