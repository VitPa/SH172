#ifndef INITIALCONDITION_H
#define INITIALCONDITION_H

void checkVelAlt(double *V, double *h, double *gamma);

void physicalCheck(double V, double h, double Mdg, double vsuono_h);

void loadCI(double *CI);

#endif