#ifndef INITIALCONDITION_H
#define INITIALCONDITION_H

void endSection();

void checkVelAlt(double *V, double *h, double *gamma);
void physicalCheck(double V, double h, double m, double Mdg, double vsuono_h);

void loadCI(double *CI);

void openFiles();
void closeFiles();

#endif