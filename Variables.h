#include <stdio.h>
#include "EstrazioneDati.h"

#define g 9.80665
#define pi 3.14159265

// Percorsi file I/O
//char *path_dba = "input_files/DBA.txt";

// Puntatori ai file
extern FILE *ew_log;


/*// variabili globali
const double Vmax = 75;
const double Vmin = 30;
const double Hmin = 0;
const double Hmax = 4116;
double mFuelMin;  //Da inserire in Estrazione dati con = 0.95 * body_axes[0]*/
extern int RPMmin;
extern int RPMmax;
extern int liv_trim;

// variabili file
extern double *engine;
extern double *geometry_propeller, *propeller_profile, **data_propeller;
extern double *body_axes, *deflection_limits, *fuel_mass;
extern double **steady_state_coeff, **aer_der_x, **aer_der_y,**aer_der_z;
extern double **rolling_moment_der, **pitch_moment_der, **yawing_moment_der;
extern double **control_force_der, **control_moment_der, **rotary_der;

extern double *state;
extern double **command;

// variabili atmosferiche
extern double Pmax_h, press_h, temp_h, rho_h, vsuono_h;