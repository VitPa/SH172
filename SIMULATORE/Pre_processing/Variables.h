#include <stdio.h>

#define g 9.80665
#define pi 3.14159265

// Percorsi file I/O
extern char *path_dba;
extern char *path_engine;
extern char *path_propeller;

extern char *path_v_l_i;
extern char *path_v_p;

extern char *path_data;
extern char *path_com;
extern char *path_agg;
extern char *path_log;

// Puntatori ai file
extern FILE *ew_log;
extern FILE *data;
extern FILE *com;
extern FILE *agg;
extern FILE *val1;
extern FILE *val2;


// variabili globali
extern double Vmax;
extern double Vmin;
extern double Hmin;
extern double Hmax;
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