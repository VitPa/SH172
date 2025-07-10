#include "Variables.h"

FILE *ew_log = NULL;

int liv_trim = 0;

double *engine = NULL;
double *geometry_propeller = NULL, *propeller_profile = NULL, **data_propeller = NULL;
double *body_axes = NULL, *deflection_limits = NULL, *fuel_mass = NULL;
double **steady_state_coeff = NULL, **aer_der_x = NULL, **aer_der_y = NULL,**aer_der_z = NULL;
double **rolling_moment_der = NULL, **pitch_moment_der = NULL, **yawing_moment_der = NULL;
double **control_force_der = NULL, **control_moment_der = NULL, **rotary_der = NULL;
double **state = NULL;
double **command = NULL;

double Pmax_h, press_h, temp_h, rho_h, vsuono_h;