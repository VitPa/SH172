#include "Variables.h"

char *path_dba = "_input_files/DBA.txt";
char *path_engine = "_input_files/engine.txt";
char *path_propeller = "_input_files/propeller.txt";

char *path_v_l_i = "_validation_output/VALIDAZIONE_LETTURA_INTERPOLAZIONE.txt";
char *path_v_p = "_validation_output/VALIDAZIONE_PROPELLER.txt";

char *path_data = "_output_files/DATA.txt";
char *path_com = "_output_files/COMMAND.txt";
char *path_agg = "_output_files/EXTRA.txt";
char *path_log = "_output_files/LOG.txt";

FILE *ew_log = NULL;
FILE *data = NULL;
FILE *com = NULL;
FILE *agg = NULL;
FILE *val1 = NULL;
FILE *val2 = NULL;

int RPMmin = -1;
int RPMmax = -1;
int liv_trim = 0;

double *engine = NULL;
double *geometry_propeller = NULL, *propeller_profile = NULL, **data_propeller = NULL;
double *body_axes = NULL, *deflection_limits = NULL, *fuel_mass = NULL;
double **steady_state_coeff = NULL, **aer_der_x = NULL, **aer_der_y = NULL,**aer_der_z = NULL;
double **rolling_moment_der = NULL, **pitch_moment_der = NULL, **yawing_moment_der = NULL;
double **control_force_der = NULL, **control_moment_der = NULL, **rotary_der = NULL;
double *state = NULL;
double **command = NULL;

double Pmax_h, press_h, temp_h, rho_h, vsuono_h;