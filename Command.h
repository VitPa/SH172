double** load_command(double dt, double Tfs, double RPMtrim, double eTrim);
void defaultManeuver(double dt, double Tfs, double **command);
void customManeuver(double dt, double Tfs, double **command);
void step(double A, double start_command, double duration_command, double dt, double Tfs, double **command, int column);
void zero(double dt, double Tfs, double **command, int column);
void impulse(double A, double start_command, double dt, double Tfs, double **command, int column);
void symmetricImpulse(double A, double start_command, double dt, double Tfs, double **command, int column);
void ramp(double A0, double A1, double start_command, double duration_command, double dt, double Tfs, double **command, int column);

static inline double apply_trim(double val, int column);
double ask_double(double min, double max);