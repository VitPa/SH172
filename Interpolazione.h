void InterpolazioneCoeff(int stampa, double **body_axes, double *CI, double *temp_h, double mio_alpha,
    double *Int_ssc, double *Int_adx, double *Int_ady, double *Int_adz, double *Int_rmd, 
    double *Int_pmd, double *Int_ymd, double *Int_cfd, double *Int_cmd, double *Int_rd,
    double ***steady_state_coeff, double ***aer_der_x, double ***aer_der_y, 
    double ***aer_der_z, double ***rolling_moment_der, double ***pitch_moment_der, double ***yawing_moment_der, 
    double ***control_force_der, double ***control_moment_der, double ***rotary_der);

    void stampaInterpolazione(double *Int_ssc, double *Int_adx, double *Int_ady, double *Int_adz, double *Int_rmd, 
        double *Int_pmd, double *Int_ymd, double *Int_cfd, double *Int_cmd, double *Int_rd);