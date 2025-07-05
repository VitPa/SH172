#include <stdio.h>
#include <stdlib.h>
#include "command.h"
#include "EstrazioneDati.h"

static double RPMt;
static double et;
static int n;

double** load_command(double dt, double Tfs, double RPMtrim, double eTrim){
    int choise;
    RPMt = RPMtrim;
    et = eTrim;
    n = (int)(Tfs/dt);
    
    double **command = malloc(n * sizeof(double*));
    if (!command) return NULL;
    for (int i = 0; i < n; ++i) {
        command[i] = calloc(4, sizeof(double));
        if (!command[i]) {
            for (int j = 0; j < i; ++j) free(command[j]);
            free(command);
            return NULL;
        }
    }
    dimMat[12] = n;

    command[0][0] = 0.0;
    command[0][1] = eTrim;
    command[0][2] = 0.0;
    command[0][3] = RPMtrim;

    printf("(1) per le manovre standard già implementate, (2) per le manovre personalizzate: \n");
    scanf("%d", &choise);  //AGGIUNGERE CONTROLLO SULL'INPUT
    switch (choise){
    case 1:
        defaultManeuver(dt, Tfs, command, RPMtrim, eTrim);
        break;
    case 2:
        customManeuver(dt, Tfs, command);
        break;
    default:
        break;
    }

    return command;
}

void defaultManeuver(double dt, double Tfs, double **command, double RPMtrim, double eTrim){  //Manovre usate per i report
    int maneuver;

    printf("Scegliere la manovra desiderata:\n");
    printf("(1) gradino 2deg alettoni per 5s\n(2) rampa 0-3deg alettoni per 7s\n");
    scanf("%d", &maneuver);

    switch(maneuver){
        case 1:  //ESEMPIO UNA MANOVRA
            zero(dt, Tfs, command, 0);
            zero(dt, Tfs, command, 1);
            zero(dt, Tfs, command, 2);
            zero(dt, Tfs, command, 3);
            break;
    }
}

void customManeuver(double dt, double Tfs, double **command){
    int maneuver;
    double A[4] = {0.0, 0.0, 0.0, 0.0}, Af[4] = {0.0, 0.0, 0.0, 0.0}, start_command[4] = {0.0, 0.0, 0.0, 0.0}, duration_command[4] = {0, 0, 0, 0};
    int stamp[4] = {0, 0, 0, 0};
    const char *signal[4] = {"impulso", "impulso simmetrico", "gradino", "fine rampa"};
    const char *name[4] = {"alettoni", "equilibratore", "timone", "RPM"};

    for(int i = 0; i<4; ++i){
        printf("Modifica comando %s\tAmpiezza attuale %g\n", name[i], apply_trim(0.0, i));

        printf("(1) impulso\n(2) impulso simmetrico\n(3) gradino\n(4) rampa\n(0) nessun comando\n");
        do{
            scanf("%d", &maneuver);
            if(maneuver<0 || maneuver>4){
                printf("[~] WARNING: Inserire un numero compreso tra 1 e 4\n");
                continue;
            }
            break;
        }while(1);

        if(maneuver!=0){
            if(i!=3) printf("Ampiezza %s [-20, 20]: ", signal[maneuver-1]); else printf("Ampiezza %s [0, 100]: ", signal[maneuver-1]);
            scanf("%lf", &A[i]);

            if (i==3){
                if(A[i]<0){
                    A[i]=0; 
                    printf("[~]WARNING: Ampiezza minore del minimo consentito... Impostata ampiezza a 0%%\n");
                }
                if(A[i]>100){
                    A[i]=100; 
                    printf("[~]WARNING: Ampiezza maggiore del massimo consentito... Impostata ampiezza a 100%%\n");
                }
                A[i] = RPMmin + (RPMmax - RPMmin) * (A[i] - 0) / (100 - 0);  //Mappatura manetta [0, 100] -> [RPMmin, RPMmax]
            }else{
                if(A[i]<-20){
                    A[i]=-20; 
                    printf("[~]WARNING: Ampiezza minore del minimo consentito... Impostata ampiezza a -20deg\n");
                }
                if(A[i]>20){
                    A[i]=20; 
                    printf("[~]WARNING: Ampiezza maggiore del massimo consentito... Impostata ampiezza a 20deg\n");
                }
            }
        }

        switch(maneuver){
            case 0: //NIENTE
                zero(dt, Tfs, command, i);

                duration_command[i] = Tfs;
                stamp[i] = 0;
                break;
            case 1:  //IMPULSO
                printf("Tempo inizio comando[0, %g]: \n", Tfs-5*dt);
                start_command[i] = ask_double(0, Tfs-5*dt);

                impulse(apply_trim(A[i], i), start_command[i], dt, Tfs, command, i);

                duration_command[i] = 5*dt;
                stamp[i] = 1;
                break;

            case 2: //IMPULSO SIMMETRICO
                printf("Tempo inizio comando[0, %g]: \n", Tfs-10*dt);
                start_command[i] = ask_double(0, Tfs-10*dt);

                symmetricImpulse(apply_trim(A[i], i), start_command[i], dt, Tfs, command, i);

                duration_command[i] = 10*dt;
                stamp[i] = 2;
                break;

            case 3: //GRADINO
                printf("Durata comando [0, %g]: ", Tfs);
                duration_command[i] = ask_double(0, Tfs);
                
                printf("Tempo inizio comando[0, %g]: ", Tfs-duration_command[i]);
                start_command[i] = ask_double(0, Tfs-duration_command[i]);

                step(apply_trim(A[i], i), start_command[i], duration_command[i], dt, Tfs, command, i);

                stamp[i] = 3;
                break;
                
            case 4: //RAMPA
                printf("Durata comando [0, %g]: ", Tfs);
                duration_command[i] = ask_double(0, Tfs);
                
                printf("Tempo inizio comando[0, %g]: \n", Tfs-duration_command[i]);
                start_command[i] = ask_double(0, Tfs-duration_command[i]);

                ramp(apply_trim(0.0, i), apply_trim(A[i], i), start_command[i], duration_command[i], dt, Tfs, command, i);

                stamp[i] = 4;
                break;
            default:
                printf("[!] ERROR: Nessun comando valido selezionato\n");
                system("PAUSE");
                exit(0);
                break;
        }
    }
        

}

void impulse(double A, double start_command, double dt, double Tfs, double **command, int column){
    int start = (int)(start_command / dt);
    int end = (int)(((start_command)/dt)+5);
    for(int i = 0; i<n; ++i){
        if(i>=start && i<end){
            command[i][column] = A; 
        }else{
            command[i][column] = apply_trim(0.0, column);
        }
    }
}

void symmetricImpulse(double A, double start_command, double dt, double Tfs, double **command, int column){
    int start = (int)(start_command / dt);
    int end1 = (int)(((start_command)/dt)+5);
    int end2 = (int)(((start_command)/dt)+10);
    for(int i = 0; i<n; ++i){
        if(i>=start && i<end1){
            command[i][column] = A; 
        }else if(i>=end1 && i<end2){
            command[i][column] = -A;
        }else{
            command[i][column] = apply_trim(0.0, column);
        }
    }
}

void step(double A, double start_command, double duration_command, double dt, double Tfs, double **command, int column){
    int start = (int)(start_command / dt);
    int end = (int)((start_command + duration_command) / dt);
    for(int i = 0; i<n; ++i){
        if(i>=start && i<end){
            command[i][column] = A; 
        }else{
            command[i][column] = apply_trim(0.0, column);
        }
    }
}

void ramp(double A0, double A1, double start_command, double duration_command, double dt, double Tfs, double **command, int column){
    int start = (int)(start_command / dt);
    int end = (int)((start_command + duration_command) / dt);
    int total_steps = end - start;

    for (int i = 0; i < n; ++i) {
        if (i < start) {
            command[i][column] = A0;
        } else if (i >= start && i <= end && total_steps > 0) {
            double frac = (double)(i - start) / total_steps;
            double val = A0 + frac * (A1 - A0);
            command[i][column] = val;
        } else {
            command[i][column] = A1;
        }
    }
}

void zero(double dt, double Tfs, double **command, int column){
    for(int i = 0; i<n; ++i){
        command[i][column] = apply_trim(0.0, column);
    }
}

//Utility
static inline double apply_trim(double val, int column) {
    return (column == 3) ? (val + RPMt) : ((column == 1) ? (val + et) : val);
}

double ask_double(double min, double max) {
    double val;
    do {
        scanf("%lf", &val);
        if(val < min || val > max) {
            printf("[~]WARNING: Inserire un valore compreso tra %.2f e %.2f\n", min, max);
            continue;
        }
        break;
    } while(1);
    return val;
}