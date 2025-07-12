#include <stdio.h>
#include <stdlib.h>
#include "Command.h"
#include "../Error_Warning/ErrorWarning.h"
#include "../Error_Warning/InitialCondition.h"
#include "../Pre_processing/Variables.h"
#include "../Pre_processing/Data.h"

static double throttle;
static double et;
static int n;

void load_command(double dt, double Tfs, double RPMtrim, double eTrim){
    int choice;
    
    throttle = (RPMtrim - RPMmin) / (RPMmax - RPMmin);              // Compute mapped throttle value
    et = eTrim;
    n = ((int)(Tfs/dt))+1;
    
    
    for(int i = 0; i<n; ++i){                                       // Allocate command matrix for each time step
        command = reallocCommand(4);
    }

    printf("\n(1) per le manovre standard gia' implementate, (2) per le manovre personalizzate: \n");
    do{
        scanf("%d", &choice);
        if(choice != 1 && choice != 2){
            WARNING(500, 1.0, 2.0);
            continue;
        }
        break;
    }while(1);

    switch (choice){
        case 1:
            defaultManeuver(dt, Tfs);
            break;
        case 2:
            customManeuver(dt, Tfs);
            break;
    }
}

void defaultManeuver(double dt, double Tfs){
    int maneuver;
    int nDefaultManeuver = 2;
    const char *maneuverName[] = {"Volo livellato", "Prova mia"};

    printf("\nScegliere la manovra desiderata:\n");
    for(int i = 0; i<nDefaultManeuver; ++i){
        printf("(%d) %s\n", i+1, maneuverName[i]);
    }
    do{
        scanf("%d", &maneuver);
        if(maneuver<0 || maneuver>nDefaultManeuver){
            WARNING(500, 1.0, (double)nDefaultManeuver);
            continue;
        }
        break;
    }while(1);

    switch(maneuver){
        case 1:  // Level Flight
            zero(dt, Tfs, 0);
            zero(dt, Tfs, 1);
            zero(dt, Tfs, 2);
            zero(dt, Tfs, 3);
            break;
        case 2:  // First simple maneuver
            zero(dt, Tfs, 0);
            ramp(apply_trim(0.0, 1), apply_trim(-2, 1), 400, 2, dt, Tfs, 1, 0);
            ramp(apply_trim(0.0, 1), apply_trim(0, 1), 403, 2, dt, Tfs, 1, 1);
            zero(dt, Tfs, 2);
            ramp(apply_trim(0.0, 3), apply_trim(0.6, 3), 50, 10, dt, Tfs, 3, 0);
            break;
    }
}

void customManeuver(double dt, double Tfs){
    int maneuver;
    char *signal[5] = {"impulso", "impulso simmetrico", "gradino", "gradino con durata", "rampa"};
    char *name[4] = {"alettoni", "equilibratore", "timone", "manetta"};
    int n = sizeof(signal)/sizeof(signal[0]);

    for(int i = 0; i<4; ++i){
        
        startSection(5);

        int l = 0;
        double A = 0.0, start_command= 0.0, duration_command = 0.0, old_A, old_start, old_dur;
        char *old_signal;
        printf("Modifica comando %s \t\tValore attuale %g\n", name[i], apply_trim(0.0, i));
        do{
            if(l>0){
                printf("\nModifica aggiuntiva comando %s\n", name[i]);
                old_signal = signal[maneuver-1];
                old_A = A;
                old_start = start_command;
                old_dur = duration_command;
                printf("Comando attualmente presente: \n");
                printf(" | ** %s **\n", old_signal);
                printf(" | Ampiezza: %g\n", old_A);
                printf(" | Durata: [%g, %g]\n", old_start, old_start+old_dur);
                printf("___________________________________________\n");
            }
            for(int j = 0; j<n; ++j){
                printf("(%d) %s\n", j+1, signal[j]);
            }
            do{
                scanf("%d", &maneuver);
                if(maneuver<0 || maneuver>n){
                    WARNING(500, 1, n);
                    continue;
                }
                break;
            }while(1);

            if(maneuver!=0){
                if(i==3) printf("Ampiezza %s [%g, %g]: ", signal[maneuver-1], 0-throttle, 1-throttle); 
                else if(i==1) printf("Ampiezza %s [%g, %g]: ", signal[maneuver-1], -20-et, 20-et);
                else printf("Ampiezza %s [-20, 20]: ", signal[maneuver-1]);
                scanf("%lf", &A);

                if (i==3){
                    if(A+throttle<0){
                        A=0-throttle; 
                        WARNING(100, 0-throttle);
                    }
                    if(A+throttle>1){
                        A=1-throttle; 
                        WARNING(101, 1-throttle);
                    }
                }else if (i==1){
                    if(A+et<-20){
                        A=-20-et; 
                        WARNING(100, -20-et);
                    }
                    if(A+et>20){
                        A=20-et; 
                        WARNING(101, 20-et);
                    }
                }else{
                    if(A<-20){
                        A=-20; 
                        WARNING(100, (double)-20);
                    }
                    if(A>20){
                        A=20; 
                        WARNING(100, (double)20);
                    }
                }
            }

            switch(maneuver){
                case 0:                 // ZERO COMMAND
                    zero(dt, Tfs, i);
                    l = 1;
                    break;
                case 1:                 // IMPLUSE
                    if(l>0) {
                        printf("Tempo inizio comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g, %g])", old_signal, old_start, old_start+old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs-5*dt);
                    }
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-5*dt);
                    start_command = ask_double(0, Tfs-5*dt);

                    impulse(apply_trim(A, i), start_command, dt, Tfs, i, l);

                    duration_command = 5*dt;
                    break;

                case 2:                 // SYMMITRIC IMPULSE
                    if(l>0) {
                        printf("Tempo inizio comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g, %g])", old_signal, old_start, old_start+old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs-10*dt);
                    }
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-10*dt);
                    start_command = ask_double(0, Tfs-10*dt);

                    symmetricImpulse(apply_trim(A, i), start_command, dt, Tfs, i, l);

                    duration_command = 10*dt;
                    break;

                case 3:                 // STEP
                    if(l>0) {
                        printf("Tempo inizio comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g, %g])", old_signal, old_start, old_start+old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs);
                    }
                    else printf("Tempo inizio comando [0, %g]: ", Tfs);
                    start_command = ask_double(0, Tfs);

                    step(apply_trim(A, i), start_command, dt, Tfs, i, l);
                    break;

                case 4:                 // STEP SHORT
                    if(l>0) {
                        printf("Tempo durata comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g])", old_signal, old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs-5*dt);
                    }
                    else printf("Tempo durata comando [0, %g]: ", Tfs);
                    duration_command = ask_double(0, Tfs);
                    
                    if(l>0) {
                        printf("Tempo inizio comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g, %g])", old_signal, old_start, old_start+old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs-duration_command);
                    }
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-duration_command);
                    start_command = ask_double(0, Tfs-duration_command);

                    stepShort(apply_trim(A, i), start_command, duration_command, dt, Tfs, i, l);
                    break;
                    
                case 5:                 // RAMP
                    if(l>0) {
                        printf("Tempo durata comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g])", old_signal, old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs-5*dt);
                    }
                    else printf("Tempo durata comando [0, %g]: ", Tfs);
                    duration_command = ask_double(0, Tfs);
                    
                    if(l>0) {
                        printf("Tempo inizio comando ");
                        SetColor(8);
                        printf("(precedente -> %s [%g, %g])", old_signal, old_start, old_start+old_dur);
                        SetColor(15);
                        printf(" [0, %g]: ", Tfs-duration_command);
                    }
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-duration_command);
                    start_command = ask_double(0, Tfs-duration_command);

                    ramp(apply_trim(0.0, i), apply_trim(A, i), start_command, duration_command, dt, Tfs, i, l);
                    break;
                
                default:
                    MY_ERROR(500);
            }

            if(++l<=1){                             // Check if the user wants to add another command or if there are just two signals
                char choice = check_choice();
                if(choice == 1) continue;
                break;
            };
        }while(l<=1);
    }
    endSection();
}

void impulse(double A, double start_command, double dt, double Tfs, int column, int l){
    int start = (int)(start_command / dt);
    int end = (int)(((start_command)/dt)+5);
    for(int i = 0; i<n; ++i){
        if(i>=start && i<end){
            command[i][column] = A; 
        }else{
            command[i][column] = l==0 ? apply_trim(0.0, column) : command[i][column];
        }
    }
}

void symmetricImpulse(double A, double start_command, double dt, double Tfs, int column, int l){
    int start = (int)(start_command / dt);
    int end1 = (int)(((start_command)/dt)+5);
    int end2 = (int)(((start_command)/dt)+10);
    for(int i = 0; i<n; ++i){
        if(i>=start && i<end1){
            command[i][column] = A; 
        }else if(i>=end1 && i<end2){
            command[i][column] = -A;
        }else{
            command[i][column] = l==0 ? apply_trim(0.0, column) : command[i][column];
        }
    }
}

void stepShort(double A, double start_command, double duration_command, double dt, double Tfs, int column, int l){
    int start = (int)(start_command / dt);
    int end = (int)((start_command + duration_command) / dt);
    for(int i = 0; i<n; ++i){
        if(i>=start && i<end){
            command[i][column] = A; 
        }else{
            command[i][column] = l==0 ? apply_trim(0.0, column) : command[i][column];
        }
    }
}

void step(double A, double start_command, double dt, double Tfs, int column, int l){
    int start = (int)(start_command / dt);
    for(int i = 0; i<n; ++i){
        if(i>=start){
            command[i][column] = A; 
        }else{
            command[i][column] = l==0 ? apply_trim(0.0, column) : command[i][column];
        }
    }
}

void ramp(double A00, double A1, double start_command, double duration_command, double dt, double Tfs, int column, int l){
    int start = (int)(start_command / dt);
    int end = (int)((start_command + duration_command) / dt);
    int total_steps = end - start;

    double A0 = l==0 ? A00 : command[start][column];
    for (int i = 0; i <n; ++i) {
        if (i < start) {
            command[i][column] = l==0 ? A00 : command[i][column];
        } else if (i >= start && i <= end && total_steps > 0) {
            double frac = (double)(i - start) / total_steps;
            double val = A0 + frac * (A1 - A0);
            command[i][column] = val;
        } else {
            command[i][column] = A1;
        }
    }
}

void zero(double dt, double Tfs, int column){
    for(int i = 0; i<n; ++i){
        command[i][column] = apply_trim(0.0, column);
    }
}

//Utility
static inline double apply_trim(double val, int column) {
    return (column == 1) ? (val + et) : (column == 3) ? (val + throttle) : val;
}

double ask_double(double min, double max) {
    double val;
    do {
        scanf("%lf", &val);
        if(val < min || val > max) {
            WARNING(500, min, max);
            continue;
        }
        break;
    } while(1);
    return val;
}

char check_choice(){
    printf("Inserire altri segnali per questo comando? [0=No, 1=Si]: ");
    int choice;
    do{
        scanf("%d", &choice);
        if(choice !=0 && choice !=1){
            WARNING(500, 0, 1.0);
            continue;
        }
        break;
    }while(1);
}