#include <stdio.h>
#include <stdlib.h>
#include "command.h"
#include "EstrazioneDati.h"

static double manettat;
static double et;
static int n;

double** load_command(double dt, double Tfs, double RPMtrim, double eTrim){
    int choise;
    manettat = 100 * (RPMtrim - RPMmin) / (RPMmax - RPMmin);
    et = eTrim;
    n = ((int)(Tfs/dt))+1;
    
    double **command = NULL;
    for(int i = 0; i<n; ++i){
        command = reallocCommand(command, 4);
    }

    printf("(1) per le manovre standard già implementate, (2) per le manovre personalizzate: \n");
    scanf("%d", &choise);  //AGGIUNGERE CONTROLLO SULL'INPUT
    switch (choise){
    case 1:
        defaultManeuver(dt, Tfs, command);
        break;
    case 2:
        customManeuver(dt, Tfs, command);
        break;
    default:
        break;
    }

    return command;
}

void defaultManeuver(double dt, double Tfs, double **command){  //Manovre usate per i report
    int maneuver;

    printf("Scegliere la manovra desiderata:\n");
    printf("(1) Volo livellato\n(2) rampa 0-3deg alettoni per 7s\n");
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
    char *signal[4] = {"impulso", "impulso simmetrico", "gradino", "rampa"};
    char *name[4] = {"alettoni", "equilibratore", "timone", "manetta"};

    for(int i = 0; i<4; ++i){
        system("cls");
        int l = 0;
        double A = 0.0, start_command= 0.0, duration_command = 0.0, old_A, old_start, old_dur;
        char *old_signal;
        printf("Modifica comando %s \tAmpiezza attuale %g\n", name[i], apply_trim(0.0, i));
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
            printf("(0) nessun comando\n(1) impulso\n(2) impulso simmetrico\n(3) gradino\n(4) rampa\n");
            do{
                scanf("%d", &maneuver);
                if(maneuver<0 || maneuver>4){
                    printf("[~] WARNING: Inserire un numero compreso tra 1 e 4\n");
                    continue;
                }
                break;
            }while(1);

            if(maneuver!=0){
                if(i==3) printf("Ampiezza %s [%g, %g]: ", signal[maneuver-1], 0-manettat, 1-manettat); 
                else if(i==1) printf("Ampiezza %s [%g, %g]: ", signal[maneuver-1], -20-et, 20-et);
                else printf("Ampiezza %s [-20, 20]: ", signal[maneuver-1]);
                scanf("%lf", &A);

                if (i==3){
                    if(A+manettat<0){
                        A=0-manettat; 
                        printf("[~]WARNING: Ampiezza minore del minimo consentito... Impostata ampiezza a 0%%\n");
                    }
                    if(A+manettat>1){
                        A=1-manettat; 
                        printf("[~]WARNING: Ampiezza maggiore del massimo consentito... Impostata ampiezza a 100%%\n");
                    }
                }else if (i==1){
                    if(A+et<-20){
                        A=-20-et; 
                        printf("[~]WARNING: Ampiezza minore del minimo consentito... Impostata ampiezza a %gdeg\n", -20-et);
                    }
                    if(A+et>20){
                        A=20-et; 
                        printf("[~]WARNING: Ampiezza maggiore del massimo consentito... Impostata ampiezza a %gdeg\n", 20-et);
                    }
                }else{
                    if(A<-20){
                        A=-20; 
                        printf("[~]WARNING: Ampiezza minore del minimo consentito... Impostata ampiezza a -20deg\n");
                    }
                    if(A>20){
                        A=20; 
                        printf("[~]WARNING: Ampiezza maggiore del massimo consentito... Impostata ampiezza a 20deg\n");
                    }
                }
            }

            switch(maneuver){
                case 0: // ZERO COMMAND
                    zero(dt, Tfs, command, i);
                    l = 1;
                    break;
                case 1:  // IMPLUSE
                    if(l>0) printf("Tempo inizio comando (precedente -> %s [%g, %g]) [0, %g]: \n", old_signal, old_start, old_start+old_dur, Tfs-5*dt);
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-5*dt);
                    start_command = ask_double(0, Tfs-5*dt);

                    impulse(apply_trim(A, i), start_command, dt, Tfs, command, i, l);

                    duration_command = 5*dt;
                    break;

                case 2: //IMPULSO SIMMETRICO
                    if(l>0) printf("Tempo inizio comando (precedente -> %s [%g, %g]) [0, %g]: \n",old_signal, old_start, old_start+old_dur, Tfs-10*dt);
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-10*dt);
                    start_command = ask_double(0, Tfs-10*dt);

                    symmetricImpulse(apply_trim(A, i), start_command, dt, Tfs, command, i, l);

                    duration_command = 10*dt;
                    break;

                case 3: //GRADINO
                    if(l>0) printf("Tempo durata comando (precedente -> %s [%g]) [0, %g]: \n",old_signal, old_dur, Tfs);
                    else printf("Tempo durata comando [0, %g]: ", Tfs);
                    duration_command = ask_double(0, Tfs);
                    
                    if(l>0) printf("Tempo inizio comando (precedente -> %s [%g, %g]) [0, %g]: \n",old_signal, old_start, old_start+old_dur, Tfs-duration_command);
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-duration_command);
                    start_command = ask_double(0, Tfs-duration_command);

                    step(apply_trim(A, i), start_command, duration_command, dt, Tfs, command, i, l);
                    break;
                    
                case 4: //RAMPA
                    if(l>0) printf("Tempo durata comando (precedente -> %s [%g]) [0, %g]: \n",old_signal, old_dur, Tfs);
                    else printf("Tempo durata comando [0, %g]: ", Tfs);
                    duration_command = ask_double(0, Tfs);
                    
                    if(l>0) printf("Tempo inizio comando (precedente -> %s [%g, %g]) [0, %g]: \n",old_signal, old_start, old_start+old_dur, Tfs-duration_command);
                    else printf("Tempo inizio comando [0, %g]: ", Tfs-duration_command);
                    start_command = ask_double(0, Tfs-duration_command);

                    ramp(apply_trim(0.0, i), apply_trim(A, i), start_command, duration_command, dt, Tfs, command, i, l);
                    break;
                default:
                    printf("[!] ERROR: Nessun comando valido selezionato\n");
                    system("PAUSE");
                    exit(0);
                    break;
            }

            if(++l<=1){
                char choice = check_choice();
                if(choice == 'y' || choice == 'Y') continue;
                break;
            };
        }while(l<=1);
    }
}

void impulse(double A, double start_command, double dt, double Tfs, double **command, int column, int l){
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

void symmetricImpulse(double A, double start_command, double dt, double Tfs, double **command, int column, int l){
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

void step(double A, double start_command, double duration_command, double dt, double Tfs, double **command, int column, int l){
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

void ramp(double A00, double A1, double start_command, double duration_command, double dt, double Tfs, double **command, int column, int l){
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

void zero(double dt, double Tfs, double **command, int column){
    for(int i = 0; i<n; ++i){
        command[i][column] = apply_trim(0.0, column);
    }
}

//Utility
static inline double apply_trim(double val, int column) {
    return (column == 1) ? (val + et) : (column == 3) ? (val + manettat) : val;
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

char check_choice(){
    printf("Inserire altri segnali per questo comando? [n/y] no/si: ");
    char choice;
    do{
        scanf(" %c", &choice);
        if(choice !='n' && choice !='N' && choice !='y' && choice !='Y'){
            printf("Valore non valido... Inserire [n/y] no/si: ");
            continue;
        }
        break;
    }while(1);
}