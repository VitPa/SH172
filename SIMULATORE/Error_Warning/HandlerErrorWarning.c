#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "ErrorWarning.h"
#include "../Pre_processing/Variables.h"

typedef struct {
    int code;
    const char* message;
} Err_War_t;

static const Err_War_t error_table[] = {
    // 0  -> Warning error
    {0, "Codice warning sconosciuto (%d).\n"},

    //1xx -> File and variables errors
    {100, "Apertura del file '%s' non riuscita.\n"},
    {101, "Caricamento vettore '%s' non riuscito.\n"},
    {102, "Caricamento matrice '%s' non riuscita.\n"},

    //2xx -> Operating conditions
    {200, "La velocita' e' inferiore della velocita' di stallo.\n"},
    {201, "La velocita' e' maggiore del Mach di drag rise.\n"},
    {202, "La velocita' e' maggiore della velocita' massima.\n"},
    {203, "La quota e' minore di zero.\n"},
    {204, "La quota e' maggiore della quota di tangenza.\n"},
    {205, "La massa di carburante e' scesa sotto la soglia minima consentita.\n"},
    
    //4xx -> Trim and Stability conditions
    {400, "Alpha e/o de di Trim non trovati.\n"},
    {401, "RPM di Trim non trovati. RPM necessari troppo %s.\n"},
    {402, "L'aereo e' staticamente instabile.\n"},
    {403, "L'aereo e' dinamicamente instabile.\n"},

    //5xx -> Commands
    {500, "Nessun comando inserito.\n"},

    //6xx -> Interpolation
    {600, "Alpha inserito '%.2lf' minore del minimo consentito.\n"},
    {601, "Alpha inserito '%.2lf' maggiore del massimo consentito.\n"},
    
    //9xx -> Dynamic memory
    {900, "Inizializzazione memoria dinamica per '%s' non riuscita.\n"},
    {901, "Ri-allocazione memoria dinamica per '%s' non riuscita.\n"},
    {902, "Allocazione memoria dinamica per '%s' non riuscita.\n"}
};

static const Err_War_t warning_table[] = {
    // 0   -> Default
    {0, "Inserito valore di default: %g.\n"},

    // Informational Warnings
    // 1xx -> Commands
    {100, "Ampiezza minore del minimo consentito... Impostata a %g.\n"},
    {101, "Ampiezza maggiore del massimo consentito... Impostata a %g.\n"},

    // 2xx -> Exceeding Operating Conditions
    {200, "La velocita' e' minore della velocita' di stallo... Impostata a %g.\n"},
    {201, "La velocita' e' maggiore del Mach di drag rise... Impostata a %g.\n"},
    {202, "La velocita' e' maggiore della velocita' massima... Impostata a %g.\n"},
    {203, "La quota e' minore di zero... Impostata a %g.\n"},
    {204, "La quota e' maggiore della quota di tangenza... Impostata a %g.\n"},
    {205, "L'angolo di rampa e' minore del minimo consentito... Impostato a %g.\n"},
    {206, "L'angolo di rampa e' maggiore del massimo consentito... Impostato a %g.\n"},

    // Action Required Warnings
    // 5xx -> Generics
    {500, "Parametro non valido. Inserire un valore compreso tra %g e %g: "},
    {501, "Parametro non valido. Inserire un valore positivo: "},
    {503, "Parametro non valido. Inserire un valore maggiore di %g: "},
    {504, "Parametro non valido. Inserire un valore numerico: "},
};


void Error(int code, const char *func_name, ...){
    va_list args;
    va_start(args, func_name);

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    for (size_t i = 0; i < sizeof(error_table)/sizeof(Err_War_t); ++i) {
        if (error_table[i].code == code) {
            SetColor(4);

            printf("\n[!]ERROR: ");
            vprintf(error_table[i].message, args);
            printf("\n");

            fprintf(ew_log, "[%02d-%02d-%04d %02d:%02d:%02d] - [!]ERROR: (%s): ",
                        t->tm_mday, t->tm_mon+1, t->tm_year+1900,
                        t->tm_hour, t->tm_min, t->tm_sec, func_name);
            vfprintf(ew_log, error_table[i].message, args);

            SetColor(15);
            va_end(args);
            system("PAUSE");
            exit(error_table[i].code);
        }
    }
    SetColor(4);
    printf("\n[!]ERROR: codice errore sconosciuto (%d).\n\n", code);
    SetColor(15);
    va_end(args);
    system("PAUSE");
    exit(code);
}

// Scrivere la void Warning(){}
void Warning(int code, const char *func_name, ...){
    int check = 0;
    va_list args;
    va_start(args, func_name);

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    for (size_t i = 0; i < sizeof(warning_table)/sizeof(Err_War_t); ++i) {
        if (warning_table[i].code == code && code != 0) {
            check = 1;
            SetColor(14);

            printf("[~]WARNING: ");
            vprintf(warning_table[i].message, args);

            fprintf(ew_log, "[%02d-%02d-%04d %02d:%02d:%02d] - [~]WARNING (%s): ",
                        t->tm_mday, t->tm_mon+1, t->tm_year+1900,
                        t->tm_hour, t->tm_min, t->tm_sec, func_name);
            vfprintf(ew_log, warning_table[i].message, args);
            if(code > 500) fprintf(ew_log, "\n");

            SetColor(15);
            va_end(args);
        }
        else if (warning_table[i].code == code && code == 0) {
            check = 1;
            SetColor(8);

            printf("[-]DEFAULT: ");
            vprintf(warning_table[i].message, args);

            SetColor(15);
            va_end(args);
        }
    }
    if(!check) MY_ERROR(0, code);
}