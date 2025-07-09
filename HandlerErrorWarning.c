#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ErrorWarning.h"

typedef struct {
    int code;
    const char* message;
} Error_t;

static const Error_t error_table[] = {
    //1xx -> Errori per file e variabili
    {100, "Apertura del file '%s' non riuscita.\n\n"},
    {101, "Caricamento vettore '%s' non riuscito.\n\n"},
    {102, "Caricamento matrice '%s' non riuscita.\n\n"},
    //9xx -> Errori per memoria dinamica
    {900, "Inizializzazione memoria dinamica per '%s' non riuscita.\n\n"},
    {902, "Riallocazione memoria dinamica per '%s' non riuscita.\n\n"},
};

void Error(int code, const char* other_msg){
    for (size_t i = 0; i < sizeof(error_table)/sizeof(Error_t); ++i) {
            if (error_table[i].code == code) {
                SetColor(4);
                if (strstr(error_table[i].message, "%s") && other_msg != NULL){
                    printf("\n[!]ERROR: ");
                    printf(error_table[i].message, other_msg);
                }else{
                    printf("\n[!]ERROR: %s", error_table[i].message);
                }
                SetColor(15);
                system("PAUSE");
                exit(error_table[i].code);
            }
        }
        SetColor(4);
        printf("\n[!]ERROR: codice errore sconosciuto (%d).\n\n", code);
        SetColor(15);
        system("PAUSE");
        exit(code);
}