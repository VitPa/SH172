#include <stdio.h>
#include <stdlib.h>
#include "Command.h"

int main(){
    double **command = load_command(0.01, 2, 2500, 0.1);
    int n = 2/0.01;
    for (int i = 0; i < n; ++i) {
        printf("%.2lf  -  ", i*0.01);
        for (int j = 0; j < 4; ++j) {
            printf("%g ", command[i][j]);
        }
        printf("\n");
    }
    return 0;
}