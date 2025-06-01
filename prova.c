#include <stdio.h>
#include <complex.h>

int main() {
    double complex z = 1.0 + 2.0 * I;
    printf("Re = %.2f, Im = %.2f\n", creal(z), cimag(z));
    return 0;
}