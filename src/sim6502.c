#include <stdio.h>
#include "config_sim6502.h"

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    printf("hello world, this is version %s\n", SIM6502_VERSION_STR);

    return 0;
}
