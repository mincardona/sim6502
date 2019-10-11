#include "config_sim6502.h"
#include <stdio.h>

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    printf("hello world, this is version %s\n", SIM6502_VERSION_STR);

    return 0;
}
