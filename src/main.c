/*
 * sim6502 - MOS 6502 simulator
 *
 * Copyright (C) 2019 Michael Incardona
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim6502.h"
#include "config_sim6502.h"

int basic_read_fn(void* pctx, uint16_t addr, uint8_t* out_byte) {
    uint8_t* mem_buf = pctx;
    *out_byte = mem_buf[addr];
    printf("=== READ $%.2x from $%.4x\n", *out_byte, addr);
    return E6502_OK;
}

int basic_write_fn(void* pctx, uint16_t addr, uint8_t in_byte) {
    uint8_t* mem_buf = pctx;
    mem_buf[addr] = in_byte;
    printf("=== WRIT $%.2x from $%.4x\n", in_byte, addr);
    return E6502_OK;
}

int main(int argc, char** argv) {
    struct m6502_machine_config config;
    uint8_t* mem_buf = NULL;
    struct m6502_machine* machine = NULL;
    int err_code = 0;
    struct m6502_reg_bank reg_bank;

    (void)argc;
    (void)argv;

    printf("hello world, this is version %s\n", SIM6502_VERSION_STR);

    mem_buf = calloc(M6502_ADDR_SPACE_SIZE, 1);
    if (!mem_buf)
    {
        printf("error: unable to allocate main memory buffer\n");
        goto ERROR;
    }

    /* test load point */
    mem_buf[0x03] = 0xAF;
    /* starting vector = 0x0200 */
    mem_buf[M6502_RES_PC_ADDR] = 0x00;
    mem_buf[M6502_RES_PC_ADDR + 1] = 0x02;
    mem_buf[0x0200] = 0xA5;  /* LDA zero-page */
    mem_buf[0x0201] = 0x03;

    config.flags = M6502_CFG_BUGGY_JMP_INDIRECT;
    config.mem_delegate.pctx = mem_buf;
    config.mem_delegate.read_fn = basic_read_fn;
    config.mem_delegate.write_fn = basic_write_fn;

    machine = m6502_make_machine(&config);
    if (!machine) {
        printf("error: unable to allocate machine memory\n");
        goto ERROR;
    }
    err_code = m6502_get_error(machine);
    if (err_code) {
        printf("error: machine construction failed: %s\n", m6502_strerror(err_code));
        goto ERROR;
    }

    m6502_get_registers(machine, &reg_bank);

    err_code = m6502_fetch_execute(machine);
    if (err_code) {
        printf("error: fetch/execute failed: %s\n", m6502_strerror(err_code));
        goto ERROR;
    }
    m6502_get_registers(machine, &reg_bank);
    if (reg_bank.a != 0xAF) {
        printf("error: A does not contain the expected value"
               " (has %d instead)\n",
               reg_bank.a
        );
        goto ERROR;
    }

    free(machine);
    free(mem_buf);

    return EXIT_SUCCESS;

ERROR:
    free(machine);
    free(mem_buf);

    return EXIT_FAILURE;
}

