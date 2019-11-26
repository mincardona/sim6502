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

#ifndef HEADER_H
#define HEADER_H

#include <stdint.h>

/** Address from which the PC is loaded during an NMI */
#define M6502_NMI_PC_ADDR 0xFFFA
/** Address from which the PC is loaded during a reset */
#define M6502_RES_PC_ADDR 0xFFFC
/** Address from which the PC is loaded during an IRQ */
#define M6502_IRQ_PC_ADDR 0xFFFE

/** Size, in bytes, of the address space */
#define M6502_ADDR_SPACE_SIZE 65536

/* Flag constants */
#define M6502_FLAG_MASK_C   1u
#define M6502_FLAG_MASK_Z   2u
#define M6502_FLAG_MASK_I   4u
#define M6502_FLAG_MASK_D   8u
#define M6502_FLAG_MASK_B   16u
/* flag 5 is always set, I think */
#define M6502_FLAG_MASK_5   32u
#define M6502_FLAG_MASK_V   64u
#define M6502_FLAG_MASK_N   128u

#define E6502_OK 0
#define E6502_UNKNOWN 1
#define E6502_NOMEM 2
#define E6502_MMAP 3
#define E6502_INVAL 4
#define E6502_BADSTATE 5

/** Gets a human-readable string explaining the given E6502 error code */
const char* m6502_strerror(int code);

typedef int (*m6502_read_fn)(void*, uint16_t, uint8_t*);
typedef int (*m6502_write_fn)(void*, uint16_t, uint8_t);

struct m6502_mem_delegate {
    void* pctx; /* context pointer */
    /* (*uint8_t) must NOT be modified if the operation fails */
    /* TODO: re-evaluate this? ^ */
    m6502_read_fn read_fn;
    m6502_write_fn write_fn;
};

enum m6502_config_flags {
    M6502_CFG_NONE = 0,
    M6502_CFG_BUGGY_JMP_INDIRECT = 1    /* use buggy JMP indirect behavior */
};

struct m6502_machine_config {
    int32_t flags;
    struct m6502_mem_delegate mem_delegate;
};

struct m6502_machine;

struct m6502_reg_bank {
    uint16_t pc;
    uint8_t sp;
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t p;
};

struct m6502_machine* m6502_make_machine(const struct m6502_machine_config* config);
void m6502_destroy_machine(struct m6502_machine* machine);
int m6502_fetch_execute(struct m6502_machine* machine);
int m6502_get_registers(const struct m6502_machine* machine, struct m6502_reg_bank* bank);
int m6502_set_registers(struct m6502_machine* machine, const struct m6502_reg_bank* bank);
int m6502_get_error(const struct m6502_machine* machine);
int m6502_begin_irq(struct m6502_machine* machine);
int m6502_begin_nmi(struct m6502_machine* machine);

#endif

