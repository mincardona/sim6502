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

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim6502.h"


/*********************************************************************
 * Some debug macros
 *********************************************************************/
#ifdef NDEBUG
#define DEBUG_PRINT_1(S, A) ((void)0)
#define DEBUG_PRINT_0(S) ((void)0)
#else
#define DEBUG_PRINT_1(S, A) printf(S, A)
#define DEBUG_PRINT_0(S) printf("%s", S)
#endif


/*********************************************************************
 * Bit and numerical manipulation functions
 *********************************************************************/

/* extracts Q bits starting at position P from N */
#define GETBITS(N, P, Q) (((N) >> (P)) & ((1 << ((Q) + 1)) - 1))
#define GETBIT(N, P) (GETBITS((N), (P), 1))
#define BYTE0(N) ((N) & 0xFF)
#define BYTE1(N) (((N) & 0xFF00) >> 8)

/* Operations with the flag from N having mask M (M6502_FLAG_MASK_???) */
#define GETFLAG(N, M) ((N) & (M) ? 1 : 0)
#define FLAG_SET(N, M) ((N) | (M))
#define FLAG_CLEAR(N, M) ((N) & ~(M))

/**
 * Sets (1) or clears (0) the flag identified by flag_mask (M6502_FLAG_MASK_???)
 * based on whether do_set is truthy.
 */
static uint8_t flag_copy_8(uint8_t reg, uint8_t flag_mask, int do_set) {
    return do_set ? FLAG_SET(reg, flag_mask) : FLAG_CLEAR(reg, flag_mask);
}

/**
 * Computes the page on which the given memory address resides.
 */
/* TODO: use this function somewhere or remove it */
#if 0
static uint8_t m6502_page_of(uint16_t addr) {
    return (uint8_t)BYTE1(addr);
}
#endif

/* converts a little endian 2-byte sequence to a number */
static uint16_t m6502_contain_16(const uint8_t* bytes) {
    return ((uint16_t)bytes[1] << 8) | bytes[0];
}



/*********************************************************************
 * Other constants
 *********************************************************************/

#define M6502_STACK_OFS 0x100



/*********************************************************************
 * Error definitions
 *********************************************************************/

static const char* m6502_strerrors[] = {
    "ok",
    "unclassified internal error",
    "unable to allocate memory",
    "a memory-mapping plugin function signaled an error",
    "invalid argument provided",
    "machine is in an error state"
};

const char* m6502_strerror(int code) {
    int realcode;
    if (code < 0 || code > E6502_BADSTATE) {
        realcode = E6502_INVAL;
    } else {
        realcode = code;
    }
    return m6502_strerrors[realcode];
}



/*********************************************************************
 * Instruction decoding
 *********************************************************************/

/*
An opcode group is a collection of opcodes that differ only by addressing mode.
For example, all varieties of the STA instruction belong to one opcode group.
*/
enum OPCODE_GROUP {
    /* load/store */
    OPG_LDA = 0, OPG_LDX, OPG_LDY, OPG_STA, OPG_STX, OPG_STY,

    /* register transfer */
    OPG_TAX, OPG_TAY, OPG_TXA, OPG_TYA,

    /* stack */
    OPG_TSX, OPG_TXS, OPG_PHA, OPG_PHP, OPG_PLA, OPG_PLP,

    /* bitwise logic */
    OPG_AND, OPG_EOR, OPG_ORA, OPG_BIT,

    /* arithmetic */
    OPG_ADC, OPG_SBC, OPG_CMP, OPG_CPX, OPG_CPY,

    /* increment and decrement */
    OPG_INC, OPG_INX, OPG_INY, OPG_DEC, OPG_DEX, OPG_DEY,

    /* bit shift */
    OPG_ASL, OPG_LSR, OPG_ROL, OPG_ROR,

    /* jump and call */
    OPG_JMP, OPG_JSR, OPG_RTS,

    /* branch */
    OPG_BCC, OPG_BCS, OPG_BEQ, OPG_BMI, OPG_BNE, OPG_BPL, OPG_BVC, OPG_BVS,

    /* status flags */
    OPG_CLC, OPG_CLD, OPG_CLI, OPG_CLV, OPG_SEC, OPG_SED, OPG_SEI,

    /* system/special */
    OPG_BRK, OPG_NOP, OPG_RTI,

    OPG_COUNT   /* used to iterate from 0 */
};

/*
an id for each addressing mode
*/
enum ADDRESSING_MODE {
    ADM_IMPLICIT = 0,
    ADM_ACCUMULATOR,
    ADM_IMMEDIATE,
    ADM_ZERO_PAGE,
    ADM_ZERO_PAGE_X,
    ADM_ZERO_PAGE_Y,
    ADM_RELATIVE,
    ADM_ABSOLUTE,
    ADM_ABSOLUTE_X,
    ADM_ABSOLUTE_Y,
    ADM_INDIRECT,
    ADM_INDIRECT_X,
    ADM_INDIRECT_Y,

    ADM_COUNT /* used to iterate over all addressing modes */
};

/* All the information necessary to execute the instr */
struct m6502_decoded_instr {
    enum OPCODE_GROUP group;     /* opcode group */
    enum ADDRESSING_MODE addr_mode; /* addressing mode */
    uint8_t args[2]; /* addressing mode arguments, if any */
};

/* The number of extra bytes to fetch for a certain addressing mode */
static const int addressing_mode_arg_size[ADM_COUNT] = { 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1 };

/* opcodes[opcode_group][addressing_mode] */
static const int opcode_table[OPG_COUNT][ADM_COUNT] = {
    /* opcode    IMP,  ACC,  IMM,   ZP,  ZPX,  ZPY,  REL,  ABS, ABSX, ABSY,  IND, INDX, INDY */
    /* LDA */ {   -1,   -1, 0xa9, 0xa5, 0xb5,   -1,   -1, 0xad, 0xbd, 0xb9,   -1, 0xa1, 0xb1 },
    /* LDX */ {   -1,   -1, 0xa2, 0xa6,   -1, 0xb6,   -1, 0xae,   -1, 0xbe,   -1,   -1,   -1 },
    /* LDY */ {   -1,   -1, 0xa0, 0xa4, 0xb4,   -1,   -1, 0xac, 0xbc,   -1,   -1,   -1,   -1 },
    /* STA */ {   -1,   -1,   -1, 0x85, 0x95,   -1,   -1, 0x8d, 0x9d, 0x99,   -1, 0x81, 0x91 },
    /* STX */ {   -1,   -1,   -1, 0x86,   -1, 0x96,   -1, 0x8e,   -1,   -1,   -1,   -1,   -1 },
    /* STY */ {   -1,   -1,   -1, 0x84, 0x94,   -1,   -1, 0x8c,   -1,   -1,   -1,   -1,   -1 },

    /* TAX */ { 0xaa,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* TAY */ { 0xa8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* TXA */ { 0x8a,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* TYA */ { 0x98,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },

    /* TSX */ { 0xba,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* TXS */ { 0x9a,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* PHA */ { 0x48,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* PHP */ { 0x08,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* PLA */ { 0x68,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* PLP */ { 0x28,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },

    /* AND */ {   -1,   -1, 0x29, 0x25, 0x35,   -1,   -1, 0x2d, 0x3d, 0x39,   -1, 0x21, 0x31 },
    /* EOR */ {   -1,   -1, 0x49, 0x45, 0x55,   -1,   -1, 0x4d, 0x5d, 0x59,   -1, 0x41, 0x51 },
    /* ORA */ {   -1,   -1, 0x09, 0x05, 0x15,   -1,   -1, 0x0d, 0x1d, 0x19,   -1, 0x01, 0x11 },
    /* BIT */ {   -1,   -1,   -1, 0x24,   -1,   -1,   -1, 0x2c,   -1,   -1,   -1,   -1,   -1 },

    /* ADC */ {   -1,   -1, 0x69, 0x65, 0x75,   -1,   -1, 0x6d, 0x7d, 0x79,   -1, 0x61, 0x71 },
    /* SBC */ {   -1,   -1, 0xe9, 0xe5, 0xf5,   -1,   -1, 0xed, 0xfd, 0xf9,   -1, 0xe1, 0xf1 },
    /* CMP */ {   -1,   -1, 0xc9, 0xc5, 0xd5,   -1,   -1, 0xcd, 0xdd, 0xd9,   -1, 0xc1, 0xd1 },
    /* CPX */ {   -1,   -1, 0xe0, 0xe4,   -1,   -1,   -1, 0xec,   -1,   -1,   -1,   -1,   -1 },
    /* CPY */ {   -1,   -1, 0xc0, 0xc4,   -1,   -1,   -1, 0xcc,   -1,   -1,   -1,   -1,   -1 },

    /* INC */ {   -1,   -1,   -1, 0xe6, 0xf6,   -1,   -1, 0xee, 0xfe,   -1,   -1,   -1,   -1 },
    /* INX */ { 0xe8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* INY */ { 0xc8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* DEC */ {   -1,   -1,   -1, 0xc6, 0xd6,   -1,   -1, 0xce, 0xde,   -1,   -1,   -1,   -1 },
    /* DEX */ { 0xca,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* DEY */ { 0x88,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },

    /* ASL */ {   -1, 0x0a,   -1, 0x06, 0x16,   -1,   -1, 0x0e, 0x1e,   -1,   -1,   -1,   -1 },
    /* LSR */ {   -1, 0x4a,   -1, 0x46, 0x56,   -1,   -1, 0x4e, 0x5e,   -1,   -1,   -1,   -1 },
    /* ROL */ {   -1, 0x2a,   -1, 0x26, 0x36,   -1,   -1, 0x2e, 0x3e,   -1,   -1,   -1,   -1 },
    /* ROR */ {   -1, 0x6a,   -1, 0x66, 0x76,   -1,   -1, 0x6e, 0x7e,   -1,   -1,   -1,   -1 },

    /* JMP */ {   -1,   -1,   -1,   -1,   -1,   -1,   -1, 0x4c,   -1,   -1, 0x6c,   -1,   -1 },
    /* JSR */ {   -1,   -1,   -1,   -1,   -1,   -1,   -1, 0x20,   -1,   -1,   -1,   -1,   -1 },
    /* RTS */ {  0x60,  -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },

    /* BCC */ {   -1,   -1,   -1,   -1,   -1,   -1, 0x90,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BCS */ {   -1,   -1,   -1,   -1,   -1,   -1, 0xb0,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BEQ */ {   -1,   -1,   -1,   -1,   -1,   -1, 0xf0,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BMI */ {   -1,   -1,   -1,   -1,   -1,   -1, 0x30,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BNE */ {   -1,   -1,   -1,   -1,   -1,   -1, 0xd0,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BPL */ {   -1,   -1,   -1,   -1,   -1,   -1, 0x10,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BVC */ {   -1,   -1,   -1,   -1,   -1,   -1, 0x50,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* BVS */ {   -1,   -1,   -1,   -1,   -1,   -1, 0x70,   -1,   -1,   -1,   -1,   -1,   -1 },

    /* CLC */ { 0x18,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* CLD */ { 0xd8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* CLI */ { 0x58,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* CLV */ { 0xb8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* SEC */ { 0x38,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* SED */ { 0xf8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* SEI */ { 0x78,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },

    /* BRK */ { 0x00,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* NOP */ { 0xea,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
    /* RTI */ { 0x40,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1 },
};

/* arg bytes is the number of extra bytes to fetch for the argument
   returns nonzero on error */
static int m6502_decode_opcode(
        uint8_t opc,
        enum OPCODE_GROUP* p_group,
        enum ADDRESSING_MODE* p_mode)
{
    int opg;
    int adm;
    for (opg = 0; opg < OPG_COUNT; opg++) {
        for (adm = 0; adm < ADM_COUNT; adm++) {
            if (opcode_table[opg][adm] == opc) {
                if (p_group) {
                    *p_group = opg;
                }
                if (p_mode) {
                    *p_mode = adm;
                }

                return E6502_OK;
            }
        }
    }

    return E6502_INVAL;
}



/*********************************************************************
 * Machine data structure definition
 *********************************************************************/

struct m6502_machine {
    struct m6502_reg_bank regs;
    struct m6502_machine_config config;
    int error;  /* stored error code */
};



/*********************************************************************
 * Load/store and push/pull functions
 *********************************************************************/

/* WARNING: does not null check */
/* machine's error flag is unaffected. */
static int m6502_store_byte(struct m6502_machine* machine, uint16_t addr, uint8_t b) {
    struct m6502_mem_delegate* md = &machine->config.mem_delegate;
    return md->write_fn(md->pctx, addr, b);
}

/* WARNING: does not null check */
/* stores in little-endian order */
/* error flag is unaffected */
/* TODO: use this function somewhere or remove it */
#if 0
static int m6502_store_16(struct m6502_machine* machine, uint16_t addr, uint16_t w) {
    int ret = m6502_store_byte(machine, addr, BYTE0(w));
    if (ret) {
        return ret;
    }
    return m6502_store_byte(machine, addr + 1, BYTE1(w));
}
#endif

/* WARNING: does not null check */
/* pb is not modified if the operation fails, assuming read_fn is implemented correctly */
/* error flag in unaffected */
static int m6502_load_byte(struct m6502_machine* machine, uint16_t addr, uint8_t* pb) {
    struct m6502_mem_delegate* md = &machine->config.mem_delegate;
    return md->read_fn(md->pctx, addr, pb);
}

/* WARNING: does not null check */
/* pb is not modified if the operation fails, assuming read_fn is correct */
/* loads in little-endian order */
/* error flag is unaffected */
static int m6502_load_16(struct m6502_machine* machine, uint16_t addr, uint16_t* pw) {
    uint8_t bytes[2];
    int ret = m6502_load_byte(machine, addr, &bytes[0]);
    if (ret) {
        return ret;
    }
    ret = m6502_load_byte(machine, addr + 1, &bytes[1]);
    if (ret) {
        return ret;
    }
    *pw = m6502_contain_16(bytes);
    return E6502_OK;
}

/* WARNING: does not null check */
/* error flag is unaffected */
static int m6502_push_byte(struct m6502_machine* machine, uint8_t b) {
    /* (sp + $100) <-- n */
    int ret = m6502_store_byte(machine, machine->regs.sp + M6502_STACK_OFS, b);
    if (!ret) {
        /* sp <-- sp - 1 */
        machine->regs.sp--;
    }
    return ret;
}

/* WARNING: does not null check */
/* pushes high byte first */
/* error flag is unaffected */
static int m6502_push_16(struct m6502_machine* machine, uint16_t w) {
    int ret = m6502_push_byte(machine, BYTE1(w));
    if (ret) {
        return ret;
    }
    return m6502_push_byte(machine, BYTE0(w));
}

/* WARNING: does not null check */
/* error flag is unaffected */
static int m6502_pull_byte(struct m6502_machine* machine, uint8_t* b) {
    int ret = m6502_load_byte(machine, machine->regs.sp + M6502_STACK_OFS, b);
    if (!ret) {
        machine->regs.sp++;
    }
    return ret;
}

/* WARNING: does not null check */
/* pulls low byte first */
/* error flag is unaffected */
/* TODO: use this function somewhere or remove it */
#if 0
static int m6502_pull_16(struct m6502_machine* machine, uint16_t* pw) {
    uint8_t bytes[2];
    int ret = m6502_pull_byte(machine, &bytes[0]);
    if (ret) {
        return ret;
    }
    ret = m6502_pull_byte(machine, &bytes[1]);
    if (ret) {
        return ret;
    }
    *pw = m6502_contain_16(bytes);
    return E6502_OK;
}
#endif



/*********************************************************************
 * Constructor and destructor
 *********************************************************************/

/* caller must check (machine != NULL) and (machine->error) after returning */
struct m6502_machine* m6502_make_machine(const struct m6502_machine_config* config) {
    struct m6502_machine* machine;

    machine = malloc(sizeof(*machine));
    if (!machine) {
        return NULL;
    }

    if (!config->mem_delegate.read_fn || !config->mem_delegate.write_fn) {
        machine->error = E6502_INVAL;
        return machine;
    }

    machine->error = E6502_OK;

    machine->config = *config;

    machine->regs.pc = 0;
    machine->regs.sp = 0xFF;
    machine->regs.a = 0;
    machine->regs.x = 0;
    machine->regs.y = 0;
    machine->regs.p = 0;

    if (m6502_load_16(machine, M6502_RES_PC_ADDR, &machine->regs.pc)) {
        machine->error = E6502_MMAP;
        return machine;
    }

    /* flag 5 is hardwired to always be set */
    /* I flag is set on startup AFAIK */
    machine->regs.p |= M6502_FLAG_MASK_5 | M6502_FLAG_MASK_I;

    return machine;
}

void m6502_destroy_machine(struct m6502_machine* machine) {
    free(machine);
}



/*********************************************************************
 * Instruction implementation
 *********************************************************************/

/* returns cycle count taken or negative on failure */
typedef int (*instr_function_t)(
    struct m6502_machine* machine,
    const struct m6502_decoded_instr* instr
);

/*
 * modifies the Z and N bits in old_flags based on test_value.
 * Returns the result
 */
static uint8_t m6502_adjust_zn(uint8_t old_flags, uint8_t test_value) {
    uint8_t new_flags = old_flags;

    if (test_value == 0) {
        new_flags = FLAG_SET(new_flags, M6502_FLAG_MASK_Z);
    } else {
        new_flags = FLAG_CLEAR(new_flags, M6502_FLAG_MASK_Z);
    }

    if (GETBIT(test_value, 7)) {
        new_flags = FLAG_SET(new_flags, M6502_FLAG_MASK_N);
    } else {
        new_flags = FLAG_CLEAR(new_flags, M6502_FLAG_MASK_N);
    }

    return new_flags;
}

/*
 * Gets the arg address for these modes:
 * ZP, ZPX, ZPY, ABS, ABSX, ABSY, IND, INDX, INDY
 * does not work for:
 * IMP, ACC, IMM, REL
 */
static int m6502_compute_arg_address(
        struct m6502_machine* machine,
        const struct m6502_decoded_instr* instr,
        uint16_t* out_addr)
{
    uint16_t addr_temp = 0;
    int error = E6502_OK;

    switch (instr->addr_mode) {
    case ADM_ZERO_PAGE:
        addr_temp = instr->args[0];
        break;
    case ADM_ZERO_PAGE_X:
        addr_temp = (instr->args[0] + machine->regs.x) % 0x100;
        break;
    case ADM_ZERO_PAGE_Y:
        addr_temp = (instr->args[0] + machine->regs.y) % 0x100;
        break;
    case ADM_ABSOLUTE:
        addr_temp = m6502_contain_16(instr->args);
        break;
    case ADM_ABSOLUTE_X:
        addr_temp = m6502_contain_16(instr->args) + machine->regs.x;
        break;
    case ADM_ABSOLUTE_Y:
        addr_temp = m6502_contain_16(instr->args) + machine->regs.y;
        break;
    case ADM_INDIRECT:
    {
        uint16_t contained_args = m6502_contain_16(instr->args);

        if (BYTE0(contained_args) == 0xFF
                && (machine->config.flags & M6502_CFG_BUGGY_JMP_INDIRECT))
        {
            uint8_t indirect_buffer[2] = {0, 0};
            error = m6502_load_byte(machine, contained_args, &indirect_buffer[0]);
            if (error) {
                break;
            }
            error = m6502_load_byte(
                    machine,
                    contained_args & 0xFF00,
                    &indirect_buffer[1]);
            if (error) {
                break;
            }
            addr_temp = m6502_contain_16(indirect_buffer);
        } else {
            error = m6502_load_16(machine, contained_args, &addr_temp);
        }
        break;
    }
    case ADM_INDIRECT_X:
        error = m6502_load_16(
                machine,
                (instr->args[0] + machine->regs.x) % 0x100,
                &addr_temp);
        break;
    case ADM_INDIRECT_Y:
        error = m6502_load_16(machine, instr->args[0], &addr_temp);
        addr_temp += machine->regs.y;
        break;
    default:
        assert("Unknown addr mode passed to m6502_compute_arg_address" == 0);
        error = E6502_UNKNOWN;
    }

    if (!error) {
        *out_addr = addr_temp;
    }
    return error;
}

/* macro for implementing the load instructions succinctly */
#define INSTR_IMPL_LD_TEMPLATE(NAME, REG) static int NAME( \
    struct m6502_machine* machine, \
    const struct m6502_decoded_instr* instr) \
{ \
    uint8_t newval = 0; \
    int error = E6502_OK; \
    if (instr->addr_mode == ADM_IMMEDIATE) { \
        newval = instr->args[0]; \
    } else { \
        uint16_t newval_addr; \
        error = m6502_compute_arg_address(machine, instr, &newval_addr); \
        if (error) { \
            return error; \
        } \
        error = m6502_load_byte(machine, newval_addr, &newval); \
        if (error) { \
            return error; \
        } \
    } \
    machine->regs.REG = newval; \
    machine->regs.p = m6502_adjust_zn(machine->regs.p, machine->regs.REG); \
    machine->regs.pc++; \
    return E6502_OK; \
}

INSTR_IMPL_LD_TEMPLATE(instr_impl_lda, a)
INSTR_IMPL_LD_TEMPLATE(instr_impl_ldx, x)
INSTR_IMPL_LD_TEMPLATE(instr_impl_ldy, y)

#undef INSTR_IMPL_LD_TEMPLATE


/* macro for implementing the store instructions succinctly */
#define INSTR_IMPL_ST_TEMPLATE(NAME, REG) static int NAME(\
    struct m6502_machine* machine, \
    const struct m6502_decoded_instr* instr ) \
{ \
    uint16_t arg_addr; \
    int error = m6502_compute_arg_address(machine, instr, &arg_addr); \
    if (error) { \
        return error; \
    } \
    error = m6502_store_byte(machine, arg_addr, machine->regs.REG); \
    if (error) { \
        return error; \
    } \
    machine->regs.pc++; \
    return E6502_OK; \
}

INSTR_IMPL_ST_TEMPLATE(instr_impl_sta, a)
INSTR_IMPL_ST_TEMPLATE(instr_impl_stx, x)
INSTR_IMPL_ST_TEMPLATE(instr_impl_sty, y)

#undef INSTR_IMPL_ST_TEMPLATE


/* macro for implementing the transfer instructions succinctly */
#define INSTR_IMPL_TXX_TEMPLATE(NAME, SRC, DEST) static int NAME( \
struct m6502_machine* machine, \
const struct m6502_decoded_instr* instr) \
{ \
    (void)instr; \
    machine->regs.DEST = machine->regs.SRC; \
    machine->regs.p = m6502_adjust_zn(machine->regs.p, machine->regs.DEST); \
    machine->regs.pc++; \
    return E6502_OK; \
}

INSTR_IMPL_TXX_TEMPLATE(instr_impl_tax, a, x)
INSTR_IMPL_TXX_TEMPLATE(instr_impl_tay, a, y)
INSTR_IMPL_TXX_TEMPLATE(instr_impl_txa, x, a)
INSTR_IMPL_TXX_TEMPLATE(instr_impl_tya, y, a)
INSTR_IMPL_TXX_TEMPLATE(instr_impl_tsx, sp, x)

/* TXS is implemented separately because it does not modify status flags */
static int instr_impl_txs(
    struct m6502_machine* machine,
    const struct m6502_decoded_instr* instr)
{
    (void)instr;
    machine->regs.sp = machine->regs.x;
    machine->regs.pc++;
    return E6502_OK;
}

#undef INSTR_IMPL_TXX_TEMPLATE


static int instr_impl_pha(
    struct m6502_machine* machine,
    const struct m6502_decoded_instr* instr)
{
    int ret = m6502_push_byte(machine, machine->regs.a);
    machine->regs.pc++;
    (void)instr;
    return ret;
}

static int instr_impl_php(
    struct m6502_machine* machine,
    const struct m6502_decoded_instr* instr)
{
    int ret = m6502_push_byte(machine, machine->regs.p);
    machine->regs.pc++;
    (void)instr;
    return ret;
}

static int instr_impl_pla(
    struct m6502_machine* machine,
    const struct m6502_decoded_instr* instr)
{
    int ret = m6502_pull_byte(machine, &machine->regs.a);
    machine->regs.p = m6502_adjust_zn(machine->regs.p, machine->regs.a);
    machine->regs.pc++;
    (void)instr;
    return ret;
}

static int instr_impl_plp(
    struct m6502_machine* machine,
    const struct m6502_decoded_instr* instr)
{
    int ret = m6502_pull_byte(machine, &machine->regs.p);
    machine->regs.pc++;
    (void)instr;
    return ret;
}


/* macro for implementing binary boolean (AND, EOR, ORA) instructions succinctly */
#define INSTR_IMPL_BINBOOL_TEMPLATE(NAME, OPERATOR) static int NAME( \
    struct m6502_machine* machine, \
    const struct m6502_decoded_instr* instr) \
{ \
    uint8_t arg = 0; \
    int error = E6502_OK; \
\
    if (instr->addr_mode == ADM_IMMEDIATE) { \
        arg = instr->args[1]; \
    } else { \
        uint16_t arg_addr; \
\
        error = m6502_compute_arg_address(machine, instr, &arg_addr); \
        if (error) { \
            return error; \
        } \
\
        error = m6502_load_byte(machine, arg_addr, &arg); \
    } \
\
    if (!error) { \
        machine->regs.a OPERATOR arg; \
        machine->regs.p = m6502_adjust_zn(machine->regs.p, machine->regs.a); \
    } \
\
    return error; \
}

INSTR_IMPL_BINBOOL_TEMPLATE(instr_impl_and, &=)
INSTR_IMPL_BINBOOL_TEMPLATE(instr_impl_eor, ^=)
INSTR_IMPL_BINBOOL_TEMPLATE(instr_impl_ora, |=)

static instr_function_t instr_jump_table[OPG_COUNT] = {
    instr_impl_lda, instr_impl_ldx, instr_impl_ldy,
    instr_impl_sta, instr_impl_stx, instr_impl_sty,

    instr_impl_tax, instr_impl_tay, instr_impl_txa, instr_impl_tya,

    instr_impl_tsx, instr_impl_txs,
    instr_impl_pha, instr_impl_php,
    instr_impl_pla, instr_impl_plp,

    instr_impl_and, instr_impl_eor, instr_impl_ora, NULL,
    NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL
};

/*********************************************************************
 * Fetch/execute
 *********************************************************************/

int m6502_fetch_execute(struct m6502_machine* machine) {
    int error;
    uint8_t tmp_byte;
    int arg_bytes;
    int i;
    struct m6502_decoded_instr dec_instr;
    instr_function_t ifn;
    uint16_t oldpc;

    if (!machine) {
        return E6502_INVAL;
    } else if (machine->error) {
        return E6502_BADSTATE;
    }

    oldpc = machine->regs.pc;

    error = m6502_load_byte(machine, machine->regs.pc, &tmp_byte);
    if (error) {
        goto ERROR;
    }

    error = m6502_decode_opcode(tmp_byte, &dec_instr.group, &dec_instr.addr_mode);
    if (error) {
        goto ERROR;
    }

    arg_bytes = addressing_mode_arg_size[dec_instr.addr_mode];

    dec_instr.args[1] = dec_instr.args[0] = 0;

    for (i = 0; i < arg_bytes; i++) {
        /* It is important to increment PC here instead of in the for loop
         * header itself, so that it points *at* (instead of *past*) the last
         * byte fetched when the loop terminates.
         */
        machine->regs.pc++;
        error = m6502_load_byte(machine, machine->regs.pc, &dec_instr.args[i]);
        if (error) {
            goto ERROR;
        }
    }

    DEBUG_PRINT_1("decoded instruction at $%.4x\n", oldpc);
    DEBUG_PRINT_1("group = %d\n", dec_instr.group);
    DEBUG_PRINT_1("addr_mode = %d\n", dec_instr.addr_mode);
    DEBUG_PRINT_1("arg bytes = [$%.2x, ", dec_instr.args[0]);
    DEBUG_PRINT_1("$%.2x]\n", dec_instr.args[1]);
    DEBUG_PRINT_0("\n");

    ifn = instr_jump_table[dec_instr.group];
    if (!ifn) {
        /* TODO: remove this when all instructions are implemented */
        assert("No instr impl function defined" == 0);
        error = E6502_UNKNOWN;
        goto ERROR;
    }

    error = ifn(machine, &dec_instr);
    if (error) {
        goto ERROR;
    }

    /*
     * NOTE:
     * the instruction function is responsible for incrementing or changing pc
     */

    return E6502_OK;

ERROR:
    machine->error = error;
    return error;
}



/*********************************************************************
 * Getters and setters
 *********************************************************************/

int m6502_get_registers(const struct m6502_machine* machine, struct m6502_reg_bank* bank) {
    if (!machine || !bank) {
        return E6502_INVAL;
    }
    *bank = machine->regs;
    return E6502_OK;
}

int m6502_set_registers(struct m6502_machine* machine, const struct m6502_reg_bank* bank) {
    if (!machine || !bank) {
        return E6502_INVAL;
    }
    machine->regs = *bank;
    return E6502_OK;
}

int m6502_get_error(const struct m6502_machine* machine) {
    if (!machine) {
        return E6502_INVAL;
    }
    return machine->error;
}



/*********************************************************************
 * Interrupt functions
 *********************************************************************/

/* Does NULL check */
/* May set the error flag */
static int m6502_begin_interrupt_generic(struct m6502_machine* machine, int is_maskable) {
    int error = E6502_OK;

    if (!machine) {
        return E6502_INVAL;
    } else if (machine->error) {
        return E6502_BADSTATE;
    }

    if (is_maskable && (machine->regs.p & M6502_FLAG_MASK_I)) {
        return E6502_OK;
    }

    error = m6502_push_16(machine, machine->regs.pc);
    if (error) {
        goto ERROR;
    }
    error = m6502_push_byte(machine, machine->regs.p);
    if (error) {
        goto ERROR;
    }

    /* The docs aren't clear but I think this also happens for NMIs */
    machine->regs.p &= M6502_FLAG_MASK_I;

    return m6502_load_16(machine, (is_maskable ? M6502_IRQ_PC_ADDR : M6502_NMI_PC_ADDR), &machine->regs.pc);

ERROR:
    machine->error = error;
    return error;
}

/* note: this function does nothing if the interrupt disable flag is set */
int m6502_begin_irq(struct m6502_machine* machine) {
    return m6502_begin_interrupt_generic(machine, 0);
}

int m6502_begin_nmi(struct m6502_machine* machine) {
    return m6502_begin_interrupt_generic(machine, 1);
}
