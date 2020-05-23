# sim6502
MOS 6502 simulator

(Unfinished) simulator for the MOS 6502 processor.

This simulator uses a plugin memory system, in which reads and writes to memory are implemented using custom functions specified by client code (`read_fn` and `write_fn`). This simulates the existence of the address and data buses, and allows for memory-mapped I/O with a simulated external device.

Currently, the following instructions (and all of their associated addressing modes) are implemented:

LDA, LDX, LDY\
STA, STX, STY\
TAX, TAY, TXA, TYA, TSX, TXS\
PHA, PHP, PLA, PLP\
AND, EOR, ORA, BIT

BCD math is not implemented and functions the same as binary math (i.e. the flag is ignored by arithmetic instructions).

The final version of the simulator will include support for interrupts, compatibility flags, power-on/reset behavior, and an implementation of every instruction.
