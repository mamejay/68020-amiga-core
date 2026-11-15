| Small benchmark loop (prog mode): typical integer mix with the instruction cache on.
        .equ DATA, 0x10000
        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        moveq   #1,d0
        movec   d0,cacr                 | enable instruction cache
        move.w  #499,d7                 | 500 iterations
        moveq   #0,d2
loop:   lea     DATA,a0
        move.l  (a0)+,d1                | load
        add.l   d1,d2                   | alu
        move.l  d2,(a0)                 | store
        lsl.l   #2,d1                   | shift
        eor.l   d1,d2
        cmp.l   #1000,d2
        bne.s   1f
        addq.l  #1,d3
1:      mulu.w  d7,d1                   | 16x16 multiply
        dbra    d7,loop
        stop    #0x2700
