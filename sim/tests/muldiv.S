        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        move.l  #zdiv,0x14.w            | vector 5: zero divide
        move.l  #0x12345678,d0
        move.l  #0x9abcdef0,d1
        move.l  d0,d2
        mulu.l  d1,d2                   | 32x32->32 overflow
        move.l  d0,d3
        mulu.l  d1,d4:d3                | 64-bit
        move.l  d0,d3
        muls.l  d1,d4:d3
        move.l  #-7,d3
        muls.l  #12345,d3
        move.l  #0x40000,d3
        muls.l  #0x40000,d3             | overflow V
        move.l  #-1,d3
        mulu.l  #-1,d5:d3
        | 32/32
        move.l  #1000000,d2
        divu.l  #7,d2
        move.l  #-1000000,d2
        divs.l  #7,d2
        move.l  #-1000000,d2
        divsl.l #7,d3:d2
        move.l  #1000001,d2
        divul.l #-7,d3:d2
        | 64/32
        move.l  #0x00000001,d3          | high
        move.l  #0x00000000,d2          | low
        divu.l  #3,d3:d2
        move.l  #0xffffffff,d3
        move.l  #0xfffffff0,d2          | -16
        divs.l  #3,d3:d2
        move.l  #0x00000005,d3
        move.l  #0,d2
        divu.l  #2,d3:d2                | overflow
        move.l  #0x80000000,d2
        moveq   #-1,d4
        divs.l  d4,d2                   | overflow (-2^31 / -1)
        divs.w  #-1,d2
        | divide by zero
        moveq   #0,d4
        divu.l  d4,d2
        divs.w  d4,d2
        moveq   #5,d0
        stop    #0x2700
zdiv:   addq.l  #1,d7
        rte
