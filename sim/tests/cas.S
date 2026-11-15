        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        lea     DATA,a0
        move.l  #0x11112222,(a0)
        move.l  #0x33334444,4(a0)
        move.l  #0x11112222,d0
        move.l  #0xaaaabbbb,d1
        cas.l   d0,d1,(a0)              | succeeds
        cas.l   d0,d1,(a0)              | fails, d0 <- mem
        cas.w   d0,d1,2(a0)
        move.b  #0x44,d2
        cas.b   d2,d1,7(a0)
        cas.b   d2,d1,7(a0)
        cas.w   d0,d1,5(a0)             | misaligned
        lea     4(a0),a1
        move.l  (a0),d2
        move.l  (a1),d3
        moveq   #1,d4
        moveq   #2,d5
        cas2.l  d2:d3,d4:d5,(a0):(a1)   | succeeds
        cas2.l  d2:d3,d4:d5,(a0):(a1)   | fails
        move.w  (a0),d2
        move.w  #0x9999,d3
        cas2.w  d2:d3,d4:d5,(a0):(a1)   | first matches, second not
        tas     3(a0)
        moveq   #6,d0
        stop    #0x2700
