        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        moveq   #1,d0
        moveq   #2,d1
        moveq   #3,d2
        moveq   #-4,d3
        move.l  #0x55555555,d4
        move.l  #0xaaaaaaaa,d5
        move.l  #0x12345678,d6
        move.l  #0x87654321,d7
        lea     DATA,a0
        lea     DATA+0x400,a1
        movea.w #-2,a2
        lea     0x1000.w,a3
        movem.l d0-d7/a0-a6,-(a1)
        movem.l (a1)+,d0-d3/a4-a6
        movem.w (a1)+,d0-d7
        movem.w d0-d7/a0-a3,-(a1)
        movem.w (a1),a0-a3
        movem.l d0/d3/d5/a2,8(a0)
        movem.l 8(a0),d4-d7
        movem.l d0-d7,(DATA+0x200).l
        movem.l (DATA+0x200).l,a0-a6
        lea     DATA,a0
        movem.l a0-a1,-(a0)             | a0 in list with predecrement (020 stores initial-4n? check)
        movem.l (a0)+,a0                | load of base register with postincrement
        movem.w ([DATA]),d0-d1
        | movep
        lea     DATA+0x300,a0
        move.l  #0x11223344,d0
        movep.l d0,0(a0)
        movep.w d0,9(a0)
        movep.l 0(a0),d1
        movep.w 9(a0),d2
        moveq   #3,d0
        stop    #0x2700
