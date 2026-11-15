        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        lea     DATA,a0
        moveq   #0,d0
        | fill a table with known values
        moveq   #63,d1
1:      move.l  d0,(a0)+
        add.l   #0x01010101,d0
        dbra    d1,1b
        lea     DATA,a0
        lea     DATA+0x80,a1
        moveq   #3,d1
        move.l  #-2,d2
        move.l  (a0),d3
        move.l  (a0)+,d3
        move.w  (a0)+,d3
        move.b  (a0)+,d3
        move.b  -(a0),d3
        move.l  -(a1),d3
        move.l  8(a0),d3
        move.w  -6(a1),d3
        move.l  4(a0,d1.w),d3
        move.l  4(a0,d1.l*4),d3
        move.l  -4(a1,d2.w*2),d3
        move.l  (a0,a1.l),d3            | brief, An index
        | full format, no memory indirect
        move.l  (0x20,a0,d1.l*8),d3
        move.l  (0x12345,a0),d3
        move.l  (0x10,za0,d1.l*4),d3    | base suppressed
        move.l  (DATA+0x20).l,d3
        | memory indirect: put pointers in memory
        lea     DATA+0x100,a2
        move.l  #DATA+0x40,(a2)
        move.l  #DATA+0x60,4(a2)
        move.l  ([a2]),d3
        move.l  ([4,a2]),d3
        move.l  ([a2],d1.l*4),d3        | postindexed
        move.l  ([a2],d1.w*2,8),d3      | postindexed + od
        move.l  ([0,a2,d1.l],4),d3      | preindexed (d1=3 -> misaligned pointer read)
        moveq   #1,d1
        move.l  ([0,a2,d1.l*4],-4),d3   | preindexed
        move.l  ([0x100.w,a0],0x10.l),d3
        move.l  ([DATA+0x100]),d3       | base+index suppressed
        move.l  ([a2,zd1],2),d3
        | PC relative
        move.l  pcdat(pc),d3
        move.w  pcdat(pc,d1.w*2),d3
        move.l  ([pcptr,pc]),d3
        move.l  ([pcptr,pc],d1.l*4,4),d3
        lea     pcdat(pc),a3
        pea     ([a2],8)
        move.l  (sp)+,d4
        | abs short/long, immediate
        move.w  0x7ffe.w,d3
        move.l  DATA+0x10,d3
        move.b  #0x5a,d3
        move.w  #0xa55a,d3
        | misaligned data access (68020 allows)
        move.l  #0xdeadbeef,DATA+0x201
        move.l  DATA+0x203,d3
        move.w  DATA+0x205,d3
        | writes through complex modes
        move.l  d3,([a2],d1.l*4,8)
        move.w  d3,(0x22,a0,d1.w*2)
        clr.l   ([4,a2])
        addq.l  #1,([a2])
        moveq   #2,d0
        stop    #0x2700
pcdat:  .long   0x11223344, 0x55667788
pcptr:  .long   pcdat
