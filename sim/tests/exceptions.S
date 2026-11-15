        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        move.l  #h_trap,0x80.w          | TRAP #0
        move.l  #h_trap,0x84.w          | TRAP #1
        move.l  #h_illegal,0x10.w
        move.l  #h_priv,0x20.w
        move.l  #h_chk,0x18.w
        move.l  #h_trapv,0x1c.w
        move.l  #h_linea,0x28.w
        move.l  #h_linef,0x2c.w
        move.l  #h_trace,0x24.w
        move.l  #h_fmterr,0x38.w
        moveq   #0,d7
        trap    #0
        trap    #1
        .word   0x4afc                  | ILLEGAL
        .word   0xa123                  | line A
        .word   0xf200,0x0000           | line F (coprocessor id 1)
        | CHK / CHK2 / CMP2 / TRAPV / TRAPcc
        moveq   #-1,d0
        chk.w   #10,d0
        moveq   #20,d0
        chk.l   #10,d0
        lea     DATA,a0
        move.l  #-5,(a0)
        move.l  #100,4(a0)
        moveq   #7,d1
        cmp2.l  (a0),d1
        chk2.l  (a0),d1
        moveq   #-6,d1
        chk2.l  (a0),d1
        move.b  #5,8(a0)
        move.b  #-5,9(a0)               | lower > upper (unsigned wrap) case
        move.b  #0x80,d1
        cmp2.b  8(a0),d1
        movea.w #-3,a1
        cmp2.w  (a0),a1                 | address register: whole register compared
        move.w  #0x0002,ccr
        trapv
        trapvc.w #0x1234
        trapvs.l #0x12345678
        trapt
        | RTD, RTR, LINK/UNLK, BSR.L
        bsr.l   sub1
        pea     ret2(pc)
        move.w  #0x001f,-(sp)
        rtr
ret2:   link.l  a6,#-0x10000
        unlk    a6
        link.w  a5,#-8
        unlk    a5
        pea     ret3(pc)
        rtd     #0
ret3:   | MOVEC / MOVE USP / VBR relocation
        move.l  #0x1234,a0
        move.l  a0,usp
        move    usp,a1
        movec   vbr,d0
        moveq   #3,d0
        movec   d0,sfc
        movec   d0,dfc
        movec   sfc,d1
        movec   cacr,d2
        movec   msp,a2
        movec   isp,a3
        | switch to master stack, then interrupt-stack frames via trap
        lea     0x70000,a4
        movec   a4,msp
        ori.w   #0x1000,sr              | M=1: A7 = MSP
        trap    #0
        andi.w  #0xefff,sr
        | user mode privilege violation, then back via trap
        lea     0x60000,a0
        move.l  a0,usp
        andi.w  #0xdfff,sr              | -> user mode
        move.w  #0x2700,sr              | privileged -> vector 8
        | trace: build an RTE frame with T1 set
        move.w  #0x0000,-(sp)           | format 0, vector 0
        pea     traced(pc)
        move.w  #0xa700,-(sp)           | T1 | S | IPL7
        rte
traced: nop                             | trace exception after this
        nop
        | format error: bogus frame format
        move.w  #0x7000,-(sp)
        pea     fmtok(pc)
        move.w  #0x2700,-(sp)
        rte
fmtok:  | VBR relocation
        lea     DATA+0x1000,a0
        moveq   #63,d1
        lea     0.w,a1
1:      move.l  (a1)+,(a0)+
        move.l  (a1)+,(a0)+
        move.l  (a1)+,(a0)+
        move.l  (a1)+,(a0)+
        dbra    d1,1b
        lea     DATA+0x1000,a0
        movec   a0,vbr
        trap    #1
        moveq   #7,d0
        stop    #0x2700

sub1:   addq.l  #1,d7
        rts
h_trap: addq.l  #1,d7
        rte
h_illegal:
        addq.l  #2,2(sp)                | skip the instruction
        addq.l  #1,d7
        rte
h_linea: addq.l #2,2(sp)
        addq.l  #1,d7
        rte
h_linef: addq.l #4,2(sp)
        addq.l  #1,d7
        rte
h_chk:  addq.l  #1,d7
        rte
h_trapv: addq.l #1,d7
        rte
h_priv: addq.l  #4,2(sp)                | skip move #imm,sr
        ori.w   #0x2000,(sp)            | return to supervisor
        addq.l  #1,d7
        rte
h_trace:
        andi.w  #0x3fff,(sp)            | clear T bits in stacked SR
        addq.l  #1,d7
        rte
h_fmterr:
        addq.l  #8,sp                   | discard the bad frame (format error frame is format 0 = 8 bytes)
        addq.l  #8,sp                   | and the bogus 8-byte frame we built
        addq.l  #1,d7
        jmp     fmtok
