        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        moveq   #-1,d0
        move.l  #0x7fffffff,d1
        add.l   d0,d1                   | V/C
        addq.l  #1,d1
        addx.l  d0,d1
        subx.l  d1,d0
        sub.w   #0x8000,d1
        subq.b  #3,d1
        cmp.l   d0,d1
        cmpi.w  #0x1234,d1
        neg.l   d1
        negx.b  d1
        not.w   d1
        and.l   #0xf0f0f0f0,d1
        or.b    #0x0f,d1
        eori.l  #0x55aa55aa,d1
        eor.w   d1,d0
        ext.w   d0
        ext.l   d0
        extb.l  d1
        swap    d1
        exg     d0,d1
        exg     d0,a0
        exg     a0,a1
        clr.b   d0
        tst.l   d1
        mulu.w  d1,d0
        muls.w  #-5,d0
        move.l  #100000,d2
        divu.w  #7,d2
        move.l  #-100000,d3
        divs.w  #7,d3
        move.l  #0x12345678,d4
        divu.w  #3,d4                   | overflow
        | shifts / rotates, register and immediate counts, incl. count 0 and >= size
        move.l  #0x81234567,d5
        moveq   #0,d6
        asl.l   d6,d5
        moveq   #33,d6
        lsr.l   d6,d5
        move.l  #0x81234567,d5
        asr.l   #4,d5
        asl.w   #7,d5
        lsl.b   #8,d5
        rol.l   #3,d5
        ror.w   #5,d5
        moveq   #17,d6
        roxl.l  d6,d5
        roxr.b  #1,d5
        moveq   #63,d6
        roxl.w  d6,d5
        asr.b   d6,d5
        lea     DATA,a0
        move.w  #0x8001,(a0)
        asl     (a0)
        lsr     (a0)
        roxr    (a0)
        rol     (a0)
        | BCD
        move.w  #0x0004,ccr
        moveq   #0x45,d0
        moveq   #0x38,d1
        abcd    d1,d0
        sbcd    d1,d0
        nbcd    d0
        move.b  #0x99,(a0)
        move.b  #0x01,1(a0)
        lea     2(a0),a1
        lea     1(a0),a2
        abcd    -(a2),-(a1)
        | Scc / DBcc / Bcc
        moveq   #5,d2
1:      addq.l  #1,d3
        dbra    d2,1b
        cmp.l   d3,d4
        sgt     d5
        sls     d5
        seq     (a0)
        tst.b   (a0)
        bne.l   2f
        nop
2:      bra.s   3f
        nop
3:      bra.w   4f
        nop
4:      tas     (a0)
        tas     d5
        moveq   #5,d2
        chk.w   #0x7fff,d2              | in range: no trap
        moveq   #1,d0
        stop    #0x2700
