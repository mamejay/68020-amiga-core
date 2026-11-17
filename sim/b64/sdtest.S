| SD-card register path test for the full-system sim (tb_sd.sv). 68000 code, so the same
| binary runs on fx68k and on m68020. The loops are copied from spisd.device (the SF2000/
| Base64 SD driver): single-byte RX ($3436), single-byte TX ($3478), 16-byte bulk RX ($373c)
| and bulk TX ($3760). MISO in the testbench is an incrementing byte counter, so every
| received block must be consecutive. Result long at $100: $600D0000 or $BAD0ttii.
        .text
        .org 0
        .long 0x0007fff0
        .long start
        .org 0x400
start:  move.w  #0x2700,sr
        .ifdef CACHE
        moveq   #9,d0                   | CACR: clear + enable instruction cache (68020 only)
        movec   d0,cacr
        .endif
        clr.l   0x100
        lea     0xE80000,a0
        move.b  #0x20,0x48(a0)          | fastmem: 8 MB at $200000
        move.b  #0x90,0x4A(a0)          | turbomem at $E90000
        move.b  #0xE9,0x48(a0)
        move.b  #0xA0,0x4A(a0)          | SD at $EA0000
        move.b  #0xEA,0x48(a0)
        lea     0xEA0000,a2
        move.w  #CLKDIV,0(a2)           | clk_div (first write switches ROM -> registers)
        move.w  #1,2(a2)                | slave select
        lea     6(a2),a4                | status
        lea     16(a2),a5               | data

| ---- T1: 64 single-byte reads ----
        lea     0x10000,a3
        move.w  #63,d7
1:      move.w  #0x4001,8(a2)
2:      move.w  (a4),d0
        btst    #0,d0                   | rx_cb_empty
        bne.s   2b
        move.b  (a5),(a3)+
        dbf     d7,1b
        lea     0x10000,a3
        move.w  #63,d7
        moveq   #1,d5
        bsr     chkseq

| ---- T2: 16 single-byte writes, 7i+3 ----
        moveq   #3,d6
        moveq   #15,d7
3:      move.w  #0x8000,8(a2)
4:      move.w  (a4),d0
        btst    #3,d0                   | tx_cb_full
        bne.s   4b
        move.b  d6,(a5)
5:      move.w  (a4),d0
        btst    #6,d0                   | shifter busy
        bne.s   5b
        addq.b  #7,d6
        dbf     d7,3b

| ---- T3: bulk 512-byte reads, chip RAM, fast RAM, odd buffer ----
        lea     0x10100,a0
        bsr     bulkrx
        lea     0x10100,a3
        move.w  #511,d7
        moveq   #3,d5
        bsr     chkseq

        lea     0x200000,a0
        bsr     bulkrx
        lea     0x200000,a3
        move.w  #511,d7
        moveq   #4,d5
        bsr     chkseq

        lea     0x200401,a0             | odd buffer: one byte first, as $32e8 does
        move.w  #0x4000+513,8(a2)
6:      move.w  (a4),d0
        btst    #0,d0
        bne.s   6b
        move.b  (a5),(a0)+
        bsr     bulkrx2
        lea     0x200401,a3
        move.w  #512,d7
        moveq   #5,d5
        bsr     chkseq

| ---- T4: bulk 512-byte write from fast RAM, pattern i & $7F ----
        lea     0x200800,a0
        moveq   #0,d0
        move.w  #511,d7
7:      move.b  d0,(a0)+
        addq.b  #1,d0
        and.b   #0x7F,d0
        dbf     d7,7b
        lea     0x200800,a0
        move.w  #0x8000,8(a2)
        moveq   #31,d0
8:      move.w  (a4),d1
        and.w   #32,d1                  | tx at least half empty
        beq.s   8b
        move.l  (a0)+,(a5)
        move.l  (a0)+,(a5)
        move.l  (a0)+,(a5)
        move.l  (a0)+,(a5)
        dbf     d0,8b
9:      move.w  (a4),d1                 | drain: tx buffers empty and shifter idle
        btst    #2,d1
        beq.s   9b
        btst    #6,d1
        bne.s   9b
        move.w  #0x8000,8(a2)
        moveq   #100,d0
10:     dbf     d0,10b
        move.l  #0x600D0000,0x100
        stop    #0x2700

| bulk read 512 bytes to (a0): mode RX, length 512, 32 x 16 bytes
bulkrx: move.w  #0x4000+512,8(a2)
bulkrx2:
        moveq   #31,d0
11:     move.w  (a4),d1
        and.w   #16,d1                  | rx at least half full
        beq.s   11b
        move.l  (a5),(a0)+
        move.l  (a5),(a0)+
        move.l  (a5),(a0)+
        move.l  (a5),(a0)+
        dbf     d0,11b
        rts

| check (a3).. d7+1 bytes are consecutive mod 256; on failure write $BAD0<d5><index> and stop
chkseq: move.b  (a3)+,d0
12:     move.b  (a3)+,d1
        addq.b  #1,d0
        cmp.b   d0,d1
        bne.s   13f
        subq.w  #1,d7
        bgt.s   12b
        rts
13:     move.l  a3,d1
        move.w  #0xBAD0,0x100
        move.b  d5,0x102
        move.b  d1,0x103
        stop    #0x2700
