| Interrupt tests (prog mode). A byte write to IRQ_PORT sets the IPL level seen by
| both Musashi and the RTL (harness hook). Autovectors 25..31.
        .equ DATA, 0x10000
        .equ IRQ_PORT, 0x00DFF0F0
        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        move.l  #h_lvl3,0x6c.w          | autovector level 3 (vector 27)
        move.l  #h_lvl5,0x74.w          | autovector level 5 (vector 29)
        move.l  #h_nmi,0x7c.w           | autovector level 7 (vector 31)
        moveq   #0,d0
        moveq   #0,d1
        moveq   #0,d2
        lea     0x7fff0,a0
        movec   a0,isp
        lea     0x6fff0,a0
        movec   a0,msp

        | 1. masked: level 3 with mask 7 is not taken
        move.b  #3,IRQ_PORT
        nop
        nop
        | 2. lower the mask -> taken immediately (handler clears the request)
        move.w  #0x2200,sr
        nop
        nop
        | 3. level 5 while mask 4: taken; level 3 while mask 4: not taken
        move.w  #0x2400,sr
        move.b  #3,IRQ_PORT
        nop
        move.b  #5,IRQ_PORT
        nop
        move.b  #0,IRQ_PORT
        | 4. NMI (level 7) is taken even with mask 7, once per edge
        move.w  #0x2700,sr
        move.b  #7,IRQ_PORT
        nop
        nop
        nop
        move.b  #0,IRQ_PORT
        nop
        | 5. M bit set: interrupt frame on MSP, throwaway frame on ISP, RTE unwinds both
        move.w  #0x3200,sr              | S=1 M=1 mask 2
        move.l  sp,d5
        move.b  #3,IRQ_PORT
        nop
        nop
        move.l  sp,d6                   | must be back on the master stack, same SP
        | 6. interrupt from user mode
        move.w  #0x0200,sr              | user mode, mask 2
        move.b  #5,IRQ_PORT
        nop
        nop
        trap    #0                      | back to supervisor via default path? (uses default_handler)
h_lvl3: addq.l  #1,d0
        move.w  sr,d3
        move.b  #0,IRQ_PORT             | acknowledge
        rte
h_lvl5: addq.l  #1,d1
        move.w  sr,d4
        move.b  #0,IRQ_PORT
        rte
h_nmi:  addq.l  #1,d2
        rte
