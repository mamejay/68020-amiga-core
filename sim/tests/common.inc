| Common prologue for directed lockstep tests.
| Vector table at 0 (all vectors -> default_handler), code starts at 0x400.
| A test ends by executing STOP (the harness treats an upcoming STOP as PASS).
        .text
        .org 0
        .long 0x0007fff0                | reset SSP (ISP)
        .long start                     | reset PC
        .rept 254
        .long default_handler
        .endr
        .org 0x400
default_handler:                        | unexpected exception: record frame and finish
        move.w  6(sp),d7                | format/vector word
        move.l  2(sp),d6                | stacked PC
        stop    #0x2700
        .equ DATA, 0x10000              | scratch RAM used by tests
