        .include "tests/common.inc"
start:  move.w  #0x2700,sr
        move.l  #0x12345678,d0
        bftst   d0{#4:8}
        bfextu  d0{#4:8},d1
        bfexts  d0{#28:8},d1            | wraps around in register
        bfextu  d0{#0:0},d1             | width 0 = 32
        bfchg   d0{#30:4}
        bfclr   d0{#1:3}
        bfset   d0{#16:16}
        bfffo   d0{#8:24},d2
        moveq   #0,d3
        bfffo   d3{#4:12},d2            | not found -> offset+width
        move.l  #0xabcdef01,d4
        bfins   d4,d0{#12:12}
        moveq   #-3,d5                  | negative register offset
        moveq   #9,d6
        bfextu  d0{d5:d6},d1
        lea     DATA,a0
        move.l  #0x89abcdef,(a0)
        move.l  #0x01234567,4(a0)
        move.l  #0xfedcba98,8(a0)
        bftst   (a0){#0:32}
        bfextu  (a0){#5:30},d1          | spans 5 bytes
        moveq   #-12,d5
        bfexts  4(a0){d5:#20},d1        | negative offset into previous bytes
        moveq   #43,d5
        bfextu  (a0){d5:d6},d1
        moveq   #-40,d5
        bfexts  8(a0){d5:#17},d1
        bfchg   (a0){#7:25}
        bfclr   1(a0){#3:2}
        moveq   #60,d5
        bfset   (a0){d5:#8}
        bfins   d4,(a0){#13:31}
        bfffo   (a0){#2:30},d2
        bfffo   2(a0){d5:d6},d2
        moveq   #0,d7
        bfins   d7,(a0){#0:16}
        bfffo   (a0){#0:16},d2
        moveq   #4,d0
        stop    #0x2700
