lui $t0,0xeee9
ori $t0,$t0,0x4560
lui $t1,0x0154
ori $t1,$t1,0xa8d0
lui $t2,0x1800
ori $t2,$t2,0x2e00
lui $t3,0x0000
ori $t3,$t3,0x102A
addu $t4,$t0,$t2
addu $t5,$t1,$t3
sltu $t6,$t4,$t0
addu $t5,$t5,$t6