lui $t0 , 0x1001
ori $t0 , $t0 , 0x0000

lui $t1 , 0x1001
ori $t1 , $t1 , 0x0004

lui $t2 , 0x1001
ori $t2 , $t2 , 0x0008

lui $t3 , 0x1001
ori $t3 , $t3 , 0x000c

lui $t4 , 0x1001
ori $t4 , $t4 , 0x0010

lui $t5 , 0x1001
ori $t5 , $t5 , 0x0014

lui $t6 , 0x1001
ori $t6 , $t6 , 0x0018

lui $t7 , 0x1001
ori $t7 , $t7 , 0x001c

lw $s0 , 0($t0)
lw $s1 , 0($t1)
lw $s2 , 0($t2)
lw $s3 , 0($t3)

add $s4 , $s4 , $0
add $s5 , $s5 , $0
add $s6 , $s6 , $0
add $s7 , $s7 , $0

multu $s0 , $s2
mflo $s4
mfhi $s5

multu $s0 , $s3
mflo $a0
addu $s5 , $s5 , $a0
sltu $a1 , $s5 , $a0
addu $s6 , $s6 , $a1
mfhi $a0
addu $s6 , $s6 , $a0

multu $s1 , $s2
mflo $a0
addu $s5 , $s5 , $a0
sltu $a1 , $s5 , $a0
addu $s6 , $s6 , $a1
mfhi $a0
addu $s6 , $s6 , $a0
sltu $a1 , $s6 , $a0
addu $s7 , $s7 , $a1

multu $s1 , $s3
mflo $a0
addu $s6 , $s6 , $a0
sltu $a1 , $s6 , $a0
addu $s7 , $s7 , $a1
mfhi $a0
addu $s7 , $s7 , $a0

sw $s4 , 0($t4) 
sw $s5 , 0($t5)
sw $s6 , 0($t6)
sw $s7 , 0($t7)










