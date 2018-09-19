lui $t1 , 0x1001
ori $t1 , $t1 , 0x0000

lui $t2 , 0x1001
ori $t2 , $t2 , 0x0004

lui $t3 , 0x1001
ori $t3 , $t3 , 0x0008

lui $t4 , 0x1001
ori $t4 , $t4 , 0x000c

lui $t5 , 0x1001
ori $t5 , $t5 , 0x0010

lui $t6 , 0x1001
ori $t6 , $t6 , 0x0014

lw $s1 , 0($t1)
lw $s2 , 0($t2)
lw $s3 , 0($t3)
lw $s4 , 0($t4)

add $s5 , $s1 , $s3
add $s6 , $s2 , $s4

sltu $t0 , $s5 , $s1
add $s6 , $s6 , $t0

sw $s5 , 0($t5)
sw $s6 , 0($t6)








