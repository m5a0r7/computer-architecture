lui $s0,0x1001
lw $a0,0($s0)
addi $s1,$0,1
addi $t0,$0,9
addi $t1,$0,10
add $t3,$0,$0
add $v0,$0,$0
add $v1,$0,$0
for:
beq $s1,$t0,firstBCDregdone
divu $a0,$t1
mflo $a0
mfhi $t2
sllv $t2,$t2,$t3
add $v0,$v0,$t2
addi $t3,$t3,4
addi $s1,$s1,1
j for
firstBCDregdone:
divu $a0,$t1
mflo $a0
mfhi $t2
sll $a0,$a0,4
add $v1,$a0,$t2
sw $v0,4($s0)
sw $v1,8($s0)
