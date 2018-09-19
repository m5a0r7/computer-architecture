main:
lui $s0,0x1001
lw $s1,0($s0)
addi $t0,$0,0x00FF
sll $t1,$t0,8
addi $t4,$0,365
addi $t5,$0,7
addi $t6,$0,31
and $t0,$t0,$s1
and $t1,$t1,$s1
srl $t1,$t1,8
srl $t2,$s1,16
year:
add $a0,$t2,$0
jal BCD2Binary
addi $s2,$v0,-1300
srl $t7,$s2,2
mult $s2,$t4
mflo $s2
add $s2,$t7,$s2
mounth:
add $a0,$t1,$0
jal BCD2Binary
addi $v0,$v0,-1
sltu $t5,$v0,$t5
beq $t5,$0,SecondHalfOfYear
mult $v0,$t6
mflo $s3
j mounthend
SecondHalfOfYear:
addi $v0,$v0,-6
addi $t6,$t6,-1
mult $t6,$v0
mflo $s3
addi $s3,$s3,186
mounthend:
day:
add $a0,$t0,$0
jal BCD2Binary
addi $s4,$v0,-1
SummingTheDays:
add $s5,$s2,$s3
add $s5,$s4,$s5
add $a0,$s5,$0
jal Binary2BCD
sw $v0,4($s0)
j end






BCD2Binary:
addi $sp,$sp,-24
sw $t0,20($sp)
sw $t1,16($sp)
sw $t2,12($sp)
sw $s1,8($sp)
sw $s2,4($sp)
sw $s3,0($sp)
addi $s1,$0,1
addi $t0,$0,9
addi $t1,$0,10
addi $s3,$0,1
addi $t2,$0,0x00000F
add $v0,$0,$0
for:
beq $s1,$t0,done
and $s2,$a0,$t2
multu $s2,$s3
mflo $s2
add $v0,$v0,$s2
srl $a0,$a0,4
multu $s3,$t1
mflo $s3
addi $s1,$s1,1
j for
done:
lw $t0,20($sp)
lw $t1,16($sp)
lw $t2,12($sp)
lw $s1,8($sp)
lw $s2,4($sp)
lw $s3,0($sp)
addi $sp,$sp,24
jr $ra




Binary2BCD:
addi $s1,$0,1
addi $t0,$0,9
addi $t1,$0,10
add $t3,$0,$0
add $v0,$0,$0
add $v1,$0,$0
for2:
beq $s1,$t0,firstBCDregdone
divu $a0,$t1
mflo $a0
mfhi $t2
sllv $t2,$t2,$t3
add $v0,$v0,$t2
addi $t3,$t3,4
addi $s1,$s1,1
j for2
firstBCDregdone:
divu $a0,$t1
mflo $a0
mfhi $t2
sll $a0,$a0,4
add $v1,$a0,$t2
jr $ra


end: