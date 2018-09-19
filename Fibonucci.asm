main:
lui $s0,0x1001
lw $a0,0($s0)
jal fibonucci
sw $v0,4($s0)
j end
fibonucci:
addi $sp,$sp,-4
sw $ra,0($sp)
addi $t0,$0,2
slt $t0,$a0,$t0
beq $t0,$0,else
addi $v0,$0,1
addi $sp,$sp,4
jr $ra
else:
addi $t0,$0,2
bne $a0,$t0,else2
addi $v0,$0,1
addi $t1,$0,1
addi $sp,$sp,4
jr $ra
else2:
addi $a0,$a0,-1
jal fibonucci
lw $ra,0($sp)
addi $sp,$sp,4
add $t2,$v0,$0
add $v0,$v0,$t1
add $t1,$t2,$0
jr $ra
end: