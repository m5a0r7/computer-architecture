lui $s0,0x1001
lw $a0,0($s0)
addi $s1,$0,1
addi $t0,$0,9
addi $t1,$0,10
addi $s3,$0,1
addi $t2,$0,0x00000F
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
sw $v0,4($s0)