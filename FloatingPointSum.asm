lui $s0,0x0080
addi $s0,$s0,0x0007
lui $s1,0x0180
addi $s1,$s1,0x0003
addi $t0,$0,1
#making first reg s0 for bigger number
slt $t1,$s0,$s1
beq $t1,$0,FirstBigger
add $t1,$s0,$0
add $s0,$s1,$0
add $s1,$t1,$0
FirstBigger:
#Filtering
#Exponents
addi $t1,$0,0x00ff
sll $t1,$t1,23
and $t2,$s0,$t1
srl $t2,$t2,23
and $t3,$s1,$t1
srl $t3,$t3,23
#Fractions
addi $t1,$0,0x7fff
sll $t1,$t1,8
addi $t1,$t1,0x00ff
and $t4,$s0,$t1
and $t5,$s1,$t1
sll $t1,$t0,23
add $t4,$t4,$t1
add $t5,$t5,$t1
#number of shifting
sub $s2,$t2,$t3
addi $t1,$0,25
slt $t1,$s2,$t1
beq $t1,$0,TooMuchShift
#shifting
srlv $s3,$t5,$s2
sllv $t1,$s3,$s2
sub $s4,$t5,$t1 #s4 is lost bits after shifting
add $s5,$t4,$s3
#normalizing
sll $t1,$t0,24
and $t1,$s5,$t1
beq $t1,$0,normalized
and $t1,$s5,$t0
sllv $t1,$t1,$s2
or $s3,$t1,$s3#is for updating the bits for rounding
srl $s5,$s5,1
addi $t2,$t2,1
normalized:
#Rounding-process
addi $t1,$s2,-1
sllv $t1,$t0,$t1
slt $t6,$t1,$s4
beq $t6,$0,NoG1S1
addi $s5,$s5,1
j else
NoG1S1:
bne $t1,$s3,NoG1S0
and $t6,$s5,$t0
add $s5,$s5,$t6
NoG1S0:
#add $s5,$s5,$0
else:
j short
TooMuchShift:
add $s5,$t4,$0 #shifted number forcely become zero
short:
#normalizing
sll $t1,$t0,24
and $t1,$s5,$t1
beq $t1,$0,normalized2
srl $s5,$s5,1
addi $t2,$t2,1
normalized2:
#IEEE-Format
sll $t1,$t0,23
addi $t1,$t1,-1#filter the first 23 bit of $s5
and $s5,$s5,$t1
sll $t2,$t2,23
add $s5,$s5,$t2

