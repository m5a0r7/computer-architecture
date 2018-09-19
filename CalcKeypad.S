#include <xc.h>
#include "configbits.c"
    .global main
    .set noreorder
.ent main
main:
    la t0,ANSELA
    sw zero,0(t0)
    la t0,ANSELB
    sw zero,0(t0)
    
    ori t1,zero,16
    la t0,TRISA
    sw t1,0(t0)
    ori t1,zero,3
    la t0,TRISB
    sw t1,0(t0)
    
    addiu sp,sp,-40
    addiu t0,zero,0x3F //digit 0
    sw t0,0(sp)
    addiu t0,zero,0x6 //digit 1
    sw t0,4(sp)
    addiu t0,zero,0x5B//digit 2
    sw t0,8(sp)
    addiu t0,zero,0x4F //digit 3
    sw t0,12(sp)
    addiu t0,zero,0x66 //digit 4
    sw t0,16(sp)
    addiu t0,zero,0x6D //digit 5
    sw t0,20(sp)
    addiu t0,zero,0x7C //digit 6
    sw t0,24(sp)
    addiu t0,zero,0x7 //digit 7
    sw t0,28(sp)
    addiu t0,zero,0x7F //digit 8
    sw t0,32(sp)
    addiu t0,zero,0x67 //digit 9
    sw t0,36(sp)
    
    la s0,PORTA
    la s1,LATA
    sw zero,0(s1)////////////
    la s2,PORTB
    la s3,LATB
    sw zero,0(s3)///////////
    addiu t0,zero,1//row shifter
    add t1,zero,zero//counter=row
    addiu t2,zero,4//coloumn
    addiu t3,zero,0//step[0:1]
    add s5,zero,zero //number pressed in keypad
   
    
    loop:
    //Catching pressed key
    nor t4,t0,t0
    andi t4,t4,0x000f //////////////
    sw t4,0(s1)
  
    sll t0,t0,1
    lw t4,0(s0)
    lw t5,0(s2)
    andi t4,t4,0x0010
    srl t4,t4,4 ////////////
    andi t5,t5,0x0003
    sll t5,t5,1
    add t4,t4,t5//Input[0:2]
    bne t4,0x6,nopress1/////////
    nop
    add t2,zero,zero/////fff/f////
    j process
    nop
    nopress1:
    bne t4,0x5,nopress2//////////
    nop
    addiu t2,zero,1
    j process
    nop
    nopress2:
    bne t4,0x3,nopress3//////////
    nop
    addiu t2,zero,2
    //start pressed state process
    process:
    addiu t4,zero,3
    multu t4,t1
    mflo t4
    add t4,t4,t2 //position of key
    bne t4,1,notzero
    nop
    add s5,zero,zero
    j keydecoded
    nop
    notzero:
    addiu s5,t4,-2
    keydecoded:
    bne t3,0,step1
    nop
    addiu t4,zero,4
    multu s5,t4
    mflo t4
    add t4,t4,sp
    lw t4,0(t4)
    sll t4,t4,2
    sw t4,0(s3)
    addiu t3,zero,1
    j endprocess
    nop
    step1:
    addiu t4,zero,4
    multu s5,t4
    mflo t4
    add t4,t4,sp
    lw t4,0(t4)
    sll t4,t4,9
    lw t5,0(s3)
    or t4,t4,t5
    sw t4,0(s3)
    addu t3,zero,zero
    endprocess:
    addiu t2,zero,4 //reload default colomun
    add t5,zero,zero
    jal delay
    nop
    // end of pressed state process
    nopress3:
    addiu t1,t1,1//counter update
    bne t1,4,loop
    nop
    addiu t0,zero,1
    add t1,zero,zero
    
    j loop
    nop
    
    delay:
    beq t5,1000000,out
    nop
    addi t5,t5,1
    j delay
    nop
    out:
    jr ra
    nop
.end main
    