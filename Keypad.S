#include <xc.h>
#include "configbits.c"
	.global main
    .set noreorder
    .ent main
    
    main:
	//row1 : RA1
	//row2 : RA2
	//row3 : RA3
	//row4 : RA4
	//coloumn1 : RA0
	//coloumn2 : RB0
	//coloumn3 : RB1
	// RB2 : a, RB3:b, RB4:c, RB5:d, RB6:e, RB7:f, RB8:g
	// RB9 : a, RB10:b, RB11:c, RB12:d, RB13:e, RB14:f, RB15:g
	
	

addiu sp, sp, -80
    add s7, zero, sp
    //0x3, 0x1b, 0x16f, 0x13f, 0x19b, 0x1b7, 0x1f7, 0x1f, 0x9e00, 0x1bf};
    //0x7e00, 0xc00, 0xb600, 0x1ff, 0xcc00, 0xda00, 0xfa00, 0xe00, 0xfe00, 0xde00};
    ori t0, zero, 0x3
    sw t0, 0(sp)
	ori t0, zero, 0x1b
    sw t0, 4(sp)
	ori t0, zero, 0x16f
    sw t0, 8(sp)
	ori t0, zero, 0x13f
    sw t0, 12(sp)
	ori t0, zero, 0x19b
    sw t0, 16(sp)
	ori t0, zero, 0x1b7
    sw t0, 20(sp)
	ori t0, zero, 0x1f7
    sw t0, 24(sp)
	ori t0, zero, 0x1f
    sw t0, 28(sp)
	ori t0, zero, 0x1ff
    sw t0, 32(sp)
	ori t0, zero, 0x1bf
    sw t0, 36(sp)
	ori t0, zero, 0x7e00
    sw t0, 40(sp)
	ori t0, zero, 0xc00
    sw t0, 44(sp)
	ori t0, zero, 0xb600
    sw t0, 48(sp)
	ori t0, zero, 0x9e00
    sw t0, 52(sp)
	ori t0, zero, 0xcc00
    sw t0, 56(sp)
	ori t0, zero, 0xda00
    sw t0, 60(sp)
	ori t0, zero, 0xfa00
    sw t0, 64(sp)
	ori t0, zero, 0xe00
    sw t0, 68(sp)
	ori t0, zero, 0xfe00
    sw t0, 72(sp)
	ori t0, zero, 0xde00
    sw t0, 76(sp)
	
la t0, ANSELA
sw zero, 0(t0)
la t0, ANSELB
sw zero, 0(t0)
la t0, TRISA
addiu t1, zero, 1
sw t1, 0(t0)
la t0, TRISB
addiu t1, zero, 3
sw t1, 0(t0)
la t0, LATB
sw t1, 0(t0)
la t0, LATA
    add s0, zero, zero
While:

	addiu t1, zero, 0xfd
	sw t1, 0(t0)
	jal Delaymain
	nop
	addiu t1, zero, 0xfb
	sw t1, 0(t0)
	jal Delaymain
	nop
	addiu t1, zero, 0xf7
	sw t1, 0(t0)
	jal Delaymain
	nop
	addiu t1, zero, 0xef
	sw t1, 0(t0)
	jal Delaymain
	nop

j While
nop

Delaymain:
addiu sp, sp, -12
sw t0, 0(sp)
sw t1, 4(sp)
sw ra, 8(sp)
addiu t0, zero, 500
Delaymain1:
la t1, PORTA
lw t1, 0(t1)
andi t1, 1
beq t1, zero, DoSomething1
nop
la t1, PORTB
lw t1, 0(t1)
andi t1, 3
addiu t1, t1, -3
bne t1, zero, DoSomething1
nop
j continue
nop
DoSomething1:
jal DoSomething
nop
continue:
addiu t0, t0, -1
bne t0, zero, Delaymain1
nop
lw t0, 0(sp)
lw t1, 4(sp)
lw ra, 8(sp)
addiu sp, sp, 12
jr ra
nop

Delay:
addiu sp, sp, -4
sw t0, 0(sp)
lui t0, 0xc
Delay1:
addiu t0, t0, -1
bne t0, zero, Delay1
nop
lw t0, 0(sp)
addiu sp, sp, 4
jr ra
nop
    
    DoSomething:
		addiu sp, sp, -20
		sw t0, 0(sp)
		sw t1, 4(sp)
		sw ra, 8(sp)
		sw t2, 12(sp)
		la t2, LATB
		
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 3
		bne t0, zero, R1C2
		nop
		
		R1C1:
		beq s0, zero, R1C1f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xc00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 1
		j cont1
		nop
		
		R1C1f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x1b
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 1
		j cont1
		nop
		
		cont1:
		jal Delay
		nop
		wait1:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait1
		nop
		j cont 
		nop

		R1C2:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 2
		bne t0, zero, R1C3
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		bne t0, zero, R1C3
		nop
		beq s0, zero, R1C2f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xb600
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 2
		j cont2
		nop
		
		R1C2f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x16f
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 2
		j cont2
		nop
		
		cont2:
		jal Delay
		nop
		wait2:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait2
		nop
		j cont 
		nop
		
		R1C3:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 2
		bne t0, zero, R2C1
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 2
		bne t0, zero, R2C1
		nop
		beq s0, zero, R1C3f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0x9e00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 3
		j cont3
		nop
		
		R1C3f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x13f
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 3
		j cont3
		nop
	
		cont3:
		jal Delay
		nop
		wait3:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 2
		beq t0, zero, wait3
		nop
		j cont 
		nop
		
		R2C1:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 5
		bne t0, zero, R2C2
		nop
		beq s0, zero, R2C1f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xcc00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 4
		j cont4
		nop
		
		R2C1f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x19b
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 4
		j cont4
		nop
		
		cont4:
		jal Delay
		nop
		wait4:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait4
		nop
		j cont 
		nop
		
		R2C2:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 4
		bne t0, zero, R2C3
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		bne t0, zero, R2C3
		nop
		beq s0, zero, R2C2f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xda00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 5
		j cont5
		nop
		
		R2C2f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x1b7
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 5
		j cont5
		nop
	
		cont5:
		jal Delay
		nop
		wait5:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait5
		nop
		j cont 
		nop
	
		R2C3:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 4
		bne t0, zero, R3C1
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 2
		bne t0, zero, R3C1
		nop
		beq s0, zero, R2C3f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xfa00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 6
		j cont6
		nop
		
		R2C3f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x1f7
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 6
		j cont6
		nop

		cont6:
		jal Delay
		nop
		wait6:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 2
		beq t0, zero, wait6
		nop
		j cont 
		nop
		
		R3C1:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 9
		bne t0, zero, R3C2
		nop
		beq s0, zero, R3C1f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xe00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 7
		j cont7
		nop
		
		R3C1f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x1f
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 7
		j cont7
		nop
	
		cont7:
		jal Delay
		nop
		wait7:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait7
		nop
		j cont 
		nop
		
		R3C2:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 8
		bne t0, zero, R3C3
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		bne t0, zero, R3C3
		nop
		beq s0, zero, R3C2f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xfe00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 8
		j cont8
		nop
		
		R3C2f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x1ff
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 8
		j cont8
		nop
	
		cont8:
		jal Delay
		nop
		wait8:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait8
		nop
		j cont 
		nop
		
		R3C3:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 8
		bne t0, zero, R4C1
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 2
		bne t0, zero, R4C1
		nop
		beq s0, zero, R3C3f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0xde00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 9
		j cont9
		nop
		
		R3C3f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0x1bf
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 9
		j cont9
		nop
	
		cont9:
		jal Delay
		nop
		wait9:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 2
		beq t0, zero, wait9
		nop
		j cont 
		nop
		
		R4C1:
		
	
		R4C2:
		la t0, PORTA
		lw t0, 0(t0)
		andi t0, t0, 16
		bne t0, zero, R4C3
		nop
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		bne t0, zero, R4C3
		nop
		beq s0, zero, R4C2f
		nop
		lw t0, 0(t2) //LATB
		ori t1, zero, 0x1ff
		and t0, t0, t1
		ori t1, zero, 0x7e00
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s2, zero, 0
		j cont0
		nop
		
		R4C2f:
		lw t0, 0(t2) //LATB
		ori t1, zero, 0xfe03
		and t0, t0, t1
		ori t1, zero, 0xff
		or t0, t0, t1
		sw t0, 0(t2)
		addiu s1, zero, 0
		j cont0
		nop
	
		cont0:
		jal Delay
		nop
		wait0:
		la t0, PORTB
		lw t0, 0(t0)
		andi t0, t0, 1
		beq t0, zero, wait0
		nop
		j cont 
		nop
		
		R4C3:
		
	
		cont:
		
		
	Return:
		addiu t1, zero, 1
		subu s0, t1, s0
		lw t0, 0(sp)
		lw t1, 4(sp)
		lw ra, 8(sp)
		lw t2, 12(sp)
		addiu sp, sp, 20
		jr ra
		nop

.end main