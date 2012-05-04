#pragma once

#define DUMP_MARKER 0x43FA1AA5
#define MAX_STACK_DUMP 512

#define ARM_USER_MODE       0x10
#define ARM_FIQ_MODE        0x11
#define ARM_IRQ_MODE        0x12
#define ARM_SVC_MODE        0x13
#define ARM_ABORT_MODE      0x17
#define ARM_UNDEF_MODE      0x1B
#define ARM_SYS_MODE        0x1F

#define ARM_CPSR_I_BIT      0x80
#define ARM_CPSR_F_BIT      0x40
#define ARM_CPSR_T_BIT      0x20

#define MARK_DATA_ABORT     0
#define MARK_PREFETCH_ABORT 1
#define MARK_UNDEFINED_OP   2

#define CRASH_BUFFER_R0     8
#define CRASH_BUFFER_R8     40
#define CRASH_BUFFER_R13    60
#define CRASH_BUFFER_STACK  92

#if __ASSEMBLER__

.macro bad_exception_enter, mode, abort_type

    msr	cpsr_c, #((\mode) | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT) // disable the interrupts while staying in the current mode
    // refer to "struct crash_dump"
    // save registers/context to a buffer in memory for debugging purposes
    ldr sp, =crash_dump_buffer // use the stack pointer to load our output address
    add sp, sp, #CRASH_BUFFER_R0 // go to r0's offset in the buffer
    stmia sp, {r0-r7} // save r0-r7, the unbanked registers - they are the same as they were in the mode leading to the crash
    // we are now free to use r0-r7

    ldr sp, =crash_dump_buffer
    ldr r0, =DUMP_MARKER // mark crash as valid
    stmia sp!, {r0}
    ldr r0, =(\abort_type) // mark crash type
    stmia sp, {r0}

    // we now need to go back to the crashed mode, to retrieve the banked registers
    ldr r0, =crash_dump_buffer
    add r0, r0, #CRASH_BUFFER_R8 // go to r8's offset in the buffer
    mrs r1, spsr
    orr r1, #(ARM_CPSR_F_BIT | ARM_CPSR_I_BIT) // make sure the interrupts stay disabled
    msr	cpsr_c, r1
    stmia r0!, {r8-r14} // save the banked registers

    // return to the abort mode, and grab the rest of the useful info
    msr	cpsr_c, #((\mode) | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
    mov sp, r0 // go back to using sp to point into our abort buffer
    sub lr, lr, #4
    .if	((\abort_type) != MARK_PREFETCH_ABORT)
	sub	lr, lr, #4
	.endif
    stmia sp!, {lr}
    mrc p15, 0, r0, c6, c0, 0 // read FAR into r0
    stmia sp!, {r0}
    mrc p15, 0, r0, c5, c0, 0 // read DFSR into r0
    stmia sp!, {r0}
    mrc p15, 0, r0, c5, c0, 1 // read IFSR into r0
    stmia sp!, {r0}
    mrs r0, spsr // save CPSR snapped at instant of crash
    stmia sp!, {r0}
    mrs r0, cpsr // save CPSR as well, could be helpful
    stmia sp!, {r0}

    // dump the stack, using the stack pointer retrieved from the crashing mode
    ldr sp, =crash_dump_buffer
    ldr r0, [sp, #CRASH_BUFFER_R13]
    add r1, sp, #CRASH_BUFFER_STACK
    add r2, r1, #MAX_STACK_DUMP
    bl stack_copy

    .if	((\mode) != ARM_ABORT_MODE)
    ldr sp, =__stack_abt_end__ // restore sp
    .endif
    .if	((\mode) != ARM_UNDEF_MODE)
    ldr sp, =__stack_und_end__ // restore sp
    .endif
.endm

// macros taken from EtherNut, for inspiration
/*
.macro bad_exception_enter, whatmode, frametype

	.if	((\whatmode) != ARM_SVC_MODE)
	sub	lr,lr,#4
	.endif
	// switch to system mode
	msr	cpsr_c,#(ARM_SYS_MODE | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
	
	// make room for PTRACE frame
	sub	sp,sp,#(PTRACE_FRAME_size*4)

    // save the world
	stmia sp,{r0-r14}

	// save the return value here
	str	r0,[sp,#(PTRACE_R0_retval_idx*4)]
	
	// Need to go back to old mode and pickup things 
	// When we get there, we will need the sys stack
	mov	r0,sp

	// step over to the dark side
	msr	cpsr_c,#((\whatmode) | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
	// back in offending mode

	// save the exception address
	str	lr,[r0,#(4*PTRACE_R15_idx)]

	// And the saved PSR
	mrs	r1,spsr
	str	r1,[r0,#(4*PTRACE_CPSR_idx)]
	// and back to supervisor mode
	msr	cpsr_c,#(ARM_SYS_MODE | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
	// done, w/ IRQ & FIQ disabled
	// APP can decide to re-enable if *IT* wants to
	.if	((\frametype) != 0)
	mov	r1,#(\frametype)
	str	r1,[r0,#(4*PTRACE_FRAMETYPE_idx)]
	.endif
.endm

.macro	bad_exception_exit, whatmode
	// SP = the saved state
	// we always restore via SYSMODE

	// pre-position some values
	ldr	r0,[sp,#(PTRACE_R15_idx*4)]
	ldr	r1,[sp,#(PTRACE_CPSR_idx*4)]

	// go to the offending mode
	msr	cpsr_c,#((\whatmode) | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
	
	mov	lr,r0
	msr	spsr,r1

	// back to the main mode
	msr	cpsr_c,#(ARM_SYS_MODE | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
	// restore the world
	ldmia	sp,{r0-r14}
	.if	((\whatmode) == ARM_SVC_MODE)
	ldr	r0,[sp,#(PTRACE_R0_retval_idx*4)]
	.endif
	add	sp,sp,#(PTRACE_FRAME_size*4)

	// back to the offending mode
	msr	cpsr_c,#((\whatmode) | ARM_CPSR_F_BIT | ARM_CPSR_I_BIT)
	// and effect the return
	movs	pc,lr

.endm
*/

#endif