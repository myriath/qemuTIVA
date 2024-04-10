//*****************************************************************************
//
// Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2011-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//*****************************************************************************

#include "tm4c123gh6pm_kernel.h"
#define VECTOR_COUNT 155
#define NUM_INTERRUPTS 138

#define HWREG(adr) (*((volatile uint32_t *)adr))

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
void ExitQEMU(void);
static void NmiSR(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the main function to be called when the processor
// restarts.
//
//*****************************************************************************
extern void main(void);

//*****************************************************************************
//
// Linker variables that mark the different placements of sections
//
//*****************************************************************************
extern unsigned int _sram_stacktop;
extern unsigned int _flash_sdata;
extern unsigned int _sram_sdata;
extern unsigned int _sram_edata;
extern unsigned int _sram_sbss;
extern unsigned int _sram_ebss;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//****************************************************************************

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
__attribute__((used, section(".vectors")))
static void (* const vectors[VECTOR_COUNT])(void) =
{
    (void (*)(void))(&_sram_stacktop),
    ResetISR,                               // Stack top
    NmiSR,                                  // Reset
    ExitQEMU,                               // NMI
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
    IntDefaultHandler,                      // The SysTick handler
    IntDefaultHandler,                      // GPIO Port A
    IntDefaultHandler,                      // GPIO Port B
    IntDefaultHandler,                      // GPIO Port C
    IntDefaultHandler,                      // GPIO Port D
    IntDefaultHandler,                      // GPIO Port E
    IntDefaultHandler,                      // UART0 Rx and Tx
    IntDefaultHandler,                      // UART1 Rx and Tx
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // I2C0 Master and Slave
    IntDefaultHandler,                      // PWM Fault
    IntDefaultHandler,                      // PWM Generator 0
    IntDefaultHandler,                      // PWM Generator 1
    IntDefaultHandler,                      // PWM Generator 2
    IntDefaultHandler,                      // Quadrature Encoder 0
    IntDefaultHandler,                      // ADC Sequence 0
    IntDefaultHandler,                      // ADC Sequence 1
    IntDefaultHandler,                      // ADC Sequence 2
    IntDefaultHandler,                      // ADC Sequence 3
    IntDefaultHandler,                      // Watchdog timer
    IntDefaultHandler,                      // Timer 0 subtimer A
    IntDefaultHandler,                      // Timer 0 subtimer B
    IntDefaultHandler,                      // Timer 1 subtimer A
    IntDefaultHandler,                      // Timer 1 subtimer B
    IntDefaultHandler,                      // Timer 2 subtimer A
    IntDefaultHandler,                      // Timer 2 subtimer B
    IntDefaultHandler,                      // Analog Comparator 0
    IntDefaultHandler,                      // Analog Comparator 1
    IntDefaultHandler,                      // Analog Comparator 2
    IntDefaultHandler,                      // System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // FLASH Control
    IntDefaultHandler,                      // GPIO Port F
    IntDefaultHandler,                      // GPIO Port G
    IntDefaultHandler,                      // GPIO Port H
    IntDefaultHandler,                      // UART2 Rx and Tx
    IntDefaultHandler,                      // SSI1 Rx and Tx
    IntDefaultHandler,                      // Timer 3 subtimer A
    IntDefaultHandler,                      // Timer 3 subtimer B
    IntDefaultHandler,                      // I2C1 Master and Slave
    IntDefaultHandler,                      // Quadrature Encoder 1
    IntDefaultHandler,                      // CAN0
    IntDefaultHandler,                      // CAN1
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // Hibernate
    IntDefaultHandler,                      // USB0
    IntDefaultHandler,                      // PWM Generator 3
    IntDefaultHandler,                      // uDMA Software Transfer
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // ADC1 Sequence 0
    IntDefaultHandler,                      // ADC1 Sequence 1
    IntDefaultHandler,                      // ADC1 Sequence 2
    IntDefaultHandler,                      // ADC1 Sequence 3
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port J
    IntDefaultHandler,                      // GPIO Port K
    IntDefaultHandler,                      // GPIO Port L
    IntDefaultHandler,                      // SSI2 Rx and Tx
    IntDefaultHandler,                      // SSI3 Rx and Tx
    IntDefaultHandler,                      // UART3 Rx and Tx
    IntDefaultHandler,                      // UART4 Rx and Tx
    IntDefaultHandler,                      // UART5 Rx and Tx
    IntDefaultHandler,                      // UART6 Rx and Tx
    IntDefaultHandler,                      // UART7 Rx and Tx
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C2 Master and Slave
    IntDefaultHandler,                      // I2C3 Master and Slave
    IntDefaultHandler,                      // Timer 4 subtimer A
    IntDefaultHandler,                      // Timer 4 subtimer B
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // Timer 5 subtimer A
    IntDefaultHandler,                      // Timer 5 subtimer B
    IntDefaultHandler,                      // Wide Timer 0 subtimer A
    IntDefaultHandler,                      // Wide Timer 0 subtimer B
    IntDefaultHandler,                      // Wide Timer 1 subtimer A
    IntDefaultHandler,                      // Wide Timer 1 subtimer B
    IntDefaultHandler,                      // Wide Timer 2 subtimer A
    IntDefaultHandler,                      // Wide Timer 2 subtimer B
    IntDefaultHandler,                      // Wide Timer 3 subtimer A
    IntDefaultHandler,                      // Wide Timer 3 subtimer B
    IntDefaultHandler,                      // Wide Timer 4 subtimer A
    IntDefaultHandler,                      // Wide Timer 4 subtimer B
    IntDefaultHandler,                      // Wide Timer 5 subtimer A
    IntDefaultHandler,                      // Wide Timer 5 subtimer B
    IntDefaultHandler,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C4 Master and Slave
    IntDefaultHandler,                      // I2C5 Master and Slave
    IntDefaultHandler,                      // GPIO Port M
    IntDefaultHandler,                      // GPIO Port N
    IntDefaultHandler,                      // Quadrature Encoder 2
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port P (Summary or P0)
    IntDefaultHandler,                      // GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    IntDefaultHandler,                      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
    IntDefaultHandler,                      // PWM 1 Generator 0
    IntDefaultHandler,                      // PWM 1 Generator 1
    IntDefaultHandler,                      // PWM 1 Generator 2
    IntDefaultHandler,                      // PWM 1 Generator 3
    IntDefaultHandler                       // PWM 1 Fault
};

// RAM Vector table, for dynamic interrupts
__attribute__((used, section(".vectors_dynamic"))) 
static void (*vectors_ram[VECTOR_COUNT])(void) __attribute__((aligned(1024)));

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    unsigned int *src, *dst;

    // Relocate the flash .data to the sram .data.
    src = &_flash_sdata;
    dst = &_sram_sdata;
    while (dst < &_sram_edata)
        *dst++ = *src++;

    dst = &_sram_sbss;
    while (dst < &_sram_ebss)
        *dst++ = 0;

    main();

    ExitQEMU();
}

// Exits QEMU cleanly via semihosting call 0x18

void ExitQEMU(void)
{
    __asm__(
        " .global _exit_qemu\n"
        " _exit_qemu:\n"                // Exits QEMU on finish cleanly
        "     mov r0, #0x18\n"          // angel_SWIreason_ReportException
        "     ldr r1, =#0x20026\n"      // ADP_Stopped_ApplicationExit
        "     bkpt #0xab"               // create exception with semihosting
    );
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    // Do nothing and return. Infinite loops would be bad
    return;
}

// Executes the CPSIE instruction to enable interrupts.
uint32_t CPU_CPSIE(void)
{
    __asm__(
        "   mrs     r0, PRIMASK\n"
        "   cpsie   i\n"
        "   bx      lr\n"
    );
    // Unreachable; for the compiler
    return 0;
};

// Executes the CPSID instruction to disable interrupts.
uint32_t CPU_CPSID(void)
{
    __asm__(
        "   mrs     r0, PRIMASK\n"
        "   cpsid   i\n"
        "   bx      lr\n"
    );
    // Unreachable; for the compiler
    return 0;
};

// Enable interrupts
bool IntMasterEnable()
{
    return(CPU_CPSIE());
}

// Disable interrupts
bool IntMasterDisable()
{
    return(CPU_CPSID());
}

// Handle dynamic interrupts.
void IntRegister(uint32_t interrupt, void (*handler)(void))
{
    // Check the interrupt number is valid
    if (interrupt >= VECTOR_COUNT) return;
    // Check the vector table is aligned 1024
    if ((uint32_t)vectors_ram & 0x3ff != 0) return;

    // Initialize the RAM vector table, if it hasn't been
    if (NVIC_VTABLE_R != (uint32_t)vectors_ram) {
        // Copy the vectors to the new table
        int i;
        for (i = 0; i < VECTOR_COUNT; i++) {
            vectors_ram[i] = vectors[i];
        }
        // Point the NVIC to the RAM vector table
        NVIC_VTABLE_R = (uint32_t)vectors_ram;
    }

    vectors_ram[interrupt] = handler;
}

void IntUnregister(uint32_t interrupt)
{
    if (interrupt >= VECTOR_COUNT) return;

    switch (interrupt) {
        case 0:
            vectors_ram[0] = (void (*)(void))(_sram_stacktop);    // Stack
            break;
        case 1:
            vectors_ram[1] = ResetISR;          // Reset
            break;
        case 2:
            vectors_ram[2] = NmiSR;             // NMI
            break;
        case 3:
            vectors_ram[3] = ExitQEMU;          // Hard Fault
            break;
        default:
            vectors_ram[interrupt] = IntDefaultHandler;
    }
}

// Sets the priority grouping for the interrupt controller.
void IntPriorityGroupingSet(uint32_t bits)
{
    // TODO: there should be a check here, not sure what the real value is tho

    NVIC_APINT_R = NVIC_APINT_R & ~NVIC_APINT_PRIGROUP_M | ((7 - bits) << 8);
}

// Gets the priority grouping for the interrupt controller.
uint32_t IntPriorityGroupingGet(void)
{
    // TODO: there should be a check here, not sure what the real value is tho

    return 7 - ((NVIC_APINT_R & NVIC_APINT_PRIGROUP_M) >> 8);
}

const uint32_t NVIC_PRI_BASE = 0xe000e400;

// Set an interrupt's new priority
void IntPrioritySet(uint32_t interrupt, uint8_t priority)
{
    if (interrupt < 4 || interrupt >= VECTOR_COUNT) return;

    uint32_t reg = NVIC_PRI_BASE + (interrupt & ~3);
    uint32_t shift = 8 * (interrupt & 3) + 5;
    uint32_t new_priority = (priority & 0x7) << shift;

    uint32_t reg_value = HWREG(reg);
    HWREG(reg) = reg_value & ~(7 << shift) | new_priority;
}

// Gets the priority of the interrupt
uint32_t IntPriorityGet(uint32_t interrupt)
{
    if (interrupt < 4 || interrupt >= VECTOR_COUNT) return 0;

    uint32_t reg = interrupt & ~3;
    uint32_t field = interrupt & 3;

    return ((HWREG(NVIC_PRI_BASE + reg) >> (8 * field + 5)) & 0xff);
}

static const uint32_t NVIC_EN_BASE = 0xe000e100;

// Enable the given interrupt in the NVIC.
// Interrupt numbers are offset 16 from the vector table
void IntEnable(uint32_t interrupt)
{
    if (interrupt >= VECTOR_COUNT) return;

    switch (interrupt) {
        case 3:     // MPU Fault
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM;
            break;
        case 4:     // BUS Fault
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUS;
            break;
        case 5:     // Usage Fault
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE;
            break;
        case 14:    // SysTick Fault
            NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN;
            break;
        default:
            if (interrupt < 16) return;
            // Vector Numbers in NVIC are offset 16 from the actual vector table
            interrupt -= 16;
            // en# register |= mask
            uint32_t reg = (interrupt >> 5) << 2;
            uint32_t field = interrupt & 0x1f;

            HWREG(NVIC_EN_BASE + reg) |= 1 << field;
    }
}

static const uint32_t NVIC_DIS_BASE = 0xe000e180;

// Disable the given interrupt
void IntDisable(uint32_t interrupt)
{
    if (interrupt >= VECTOR_COUNT) return;

    switch (interrupt) {
        case 3:     // MPU Fault
            NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEM;
            break;
        case 4:     // BUS Fault
            NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_BUS;
            break;
        case 5:     // Usage Fault
            NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_USAGE;
            break;
        case 14:    // SysTick Fault
            NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_INTEN;
            break;
        default:
            if (interrupt < 16) return;
            // Vector Numbers in NVIC are offset 16 from the actual vector table
            interrupt -= 16;
            // en# register |= mask
            uint32_t reg = (interrupt >> 5) << 2;
            uint32_t field = interrupt & 0x1f;

            HWREG(NVIC_DIS_BASE + reg) |= 1 << field;
    }
}

// Check if the interrupt is enabled
uint32_t IntIsEnabled(uint32_t interrupt)
{
    if (interrupt >= VECTOR_COUNT) return 0;

    switch (interrupt) {
        case 3:     // MPU Fault
            return NVIC_SYS_HND_CTRL_R & NVIC_SYS_HND_CTRL_MEM;
        case 4:     // BUS Fault
            return NVIC_SYS_HND_CTRL_R & NVIC_SYS_HND_CTRL_BUS;
        case 5:     // Usage Fault
            return NVIC_SYS_HND_CTRL_R & NVIC_SYS_HND_CTRL_USAGE;
        case 14:    // SysTick Fault
            return NVIC_ST_CTRL_R & NVIC_ST_CTRL_INTEN;
        default:
            if (interrupt < 16) return 0;
            // NVIC interrupts are offset 16
            interrupt -= 16;
            // each register holds 32 interrupts, each register offset is 4: 
            // interrupt / 32 * 4 = interrupt / 8 = interrupt >> 3
            uint32_t reg = (interrupt >> 5) << 2;
            uint32_t field = interrupt & 0x1f;
            return HWREG(NVIC_EN_BASE + reg) & (1 << field);
    }
}

static const uint32_t NVIC_PEND_BASE = 0xe000e200;

// Pend an interrupt
void IntPendSet(uint32_t interrupt)
{
    if (interrupt >= VECTOR_COUNT) return;

    switch (interrupt) {
        case 2:     // NMI Fault
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_NMI_SET;
            break;
        case 10:    // PendSV Fault
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        case 14:    // SysTick Fault
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PENDSTSET;
            break;
        default:
            if (interrupt < 16) return;
            // Vector Numbers in NVIC are offset 16 from the actual vector table
            interrupt -= 16;
            // en# register |= mask
            uint32_t reg = (interrupt >> 5) << 2;
            uint32_t field = interrupt & 0x1f;

            HWREG(NVIC_PEND_BASE + reg) |= 1 << field;
    }
}

static const uint32_t NVIC_UNPEND_BASE = 0xe000e280;

// Unpend an interrupt
void IntPendClear(uint32_t interrupt)
{
    if (interrupt >= VECTOR_COUNT) return;

    switch (interrupt) {
        case 10:    // PendSV Fault
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_UNPEND_SV;
            break;
        case 14:    // SysTick Fault
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PENDSTCLR;
            break;
        default:
            if (interrupt < 16) return;
            // Vector Numbers in NVIC are offset 16 from the actual vector table
            interrupt -= 16;
            // en# register |= mask
            uint32_t reg = (interrupt >> 5) << 2;
            uint32_t field = interrupt & 0x1f;

            HWREG(NVIC_UNPEND_BASE + reg) |= 1 << field;
    }
}

// Set priority mask
void IntPriorityMaskSet(uint32_t priority_mask)
{
    __asm__(
        "   msr     BASEPRI, r0\n"
        "   bx      lr\n"
    );
}

// Get priority mask
uint32_t IntPriorityMaskGet(void)
{
    __asm__(
        "   mrs     r0, BASEPRI\n"
        "   bx      lr\n"
    );
    // Unreachable; for the compiler
    return 0;
}

// Trigger an interrupt
void IntTrigger(uint32_t interrupt)
{
    // Ensure valid interrupt
    if (interrupt < 16 || interrupt >= VECTOR_COUNT) return;
    // Trigger the interrupt
    NVIC_SW_TRIG_R = interrupt;
}
