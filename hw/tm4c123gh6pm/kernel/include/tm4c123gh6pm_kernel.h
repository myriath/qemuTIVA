#ifndef TM4C123GH6PM_KERNEL_H_
#define TM4C123GH6PM_KERNEL_H_

#include <stdint.h>

#include "tm4c123gh6pm.h"

// Register an interrupt. Returns 0 if the interrupt number is reserved.
int IntRegister(uint32_t interrupt, void (*handler)(void));
// Return an interrupt to the default handler. Returns 0 if the interrupt number is reserved.
int IntUnregister(uint32_t interrupt);
// Cleanly exit the QEMU emulator
void ExitQEMU(void);

#endif
