#ifndef TM4C123GH6PM_KERNEL_H_
#define TM4C123GH6PM_KERNEL_H_

#include <stdint.h>
#include <stdbool.h>

#include "tm4c123gh6pm.h"

#define NVIC_MIN_INTERRUPT_VECTOR 16

/**
 * Enables interrupts for the processor via the CSPIE i instruction
 * 
 * \return True if the interrupts were previously enabled before the call
 */
bool IntMasterEnable();
/**
 * Disables interrupts for the processor via the CSPID i instruction
 * 
 * \return True if the interrupts were previously disabled before the call
 */
bool IntMasterEnable();

/**
 * Registers the given interrupt with the given handler.
 * 
 * \param interrupt Interrupt number of the interrupt in question.
 * \param handler void-returning function pointer to the new handler for the interrupt
 * 
 * \return None
 */
void IntRegister(uint32_t interrupt, void (*handler)(void));
/**
 * Unregisters the given interrupt, returning it to the default handler.
 * 
 * \param interrupt Interrupt number of the interrupt in question.
 * 
 * \return None.
 */
void IntUnregister(uint32_t interrupt);

/**
 * Sets the priority grouping of the interrupt controller.
 * 
 * \param bits specifies the number of bits of preemptable priority.
 * 
 * \return None.
 */
void IntPriorityGroupingSet(uint32_t bits);
/**
 * Gets the priority grouping for the interrupt controller.
 * 
 * \return Number of bits of preemptable priority.
 */
uint32_t IntPriorityGroupingGet(void);
/**
 * Sets the priority of an interrupt.
 * 
 * \param interrupt Interrupt number of the interrupt in question
 * \param priority Specifies the priority of the interrupt. 0-7; 0 is highest
 * 
 * \return None.
 */
void IntPrioritySet(uint32_t interrupt, uint8_t priority);
/**
 * Gets the priority of an interrupt.
 * 
 * \param interrupt Interrupt number of the interrupt in question
 * 
 * \return Priority of the interrupt. 0-7; 0 is highest
 */
uint32_t IntPriorityGet(uint32_t interrupt);

/**
 * Enables an interrupt.
 * 
 * \param interrupt Interrupt number of the interrupt to enable
 * 
 * \return None.
 */
void IntEnable(uint32_t interrupt);
/**
 * Disables an interrupt.
 * 
 * \param interrupt Interrupt number of the interrupt to disable
 * 
 * \return None.
 */
void IntDisable(uint32_t interrupt);
/**
 * Checks if the given interrupt is enabled.
 * 
 * \param interrupt Interrupt number of the interrupt to check
 * 
 * \return Non-zero value if the interrupt is enabled.
 */
uint32_t IntIsEnabled(uint32_t interrupt);

/**
 * Pends an interrupt.
 * 
 * \param interrupt Interrupt number of the interrupt to pend.
 * 
 * \return None.
 */
void IntPendSet(uint32_t interrupt);
/**
 * Un-pends an interrupt.
 * 
 * \param interrupt Interrupt number of the interrupt to un-pend.
 * 
 * \return None.
 */
void IntPendClear(uint32_t interrupt);

/**
 * Sets the priority masking level
 * 
 * \param priority_mask Priority level that is masked.
 * 
 * \return None.
 */
void IntPriorityMaskSet(uint32_t priority_mask);
/**
 * Gets the priority masking level
 * 
 * \return Value of the interrupt priority level mask.
 */
uint32_t IntPriorityMaskGet(void);

/**
 * Manually triggers the given interrupt
 * 
 * \param interrupt Interrupt number of the interrupt to trigger.
 * 
 * \return None.
 */
void IntTrigger(uint32_t interrupt);

/**
 * Cleanly exit the QEMU process.
 * Normally, hangs or exits will just throw QEMU into an infinite loop, which makes grading difficult.
 * 
 * \return None.
 */
void ExitQEMU(void);

#endif
