#ifndef TM4_ADC_H_
#define TM4_ADC_H_

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/qdev-clock.h"

#define TM4_ADC_EM_CONTROLLER 0
#define TM4_ADC_EM_COMP       1
#define TM4_ADC_EM_EXTERNAL   4
#define TM4_ADC_EM_TIMER      5
#define TM4_ADC_EM_PWM0       6
#define TM4_ADC_EM_PWM1       7
#define TM4_ADC_EM_PWM2       8

#define TM4_ADC_FIFO_EMPTY    0x0100
#define TM4_ADC_FIFO_FULL     0x1000

#define COUNT_ADC 2
#define COUNT_AIN 12
#define COUNT_SS 4

// Define device registers
struct ADCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t actss;
    uint32_t ris;
    uint32_t im;
    uint32_t emux;
    uint32_t ostat;
    uint32_t ustat;
    uint32_t tssel;
    uint32_t sspri;
    uint32_t spc;
    uint32_t pssi;
    uint32_t sac;
    uint32_t dcisc;
    uint32_t ctl;
    struct {
        uint32_t state;
        uint32_t data[16];
    } fifo[4];
    uint32_t ssmux[4];
    uint32_t ssctl[4];
    uint32_t ssop[4];
    uint32_t ssdc[4];
    uint32_t dcric;
    uint32_t dcctl[8];
    uint32_t dccmp[8];
    uint32_t pc;
    uint32_t cc;

    qemu_irq nvic_irq[4];
    qemu_irq ain_irq[12];
    // Each ain value is the voltage of the AIN in milli-volts
    uint32_t ain[12];

    uint8_t adc;
    bool debug;

    Clock *clk;
    bool clock_active;
};

#define TYPE_TM4_ADC "tm4-adc"

typedef struct ADCState ADCState;
DECLARE_INSTANCE_CHECKER(ADCState, TM4_ADC, TYPE_TM4_ADC)

DeviceState *adc_create(bool debug, hwaddr addr, qemu_irq *nvic_irq, uint8_t adc, Clock *clk);

#endif
