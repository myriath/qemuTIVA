#ifndef TM4_GPIO_H_
#define TM4_GPIO_H_

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define N_BITS 8

struct GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t data;
    uint32_t dir;
    uint32_t is;
    uint32_t ibe;
    uint32_t iev;
    uint32_t im;
    uint32_t ris;
    uint32_t afsel;
    uint32_t dr2r;
    uint32_t dr4r;
    uint32_t dr8r;
    uint32_t odr;
    uint32_t pur;
    uint32_t pdr;
    uint32_t slr;
    uint32_t den;
    uint32_t lock;
    uint32_t cr;
    uint32_t amsel;
    uint32_t pctl;
    uint32_t adcctl;
    uint32_t dmactl;
    const unsigned char *id;

    uint32_t old_out_data;
    uint32_t old_in_data;

    qemu_irq nvic_irq;
    qemu_irq out[N_BITS];
};

#define TYPE_TM4_GPIO "tm4-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(GPIOState, TM4_GPIO)

#endif
