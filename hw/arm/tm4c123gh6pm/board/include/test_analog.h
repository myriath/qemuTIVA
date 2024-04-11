#ifndef TEST_ANALOG_IN_H_
#define TEST_ANALOG_IN_H_

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

struct AnalogState {
    SysBusDevice parent_obj;

    MemoryRegion iomem; 
    uint32_t voltage_in;
    uint32_t ain;

    qemu_irq out[12];
};

#define TYPE_TEST_ANALOG "tm4-test-analog"

typedef struct AnalogState AnalogState;
DECLARE_INSTANCE_CHECKER(AnalogState, TEST_ANALOG, TYPE_TEST_ANALOG)

#endif
