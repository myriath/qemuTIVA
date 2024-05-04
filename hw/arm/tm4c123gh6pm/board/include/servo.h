#ifndef TM4_SERVO_H_
#define TM4_SERVO_H_

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "hw/qdev-clock.h"
#include "hw/clock.h"
#include "qapi/error.h"

struct ServoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem; 
    uint64_t pulse_width;

    uint64_t start_ns;
    uint64_t pulse_end_ns;
    uint64_t tick;

    Clock *clk;
    QEMUTimer *timer;

    bool started;

    qemu_irq duty_signal;

    bool debug;
};

#define TYPE_SERVO "tm4-servo"

typedef struct ServoState ServoState;
DECLARE_INSTANCE_CHECKER(ServoState, TM4_SERVO, TYPE_SERVO)

DeviceState *servo_create(bool debug, hwaddr addr, Clock *clk);

#endif
