#ifndef HW_JOIN_IRQ_H
#define HW_JOIN_IRQ_H

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_JOIN_IRQ "join-irq"

#define MAX_SPLIT_LINES 16


struct JoinIRQ {
    DeviceState parent_obj;

    qemu_irq out_irq;
    uint16_t num_lines;
};

OBJECT_DECLARE_SIMPLE_TYPE(JoinIRQ, JOIN_IRQ)

#endif
