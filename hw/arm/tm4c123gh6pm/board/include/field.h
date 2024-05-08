#ifndef CYBOT_FIELD_H_
#define CYBOT_FIELD_H_

#include "packets.h"
#include "socket_handler.h"

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "hw/qdev-clock.h"
#include "hw/clock.h"


#define TYPE_CYBOT_FIELD "cybot-field"

struct FieldState {
    SysBusDevice parent_obj;

    uint32_t port;

    uint32_t last_servo;
    qemu_irq servo_in;

    QEMUTimer *timer;
    Clock *clk;
    uint64_t tick;
    uint64_t timeout;

    bool exit;
    bool reading;
    int read_count;
    int bytes_to_read;
    Packet packet;
};

OBJECT_DECLARE_SIMPLE_TYPE(FieldState, CYBOT_FIELD)

DeviceState *create_field_device(Clock *clk);

#endif
