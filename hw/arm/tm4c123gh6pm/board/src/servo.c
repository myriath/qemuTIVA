#include "hw/arm/tm4c123gh6pm/board/include/servo.h"

DeviceState *servo_create(bool debug, hwaddr addr, Clock *clk)
{
    DeviceState *dev = qdev_new(TYPE_SERVO);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
     
    qdev_prop_set_bit(dev, "debug", debug);
    qdev_connect_clock_in(dev, "clk", clk);
    sysbus_realize_and_unref(sbd, &error_fatal);

    sysbus_mmio_map(sbd, 0, addr);

    return dev;
}

static void reload(ServoState *s, bool reset)
{
    uint64_t tick;
    if (reset) {
        tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    } else {
        tick = s->tick;
    }

    s->start_ns = tick;
    s->pulse_end_ns = tick;
    tick += s->pulse_width;
    // printf("Servo waiting %ld ns\n", s->pulse_width);
    s->tick = tick;

    timer_mod(s->timer, tick);
}

static void tick(void *opaque)
{
    ServoState *s = opaque;

    int duty = (int)(
        ((double)(s->pulse_end_ns - s->start_ns) / (double)(s->tick - s->start_ns)) 
        * 100
    );
    qemu_set_irq(s->duty_signal, duty);

    if (s->debug) {
        printf("[SERVO DUTY] %d%%\n", duty);
    }
    
    reload(s, false);
}

static void handle_signal(void *opaque, int irq, int level)
{
    ServoState *s = opaque;

    if (!s->started && level) {
        reload(s, true);
        s->started = true;
    }
    // Only count the first time level goes 0
    if (level == 0 && s->pulse_end_ns == s->start_ns) {
        s->pulse_end_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static const VMStateDescription vmstate_servo =
{
    .name = TYPE_SERVO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(pulse_width, ServoState),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t servo_read(void *opaque, hwaddr offset, unsigned size)
{
    ServoState *s = opaque;

    switch (offset) {
        case 0x000:
            return s->pulse_width;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "servo_read: Bad offset %x\n", (int)offset);
            return 0;
    }
}

static void servo_write(void *opaque, hwaddr offset, 
                         uint64_t value, unsigned size)
{
    ServoState *s = opaque;

    switch (offset) {
        case 0x000:
            s->pulse_width = value;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "servo_write: Bad offset %x\n", (int)offset);
    }
}

static const MemoryRegionOps servo_ops = 
{
    .read = servo_read,
    .write = servo_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void servo_init(Object *obj)
{
    ServoState *s = TM4_SERVO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->pulse_width = 20000000;

    memory_region_init_io(&s->iomem, obj, &servo_ops, s, TYPE_SERVO, 0x1000);

    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_out(dev, &s->duty_signal, 1);
    qdev_init_gpio_in(dev, handle_signal, 1);

    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
}

static void servo_realize(DeviceState *dev, Error **errp)
{
    ServoState *s = TM4_SERVO(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "tm4-servo: clk must be connected");
        return;
    }

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tick, s);
}

static Property servo_properties[] = 
{
    DEFINE_PROP_BOOL("debug", ServoState, debug, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void servo_reset(DeviceState *dev)
{
    ServoState *s = TM4_SERVO(dev);

    s->pulse_width = 20000000;
}

static void servo_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_servo;
    dc->realize = servo_realize;
    dc->reset = servo_reset;

    device_class_set_props(dc, servo_properties);
}

static const TypeInfo servo_info =
{
    .name = TYPE_SERVO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ServoState),
    .instance_init = servo_init,
    .class_init = servo_class_init
};

static void servo_register_types(void)
{
    type_register_static(&servo_info);
}
type_init(servo_register_types)
