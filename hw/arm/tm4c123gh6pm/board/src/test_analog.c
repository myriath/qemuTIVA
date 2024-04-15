#include "hw/arm/tm4c123gh6pm/board/include/test_analog.h"

static const VMStateDescription vmstate_analog =
{
    .name = TYPE_TEST_ANALOG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(voltage_in, AnalogState),
        VMSTATE_UINT32(ain, AnalogState),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t analog_read(void *opaque, hwaddr offset, unsigned size)
{
    AnalogState *s = opaque;

    switch (offset) {
        case 0x000:
            return s->voltage_in;
        case 0x004:
            return s->ain;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "analog_read: Bad offset %x\n", (int)offset);
            return 0;
    }
}

static void analog_write(void *opaque, hwaddr offset, 
                         uint64_t value, unsigned size)
{
    AnalogState *s = opaque;

    switch (offset) {
        case 0x000:
            s->voltage_in = value;
            qemu_set_irq(s->out[s->ain], s->voltage_in);
            break;
        case 0x004:
            s->ain = value;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "analog_write: Bad offset %x\n", (int)offset);
    }
}

static const MemoryRegionOps analog_ops = 
{
    .read = analog_read,
    .write = analog_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void analog_init(Object *obj)
{
    AnalogState *s = TEST_ANALOG(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->voltage_in = 0;

    memory_region_init_io(&s->iomem, obj, &analog_ops, s, TYPE_TEST_ANALOG, 0x1000);

    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_out(dev, s->out, 12);
}

static void analog_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_analog;
}

static const TypeInfo analog_info =
{
    .name = TYPE_TEST_ANALOG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AnalogState),
    .instance_init = analog_init,
    .class_init = analog_class_init
};

static void analog_register_types(void)
{
    type_register_static(&analog_info);
}
type_init(analog_register_types)
