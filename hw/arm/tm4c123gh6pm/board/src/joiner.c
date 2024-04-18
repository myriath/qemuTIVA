#include "hw/arm/tm4c123gh6pm/board/include/joiner.h"

static void join_irq_handler(void *opaque, int n, int level)
{
    JoinIRQ *s = JOIN_IRQ(opaque);
    qemu_set_irq(s->out_irq, level);
}

static void join_irq_init(Object *obj)
{
    JoinIRQ *s = JOIN_IRQ(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->out_irq, 1);
}

static void join_irq_realize(DeviceState *dev, Error **errp)
{
    JoinIRQ *s = JOIN_IRQ(dev);

    if (s->num_lines < 1 || s->num_lines >= MAX_SPLIT_LINES) {
        error_setg(errp,
                   "IRQ splitter number of lines %d is not between 1 and %d",
                   s->num_lines, MAX_SPLIT_LINES);
        return;
    }

    qdev_init_gpio_in(dev, join_irq_handler, s->num_lines);
}

static Property join_irq_properties[] = {
    DEFINE_PROP_UINT16("num-lines", JoinIRQ, num_lines, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void join_irq_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    /* No state to reset or migrate */
    device_class_set_props(dc, join_irq_properties);
    dc->realize = join_irq_realize;

    /* Reason: Needs to be wired up to work */
    dc->user_creatable = false;
}

static const TypeInfo join_irq_type_info = {
   .name = TYPE_JOIN_IRQ,
   .parent = TYPE_DEVICE,
   .instance_size = sizeof(JoinIRQ),
   .instance_init = join_irq_init,
   .class_init = join_irq_class_init,
};

static void join_irq_register_types(void)
{
    type_register_static(&join_irq_type_info);
}

type_init(join_irq_register_types)
