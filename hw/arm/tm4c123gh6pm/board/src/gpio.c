#include "hw/arm/tm4c123gh6pm/board/include/gpio.h"

static const uint8_t pl061_id_luminary[12] =
  { 0x00, 0x00, 0x00, 0x00, 0x61, 0x00, 0x18, 0x01, 0x0d, 0xf0, 0x05, 0xb1 };

static const VMStateDescription vmstate_gpio = {
    .name = TYPE_TM4_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(data, GPIOState),
        VMSTATE_UINT32(dir, GPIOState),
        VMSTATE_UINT32(is, GPIOState),
        VMSTATE_UINT32(ibe, GPIOState),
        VMSTATE_UINT32(iev, GPIOState),
        VMSTATE_UINT32(im, GPIOState),
        VMSTATE_UINT32(ris, GPIOState),
        VMSTATE_UINT32(afsel, GPIOState),
        VMSTATE_UINT32(dr2r, GPIOState),
        VMSTATE_UINT32(dr4r, GPIOState),
        VMSTATE_UINT32(dr8r, GPIOState),
        VMSTATE_UINT32(odr, GPIOState),
        VMSTATE_UINT32(pur, GPIOState),
        VMSTATE_UINT32(pdr, GPIOState),
        VMSTATE_UINT32(slr, GPIOState),
        VMSTATE_UINT32(den, GPIOState),
        VMSTATE_UINT32(lock, GPIOState),
        VMSTATE_UINT32(cr, GPIOState),
        VMSTATE_UINT32(amsel, GPIOState),
        VMSTATE_UINT32(pctl, GPIOState),
        VMSTATE_UINT32(adcctl, GPIOState),
        VMSTATE_UINT32(dmactl, GPIOState),
        VMSTATE_END_OF_LIST()
    }
};

/**
 * Gets a mask of floating inputs, which are defined
 * as pins configured as an input and neither 1 or 0.
 */
static uint8_t gpio_floating(GPIOState *s)
{
    uint8_t floating = ~(s->pur | s->pdr);
    return floating & ~s->dir;
}

/**
 * Gets a mask of input pins pulled to high (1)
 */
static uint8_t gpio_pullups(GPIOState *s)
{
    uint8_t pullups = s->pur;
    return pullups & ~s->dir;
}

/**
 * Update the IRQs based on new state
 */
static void gpio_update(GPIOState *s)
{
    uint8_t delta, mask, out, iev_edges;
    int i;
    uint8_t pullups = gpio_pullups(s);
    uint8_t floating = gpio_floating(s);

    uint8_t data_out = s->data & s->dir;
    uint8_t floating_out = s->old_out_data & floating;

    /*
    Pins configured as outputs are driven from the data register;
    otherwise if pulled up they are 1, and if they're floating, we
    keep the same value as before.
    */
    out = data_out | pullups | floating_out;
    delta = s->old_out_data ^ out;
    if (delta) {
        s->old_out_data = out;
        for (i = 0; i < N_BITS; i++) {
            mask = 1 << i;
            if (delta & mask) {
                int level = (out & mask) != 0;
                qemu_set_irq(s->out[i], level);
            }
        }
    }

    // IEV selected edges, where both iev and data are the same
    // The check for if the bit has changed comes later
    iev_edges = ~(s->data ^ s->iev);
    /*
    Input delta calculated by xoring the data reg
    with old in status and masking with the input pins.
    */
    delta = (s->old_in_data ^ s->data) & ~s->dir;
    if (delta) {
        s->old_in_data = s->data;
        for (i = 0; i < N_BITS; i++) {
            mask = 1 << i;
            // Check bit i changed
            if (!(delta & mask)) {
                continue;
            }
            // Check bit i is sensing edge interrupts
            if (s->is & mask) {
                continue;
            }
            // Edge interrupts
            if (!(s->is & mask)) {
                // Any edge triggers interrupt
                if (s->ibe & mask) {
                    s->ris |= mask;
                }
                // Selected by IEV
                else {
                    s->ris |= iev_edges & mask;
                }
            }
        }
    }

    // Level interrupts 
    s->ris |= iev_edges & s->is;
    
    qemu_set_irq(s->nvic_irq, (s->ris & s->im) != 0);
}

static uint64_t gpio_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    GPIOState *s = opaque;
    
    switch (offset) {
    case 0x000 ... 0x3ff:
        return s->data & (offset >> 2);
    case 0x400:
        return s->dir;
    case 0x404:
        return s->is;
    case 0x408:
        return s->ibe;
    case 0x40c:
        return s->iev;
    case 0x410:
        return s->im;
    case 0x414:
        return s->ris;
    case 0x418:
        return s->ris & s->im;
    case 0x420:
        return s->afsel;
    case 0x500:
        return s->dr2r;
    case 0x504:
        return s->dr4r;
    case 0x508:
        return s->dr8r;
    case 0x50c:
        return s->odr;
    case 0x510:
        return s->pur;
    case 0x514:
        return s->pdr;
    case  0x518:
        return s->slr;
    case 0x51c:
        return s->den;
    case 0x520:
        return s->lock;
    case 0x524:
        return s->cr;
    case 0x528:
        return s->amsel;
    case 0x52c:
        return s->pctl;
    case 0x530:
        return s->adcctl;
    case 0x534:
        return s->dmactl;
    case 0xfd0 ... 0xfff:
        return s->id[(offset - 0xfd0) >> 2];
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gpio_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void gpio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    GPIOState *s = opaque;
    uint8_t mask;
    uint8_t value8 = value & 0xff;

    switch (offset) {
    case 0x000 ... 0x3ff:
        mask = (offset >> 2) & s->dir;
        s->data = (s->data & ~mask) | (value & mask);
        break;
    case 0x400:
        s->dir = value8;
        break;
    case 0x404:
        s->is = value8;
        break;
    case 0x408:
        s->ibe = value8;
        break;
    case 0x40c:
        s->iev = value8;
        break;
    case 0x410:
        s->im = value8;
        break;
    case 0x41c:
        s->ris &= ~value;
        break;
    case 0x420:
        mask = s->cr;
        s->afsel = (s->afsel & ~mask) | (value & mask);
        break;
    case 0x500:
        s->dr2r = value8;
        break;
    case 0x504:
        s->dr4r = value8;
        break;
    case 0x508:
        s->dr8r = value8;
        break;
    case 0x50c:
        s->odr = value8;
        break;
    case 0x510:
        mask = s->cr;
        s->pur = (s->pur & ~mask) | (value & mask);
        break;
    case 0x514:
        mask = s->cr;
        s->pdr = (s->pdr & ~mask) | (value & mask);
        break;
    case  0x518:
        s->slr = value8;
        break;
    case 0x51c:
        mask = s->cr;
        s->den = (s->den & ~mask) | (value & mask);
        break;
    case 0x520:
        // Locked unless you write 0x4c4f434b as per datasheet
        s->lock = (value != 0x4c4f434b);
        break;
    case 0x524:
        if (s->lock) break;
        s->cr = value8;
        break;
    case 0x528:
        s->amsel = value8;
        break;
    case 0x52c:
        s->pctl = value;
        break;
    case 0x530:
        s->adcctl = value8;
        break;
    case 0x534:
        s->dmactl = value8;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gpio_write: Bad offset %x\n", (int)offset);
    }

    gpio_update(s);
}

static void gpio_enter_reset(Object *obj, ResetType type)
{
    GPIOState *s = TM4_GPIO(obj);

    s->data = 0;
    s->dir = 0;
    s->is = 0;
    s->ibe = 0;
    s->iev = 0;
    s->im = 0;
    s->ris = 0;
    s->afsel = 0;
    s->dr2r = 0xff;
    s->dr4r = 0;
    s->dr8r = 0;
    s->odr = 0;
    s->pur = 0;
    s->pdr = 0;
    s->slr = 0;
    s->den = 0;
    s->lock = 1;
    s->cr = 0xff;
    s->amsel = 0;
    s->pctl = 0;
    s->adcctl = 0;
    s->dmactl = 0;
}

static void gpio_hold_reset(Object *obj)
{
    GPIOState *s = TM4_GPIO(obj);

    uint8_t mask;
    int i;
    uint8_t floating = gpio_floating(s);
    uint8_t pullups = gpio_pullups(s);

    for (i = 0; i < N_BITS; i++) {
        mask = 1 << i;
        if (floating & mask) {
            continue;
        }
        qemu_set_irq(s->out[i], pullups & mask);
    }
}

static void gpio_set_irq(void *opaque, int irq, int level)
{
    GPIOState *s = opaque;
    
    uint8_t mask = 1 << irq;
    // If pin #irq is an input
    if (!(s->dir & mask)) {
        // Clear existing data for the pin
        s->data &= ~mask;
        // If the irq is 1, set the data bit
        if (level) {
            s->data |= mask;
        }
        gpio_update(s);
    }
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void gpio_init(Object *obj)
{
    GPIOState *s = TM4_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->id = pl061_id_luminary;

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s, TYPE_TM4_GPIO, 0x1000);
    
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->nvic_irq);

    qdev_init_gpio_in(dev, gpio_set_irq, N_BITS);
    qdev_init_gpio_out(dev, s->out, N_BITS);
}

static void gpio_realize(DeviceState *dev, Error **errp)
{
    GPIOState *s = TM4_GPIO(dev);

    if (s->pur > 0xff) {
        error_setg(errp, "pullups property must be between 0 and 0xff");
        return;
    }
    if (s->pdr > 0xff) {
        error_setg(errp, "pulldowns property must be between 0 and 0xff");
        return;
    }
    if (s->pur & s->pdr) {
        error_setg(errp, "no bit may be set both in pullups and pulldowns");
        return;
    }
}

static Property gpio_props[] = 
{
    DEFINE_PROP_UINT32("pur", GPIOState, pur, 0xff),
    DEFINE_PROP_UINT32("pdr", GPIOState, pdr, 0x0),
    DEFINE_PROP_END_OF_LIST()
};

static void gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_gpio;
    dc->realize = gpio_realize;
    device_class_set_props(dc, gpio_props);

    rc->phases.enter = gpio_enter_reset;
    rc->phases.hold = gpio_hold_reset;
}

static const TypeInfo gpio_info =
{
    .name = TYPE_TM4_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GPIOState),
    .instance_init = gpio_init,
    .class_init = gpio_class_init
};

static void gpio_register_types(void)
{
    type_register_static(&gpio_info);
}
type_init(gpio_register_types)
