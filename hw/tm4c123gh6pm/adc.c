#include "adc.h"

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/split-irq.h"
#include "hw/sysbus.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "hw/arm/boot.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "net/net.h"
#include "hw/boards.h"
#include "qemu/log.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/arm/armv7m.h"
#include "hw/char/pl011.h"
#include "hw/input/stellaris_gamepad.h"
#include "hw/irq.h"
#include "hw/watchdog/cmsdk-apb-watchdog.h"
#include "migration/vmstate.h"
#include "hw/misc/unimp.h"
#include "hw/timer/stellaris-gptm.h"
#include "hw/qdev-clock.h"
#include "qom/object.h"
#include "qapi/qmp/qlist.h"
#include "ui/input.h"

/* Analogue to Digital Converter.  This is only partially implemented,
   enough for applications that use a combined ADC and timer tick.  */

#define TM4_ADC_EM_CONTROLLER 0
#define TM4_ADC_EM_COMP       1
#define TM4_ADC_EM_EXTERNAL   4
#define TM4_ADC_EM_TIMER      5
#define TM4_ADC_EM_PWM0       6
#define TM4_ADC_EM_PWM1       7
#define TM4_ADC_EM_PWM2       8

#define TM4_ADC_FIFO_EMPTY    0x0100
#define TM4_ADC_FIFO_FULL     0x1000

#define TYPE_TM4_ADC "tm4-adc"
typedef struct TM4ADCState TM4ADCState;
DECLARE_INSTANCE_CHECKER(TM4ADCState, TM4_ADC, TYPE_TM4_ADC)

struct TM4ADCState {
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
    uint32_t ssop[4];   // TODO Here and below
    uint32_t ssdc[4];   //
    uint32_t dcric;     //
    uint32_t dcctl[8];  //
    uint32_t dccmp[8];  //
    uint32_t pc;        //
    uint32_t cc;        // End TODO
    uint32_t noise;
    qemu_irq irq[4];
};

static uint32_t tm4c123gh6pm_adc_fifo_read(TM4ADCState *s, int fifo)
{
    int tail;

    tail = s->fifo[fifo].state & 0xf;
    if (s->fifo[fifo].state & TM4_ADC_FIFO_EMPTY) {
        s->ustat |= 1 << fifo;
    } else {
        s->fifo[fifo].state = (s->fifo[fifo].state & ~0xf) | ((tail + 1) & 0xf);
        s->fifo[fifo].state &= ~TM4_ADC_FIFO_FULL;
        if (tail + 1 == ((s->fifo[fifo].state >> 4) & 0xf))
            s->fifo[fifo].state |= TM4_ADC_FIFO_EMPTY;
    }
    return s->fifo[fifo].data[tail];
}

static void tm4c123gh6pm_adc_fifo_write(TM4ADCState *s, int fifo,
                                     uint32_t value)
{
    int head;

    /* TODO: Real hardware has limited size FIFOs.  We have a full 16 entry 
       FIFO fir each sequencer.  */
    head = (s->fifo[fifo].state >> 4) & 0xf;
    if (s->fifo[fifo].state & TM4_ADC_FIFO_FULL) {
        s->ostat |= 1 << fifo;
        return;
    }
    s->fifo[fifo].data[head] = value;
    head = (head + 1) & 0xf;
    s->fifo[fifo].state &= ~TM4_ADC_FIFO_EMPTY;
    s->fifo[fifo].state = (s->fifo[fifo].state & ~0xf0) | (head << 4);
    if ((s->fifo[fifo].state & 0xf) == head)
        s->fifo[fifo].state |= TM4_ADC_FIFO_FULL;
}

static void tm4c123gh6pm_adc_update(TM4ADCState *s)
{
    int level;
    int n;

    for (n = 0; n < 4; n++) {
        level = (s->ris & s->im & (1 << n)) != 0;
        qemu_set_irq(s->irq[n], level);
    }
}

static void tm4c123gh6pm_adc_trigger(void *opaque, int irq, int level)
{
    TM4ADCState *s = opaque;
    int n;

    for (n = 0; n < 4; n++) {
        if ((s->actss & (1 << n)) == 0) {
            continue;
        }

        if (((s->emux >> (n * 4)) & 0xff) != 5) {
            continue;
        }

        /* Some applications use the ADC as a random number source, so introduce
           some variation into the signal.  */
        s->noise = s->noise * 314159 + 1;
        /* TODO actual inputs not implemented.  Return an arbitrary value.  */
        tm4c123gh6pm_adc_fifo_write(s, n, 0x200 + ((s->noise >> 16) & 7));
        s->ris |= (1 << n);
        tm4c123gh6pm_adc_update(s);
    }
}

static void tm4c123gh6pm_adc_reset_hold(Object *obj)
{
    TM4ADCState *s = TM4_ADC(obj);
    int n;

    s->sspri = 0x00003210;

    for (n = 0; n < 4; n++) {
        s->ssmux[n] = 0;
        s->ssctl[n] = 0;
        s->fifo[n].state = TM4_ADC_FIFO_EMPTY;
        s->ssop[n] = 0;
        s->ssdc[n] = 0;
    }
    for (n = 0; n < 8; n++) {
        s->dcctl[n] = 0;
        s->dccmp[n] = 0;
    }

    s->pc = 0x7;
    s->cc = 0;
}

static uint64_t tm4c123gh6pm_adc_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    TM4ADCState *s = opaque;

    /* TODO: Implement this.  */
    if (offset >= 0x40 && offset < 0xc0) {
        int n;
        n = (offset - 0x40) >> 5;
        switch (offset & 0x1f) {
        case 0x00: /* SSMUX */
            return s->ssmux[n];
        case 0x04: /* SSCTL */
            return s->ssctl[n];
        case 0x08: /* SSFIFO */
            return tm4c123gh6pm_adc_fifo_read(s, n);
        case 0x0c: /* SSFSTAT */
            return s->fifo[n].state;
        case 0x10: /* SSOP */
            return s->ssop[n];
        case 0x14: /* SSDC */
            return s->ssdc[n];
        default:
            break;
        }
    }
    if (offset >= 0xe00 && offset <= 0xe1c) {
        return s->dcctl[(offset-0xe00) >> 2];
    }
    if (offset >= 0xe40 && offset <= 0xe5c) {
        return s->dccmp[(offset-0xe40) >> 2];
    }
    switch (offset) {
    case 0x00: /* ACTSS */
        return s->actss;
    case 0x04: /* RIS */
        return s->ris;
    case 0x08: /* IM */
        return s->im;
    case 0x0c: /* ISC */
        return s->ris & s->im;
    case 0x10: /* OSTAT */
        return s->ostat;
    case 0x14: /* EMUX */
        return s->emux;
    case 0x18: /* USTAT */
        return s->ustat;
    case 0x1c:
        return s->tssel;
    case 0x20: /* SSPRI */
        return s->sspri;
    case 0x24: 
        return s->spc;
    case 0x28:
        return s->pssi;
    case 0x30: /* SAC */
        return s->sac;
    case 0x34:
        return s->dcisc;
    case 0x38:
        return s->ctl;
    case 0xfc0:
        return 0x00b020c7;
    case 0xfc4:
        return s->pc;
    case 0xfc8:
        return s->cc;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ADC: read at bad offset 0x%x\n", (int)offset);
        return 0;
    }
}

// TODO
static void tm4c123gh6pm_adc_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    TM4ADCState *s = opaque;

    /* TODO: Implement this.  */
    if (offset >= 0x40 && offset < 0xc0) {
        int n;
        n = (offset - 0x40) >> 5;
        switch (offset & 0x1f) {
        case 0x00: /* SSMUX */
            s->ssmux[n] = value;
            return;
        case 0x04: /* SSCTL */ // TODO
            if (value != 6) {
                qemu_log_mask(LOG_UNIMP,
                              "ADC: Unimplemented sequence %" PRIx64 "\n",
                              value);
            }
            s->ssctl[n] = value;
            return;
        case 0x10: /* SSOP */
            qemu_log_mask(LOG_UNIMP, "ADC: SSOP unimplemented\n");
            s->ssop[n] = value;
            return;
        case 0x14: /* SSDC */
            qemu_log_mask(LOG_UNIMP, "ADC: SSDC unimplemented\n");
            s->ssdc[n] = value;
            return;
        default:
            break;
        }
    }
    if (offset >= 0xe00 && offset <= 0xe1c) {
        qemu_log_mask(LOG_UNIMP, "ADC: dcctl unimplemented\n");
        s->dcctl[(offset-0xe00) >> 2] = value;
        return;
    }
    if (offset >= 0xe40 && offset <= 0xe5c) {
        qemu_log_mask(LOG_UNIMP, "ADC: dccmp unimplemented\n");
        s->dccmp[(offset-0xe40) >> 2] = value;
        return;
    }
    switch (offset) {
    case 0x00: /* ACTSS */
        s->actss = value & 0xf;
        break;
    case 0x08: /* IM */
        s->im = value;
        break;
    case 0x0c: /* ISC */
        s->ris &= ~value;
        break;
    case 0x10: /* OSTAT */
        s->ostat &= ~value;
        break;
    case 0x14: /* EMUX */
        s->emux = value;
        break;
    case 0x18: /* USTAT */
        s->ustat &= ~value;
        break;
    case 0x1c:
        s->tssel = value;
        break;
    case 0x20: /* SSPRI */
        s->sspri = value;
        break;
    case 0x24:
        s->spc = value;
        break;
    case 0x28: /* PSSI */ // TODO
        qemu_log_mask(LOG_UNIMP, "ADC: sample initiate unimplemented\n");
        break;
    case 0x30: /* SAC */
        s->sac = value;
        break;
    case 0x34: // TODO
        qemu_log_mask(LOG_UNIMP, "ADC: dcisc unimplemented\n");
        s->dcisc = value;
        break;
    case 0x38: // TODO
        qemu_log_mask(LOG_UNIMP, "ADC: ctl unimplemented\n");
        s->ctl = value;
        break;
    case 0xd00:
        qemu_log_mask(LOG_UNIMP, "ADC: dcric unimplemented\n");
        s->dcric = value;
        break;
    case 0xfc4: // TODO
        qemu_log_mask(LOG_UNIMP, "ADC: pc unimplemented\n");
        s->pc = value;
        break;
    case 0xfc8: // TODO
        qemu_log_mask(LOG_UNIMP, "ADC: cc unimplemented\n");
        s->cc = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ADC: write at bad offset 0x%x\n", (int)offset);
    }
    tm4c123gh6pm_adc_update(s);
}

static const MemoryRegionOps tm4c123gh6pm_adc_ops = {
    .read = tm4c123gh6pm_adc_read,
    .write = tm4c123gh6pm_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_tm4c123gh6pm_adc = {
    .name = "tm4c123gh6pm_adc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(actss, TM4ADCState),
        VMSTATE_UINT32(ris, TM4ADCState),
        VMSTATE_UINT32(im, TM4ADCState),
        VMSTATE_UINT32(emux, TM4ADCState),
        VMSTATE_UINT32(ostat, TM4ADCState),
        VMSTATE_UINT32(ustat, TM4ADCState),
        VMSTATE_UINT32(sspri, TM4ADCState),
        VMSTATE_UINT32(spc, TM4ADCState),
        VMSTATE_UINT32(pssi, TM4ADCState),
        VMSTATE_UINT32(sac, TM4ADCState),
        VMSTATE_UINT32(dcisc, TM4ADCState),
        VMSTATE_UINT32(ctl, TM4ADCState),
        VMSTATE_UINT32(fifo[0].state, TM4ADCState),
        VMSTATE_UINT32_ARRAY(fifo[0].data, TM4ADCState, 16),
        VMSTATE_UINT32(ssmux[0], TM4ADCState),
        VMSTATE_UINT32(ssctl[0], TM4ADCState),
        VMSTATE_UINT32(ssop[0], TM4ADCState),
        VMSTATE_UINT32(ssdc[0], TM4ADCState),
        VMSTATE_UINT32(fifo[1].state, TM4ADCState),
        VMSTATE_UINT32_ARRAY(fifo[1].data, TM4ADCState, 16),
        VMSTATE_UINT32(ssmux[1], TM4ADCState),
        VMSTATE_UINT32(ssctl[1], TM4ADCState),
        VMSTATE_UINT32(ssop[1], TM4ADCState),
        VMSTATE_UINT32(ssdc[1], TM4ADCState),
        VMSTATE_UINT32(fifo[2].state, TM4ADCState),
        VMSTATE_UINT32_ARRAY(fifo[2].data, TM4ADCState, 16),
        VMSTATE_UINT32(ssmux[2], TM4ADCState),
        VMSTATE_UINT32(ssctl[2], TM4ADCState),
        VMSTATE_UINT32(ssop[2], TM4ADCState),
        VMSTATE_UINT32(ssdc[2], TM4ADCState),
        VMSTATE_UINT32(fifo[3].state, TM4ADCState),
        VMSTATE_UINT32_ARRAY(fifo[3].data, TM4ADCState, 16),
        VMSTATE_UINT32(ssmux[3], TM4ADCState),
        VMSTATE_UINT32(ssctl[3], TM4ADCState),
        VMSTATE_UINT32(ssop[3], TM4ADCState),
        VMSTATE_UINT32(ssdc[3], TM4ADCState),
        VMSTATE_UINT32(dcctl[0], TM4ADCState),
        VMSTATE_UINT32(dcctl[1], TM4ADCState),
        VMSTATE_UINT32(dcctl[2], TM4ADCState),
        VMSTATE_UINT32(dcctl[3], TM4ADCState),
        VMSTATE_UINT32(dcctl[4], TM4ADCState),
        VMSTATE_UINT32(dcctl[5], TM4ADCState),
        VMSTATE_UINT32(dcctl[6], TM4ADCState),
        VMSTATE_UINT32(dcctl[7], TM4ADCState),
        VMSTATE_UINT32(dccmp[0], TM4ADCState),
        VMSTATE_UINT32(dccmp[1], TM4ADCState),
        VMSTATE_UINT32(dccmp[2], TM4ADCState),
        VMSTATE_UINT32(dccmp[3], TM4ADCState),
        VMSTATE_UINT32(dccmp[4], TM4ADCState),
        VMSTATE_UINT32(dccmp[5], TM4ADCState),
        VMSTATE_UINT32(dccmp[6], TM4ADCState),
        VMSTATE_UINT32(dccmp[7], TM4ADCState),
        VMSTATE_UINT32(pc, TM4ADCState),
        VMSTATE_UINT32(cc, TM4ADCState),
        VMSTATE_UINT32(noise, TM4ADCState),
        VMSTATE_END_OF_LIST()
    }
};

static void tm4c123gh6pm_adc_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    TM4ADCState *s = TM4_ADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    for (n = 0; n < 4; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
    }

    memory_region_init_io(&s->iomem, obj, &tm4c123gh6pm_adc_ops, s,
                          "adc", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_in(dev, tm4c123gh6pm_adc_trigger, 1);
}


static void tm4c123gh6pm_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = tm4c123gh6pm_adc_reset_hold;
    dc->vmsd = &vmstate_tm4c123gh6pm_adc;
}

static const TypeInfo tm4c123gh6pm_adc_info = {
    .name          = TYPE_TM4_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TM4ADCState),
    .instance_init = tm4c123gh6pm_adc_init,
    .class_init    = tm4c123gh6pm_adc_class_init,
};

// attribute used to prevent compiler warnings, static function
// used in tm4c123gh6pm.c, not here
__attribute__((used)) static void adc_register_types(void)
{
    type_register_static(&tm4c123gh6pm_adc_info);
}
