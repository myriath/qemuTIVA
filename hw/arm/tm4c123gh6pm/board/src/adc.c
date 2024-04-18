#include "hw/arm/tm4c123gh6pm/board/include/adc.h"

DeviceState *adc_create(hwaddr addr, qemu_irq *nvic_irq, uint8_t adc)
{
    // Also the internals of sysbus_create_varargs because we need the adc prop set
    DeviceState *dev = qdev_new(TYPE_TM4_ADC);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
     
    qdev_prop_set_uint8(dev, "adc", adc);
    sysbus_realize_and_unref(sbd, &error_fatal);

    sysbus_mmio_map(sbd, 0, addr);
    for (int j = 0; j < COUNT_SS; j++) {
        sysbus_connect_irq(sbd, j, nvic_irq[j]);
    }

    return dev;
}

// Read a value from the SSFIFO, also updates underflow / tail / head pointers
static uint32_t adc_fifo_read(ADCState *s, int ss)
{
    int tail;

    tail = s->fifo[ss].state & 0xf;
    if (s->fifo[ss].state & TM4_ADC_FIFO_EMPTY) {
        s->ustat |= 1 << ss;
    } else {
        s->fifo[ss].state = (s->fifo[ss].state & ~0xf) | ((tail + 1) & 0xf);
        s->fifo[ss].state &= ~TM4_ADC_FIFO_FULL;
        if (tail + 1 == ((s->fifo[ss].state >> 4) & 0xf))
            s->fifo[ss].state |= TM4_ADC_FIFO_EMPTY;
    }
    return s->fifo[ss].data[tail];
}

// Write a value to the SSFIFO, also updates overflow / tail / head pointers
static void adc_fifo_write(ADCState *s, int ss, uint32_t value)
{
    int head;

    /* TODO: Real hardware has limited size FIFOs.  We have a full 16 entry 
       FIFO fir each sequencer.  */
    head = (s->fifo[ss].state >> 4) & 0xf;
    if (s->fifo[ss].state & TM4_ADC_FIFO_FULL) {
        s->ostat |= 1 << ss;
        return;
    }
    s->fifo[ss].data[head] = value;
    head = (head + 1) & 0xf;
    s->fifo[ss].state &= ~TM4_ADC_FIFO_EMPTY;
    s->fifo[ss].state = (s->fifo[ss].state & ~0xf0) | (head << 4);
    if ((s->fifo[ss].state & 0xf) == head)
        s->fifo[ss].state |= TM4_ADC_FIFO_FULL;
}

// Update interrupts based on ris and im
static void adc_update_nvic(ADCState *s)
{
    int level;
    int n;

    for (n = 0; n < 4; n++) {
        level = (s->ris & s->im & (1 << n)) != 0;
        qemu_set_irq(s->nvic_irq[n], level);
    }
}

// AIN triggers store the 'level' value as the AIN input voltage in milli-volts.
static void adc_ain_trigger(void *opaque, int irq, int level) 
{
    ADCState *s = opaque;
    s->ain[irq] = level;
}

// TODO: Any other triggers we desire (other events from SSEMUX)

// Runs when the PSSI value is updated. Handles the PSSI trigger
static void adc_pssi_trigger(ADCState *s)
{
    // Check the sync status
    if (s->pssi & 0x08000000 && ~s->pssi & 0x80000000) {
        return;
    }
    s->actss |= 0x00010000; // Set busy bit
    s->pssi &= ~0x80000000; // Clear GSYNC bit

    int sample_counts[4] = {8, 4, 4, 1}; // ss0 has 8 samples, ss1 has 4, etc
    int i, j, bit_i, nibble_j, ain_num;
    // For each sample sequence
    for (i = 0; i < 4; i++) {
        bit_i = 1 << i;
        // Check that the trigger for the sequence is set to 0 (default)
        if (((s->emux >> (i * 4)) & 0xff) != 0) {
            continue;
        }
        // Check sample sequence 'i' is triggered and the sequence is enabled
        if (~s->pssi & ~s->actss & bit_i) {
            continue;
        }
        // Read each sample of the ss
        for (j = 0; j < sample_counts[i]; j++) {
            nibble_j = j << 2;
            ain_num = (s->ssmux[i] & (0xf << nibble_j)) >> nibble_j;
            // Calculate the ADC value based on the input irq 'level' being the voltage in mV
            // printf("Writing AIN%d: %d\n", ain_num, (uint32_t)(s->ain[ain_num] / ((double)3300.0) * 4095) & 0xfff);
            adc_fifo_write(s, i, (uint32_t)(s->ain[ain_num] / ((double)3300.0) * 4095) & 0xfff);
            
            // Set interrupts if they are enabled
            if (s->ssctl[i] & (4 << nibble_j)) {
                s->ris |= bit_i;
            }
            // Check if this is the last sample
            if (s->ssctl[i] & (2 << nibble_j)) {
                break;
            }
        }
    }
    // Update the nvic with new interrupts
    adc_update_nvic(s);
    // Clear busy bit
    s->actss &= ~0x00010000;
}

// Resets the ADC
static void adc_reset_hold(Object *obj)
{
    ADCState *s = TM4_ADC(obj);
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

// Reads an offset from the ADC's iomem
static uint64_t adc_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    ADCState *s = opaque;

    if (offset >= 0x40 && offset < 0xc0) {
        int n;
        n = (offset - 0x40) >> 5;
        switch (offset & 0x1f) {
        case 0x00: /* SSMUX */
            return s->ssmux[n];
        case 0x04: /* SSCTL */
            return s->ssctl[n];
        case 0x08: /* SSFIFO */
            return adc_fifo_read(s, n);
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

#define BUF_SIZE 20

// Writes an offset to the ADC's iomem
static void adc_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    ADCState *s = opaque;

    uint32_t output = value;
    const char *reg = "BAD OFFSET";
    char buf[BUF_SIZE];

    if (offset >= 0x40 && offset < 0xc0) {
        int n;
        n = (offset - 0x40) >> 5;
        switch (offset & 0x1f) {
        case 0x00: /* SSMUX */
            reg = "SSMUX";
            s->ssmux[n] = value;
            break;
        case 0x04: /* SSCTL */
            reg = "SSCTL";
            s->ssctl[n] = value;
            break;
        case 0x10: /* SSOP */
            reg = "SSOP";
            qemu_log_mask(LOG_UNIMP, "ADC: SSOP unimplemented\n");
            s->ssop[n] = value;
            break;
        case 0x14: /* SSDC */
            reg = "SSDC";
            qemu_log_mask(LOG_UNIMP, "ADC: SSDC unimplemented\n");
            s->ssdc[n] = value;
            break;
        default:
            n = -1;
        }
        if (n != -1) {
            snprintf(buf, BUF_SIZE, "%s%d", reg, n);
            reg = buf;
        }
        goto print_ret;
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
        reg = "ACTSS";
        s->actss = value & 0xf;
        break;
    case 0x08: /* IM */
        reg = "IM";
        s->im = value;
        break;
    case 0x0c: /* ISC */
        reg = "ISC";
        s->ris &= ~value;
        break;
    case 0x10: /* OSTAT */
        reg = "OSTAT";
        s->ostat &= ~value;
        break;
    case 0x14: /* EMUX */
        reg = "EMUX";
        s->emux = value;
        break;
    case 0x18: /* USTAT */
        reg = "USTAT";
        s->ustat &= ~value;
        break;
    case 0x1c:
        reg = "TSSEL";
        s->tssel = value;
        break;
    case 0x20: /* SSPRI */
        reg = "SSPRI";
        s->sspri = value;
        break;
    case 0x24:
        reg = "SPC";
        s->spc = value;
        break;
    case 0x28: /* PSSI */
        reg = "PSSI";
        s->pssi = value;
        adc_pssi_trigger(s);
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
    adc_update_nvic(s);
    print_ret:
    printf("[ADC%d %s] 0x%.8X\n", s->adc, reg, output);
}

// ADC iomem read / write operations
static const MemoryRegionOps adc_ops = {
    .read = adc_read,
    .write = adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

// VMState for the ADC
static const VMStateDescription vmstate_adc = {
    .name = TYPE_TM4_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(actss, ADCState),
        VMSTATE_UINT32(ris, ADCState),
        VMSTATE_UINT32(im, ADCState),
        VMSTATE_UINT32(emux, ADCState),
        VMSTATE_UINT32(ostat, ADCState),
        VMSTATE_UINT32(ustat, ADCState),
        VMSTATE_UINT32(sspri, ADCState),
        VMSTATE_UINT32(spc, ADCState),
        VMSTATE_UINT32(pssi, ADCState),
        VMSTATE_UINT32(sac, ADCState),
        VMSTATE_UINT32(dcisc, ADCState),
        VMSTATE_UINT32(ctl, ADCState),
        VMSTATE_UINT32(fifo[0].state, ADCState),
        VMSTATE_UINT32_ARRAY(fifo[0].data, ADCState, 16),
        VMSTATE_UINT32(ssmux[0], ADCState),
        VMSTATE_UINT32(ssctl[0], ADCState),
        VMSTATE_UINT32(ssop[0], ADCState),
        VMSTATE_UINT32(ssdc[0], ADCState),
        VMSTATE_UINT32(fifo[1].state, ADCState),
        VMSTATE_UINT32_ARRAY(fifo[1].data, ADCState, 16),
        VMSTATE_UINT32(ssmux[1], ADCState),
        VMSTATE_UINT32(ssctl[1], ADCState),
        VMSTATE_UINT32(ssop[1], ADCState),
        VMSTATE_UINT32(ssdc[1], ADCState),
        VMSTATE_UINT32(fifo[2].state, ADCState),
        VMSTATE_UINT32_ARRAY(fifo[2].data, ADCState, 16),
        VMSTATE_UINT32(ssmux[2], ADCState),
        VMSTATE_UINT32(ssctl[2], ADCState),
        VMSTATE_UINT32(ssop[2], ADCState),
        VMSTATE_UINT32(ssdc[2], ADCState),
        VMSTATE_UINT32(fifo[3].state, ADCState),
        VMSTATE_UINT32_ARRAY(fifo[3].data, ADCState, 16),
        VMSTATE_UINT32(ssmux[3], ADCState),
        VMSTATE_UINT32(ssctl[3], ADCState),
        VMSTATE_UINT32(ssop[3], ADCState),
        VMSTATE_UINT32(ssdc[3], ADCState),
        VMSTATE_UINT32(dcctl[0], ADCState),
        VMSTATE_UINT32(dcctl[1], ADCState),
        VMSTATE_UINT32(dcctl[2], ADCState),
        VMSTATE_UINT32(dcctl[3], ADCState),
        VMSTATE_UINT32(dcctl[4], ADCState),
        VMSTATE_UINT32(dcctl[5], ADCState),
        VMSTATE_UINT32(dcctl[6], ADCState),
        VMSTATE_UINT32(dcctl[7], ADCState),
        VMSTATE_UINT32(dccmp[0], ADCState),
        VMSTATE_UINT32(dccmp[1], ADCState),
        VMSTATE_UINT32(dccmp[2], ADCState),
        VMSTATE_UINT32(dccmp[3], ADCState),
        VMSTATE_UINT32(dccmp[4], ADCState),
        VMSTATE_UINT32(dccmp[5], ADCState),
        VMSTATE_UINT32(dccmp[6], ADCState),
        VMSTATE_UINT32(dccmp[7], ADCState),
        VMSTATE_UINT32(pc, ADCState),
        VMSTATE_UINT32(cc, ADCState),
        VMSTATE_END_OF_LIST()
    }
};

// Initialize a new ADC device
static void adc_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    ADCState *s = TM4_ADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    // Initialize iomem
    memory_region_init_io(&s->iomem, obj, &adc_ops, s,
                          "adc", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    // Outputs for nvic
    for (n = 0; n < 4; n++) {
        sysbus_init_irq(sbd, &s->nvic_irq[n]);
    }

    qdev_init_gpio_in(dev, adc_ain_trigger, 12);
}

static Property adc_properties[] = 
{
    DEFINE_PROP_UINT8("adc", ADCState, adc, 0),
    DEFINE_PROP_END_OF_LIST()
};

// Initialize ADC class for QEMU
static void adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, adc_properties);

    rc->phases.hold = adc_reset_hold;
    dc->vmsd = &vmstate_adc;
}

// ADC device info
static const TypeInfo adc_info = {
    .name          = TYPE_TM4_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ADCState),
    .instance_init = adc_init,
    .class_init    = adc_class_init,
};

// Register the ADC info as a new QEMU type
static void adc_register_types(void)
{
    type_register_static(&adc_info);
}
type_init(adc_register_types)
