#include "hw/arm/tm4c123gh6pm/board/include/timer_16_32.h"

static uint8_t calc_len(GPTMState *s)
{
    uint8_t len = 0;
    if (s->width == GPTM_WIDTH_32_64) {
        len++;
    }
    if (s->cfg != GPTM_CFG_SHORT) {
        len++;
    }
    return len;
}

static uint64_t read_a_from_len(uint32_t *a_b_regs, uint64_t val64, uint8_t len)
{
    switch (len) {
        case GPTM_LEN_16:
            return a_b_regs[0];
        case GPTM_LEN_32:
            return a_b_regs[0] | (a_b_regs[1] << 16);
        case GPTM_LEN_64:
            return val64 & 0xffffffff;
    }
    return 0; // shouldn't be possible to reach
}

static uint64_t read_b_from_len(uint32_t *a_b_regs, uint64_t val64, uint8_t len)
{
    switch (len) {
        case GPTM_LEN_16:
        case GPTM_LEN_32:
            return a_b_regs[1];
        case GPTM_LEN_64:
            return (val64 >> 32) & 0xffffffff;
    }
    return 0; // shouldn't be possible to reach
}

static void gptm_update_irq(GPTMState *s)
{
    int level;
    level = (s->ris & s->imr) != 0;
    qemu_set_irq(s->nvic_irq, level);
}

static bool timer_matched(GPTMState *s, int timer, uint32_t time, bool use_p)
{
    uint32_t match = s->match[timer];
    uint32_t p_match = s->prescale_match[timer];
    if (use_p) {
        return time == (match & 0xffff) | ((p_match << 16) & 0xff);
    } else {
        return time == match;
    }
}

static bool timer_enabled(GPTMState *s, int timer)
{
    uint32_t ctl = s->ctl;
    return timer == GPTM_A ? ctl & GPTM_CTL_TAEN : ctl & GPTM_CTL_TBEN;
}

static bool get_direction(GPTMState *s, int timer)
{
    return s->mode[timer] & GPTM_MODE_CDIR;
}

static uint64_t get_raw_time(GPTMState *s, int timer)
{
    uint64_t start = s->start[timer];
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    return now - start;
}

static uint8_t calculate_prescale_true(GPTMState *s, int timer)
{
    int scalar = s->prescale[timer];
    return scalar - ((get_raw_time(s, timer) / s->ns_per_cycle) % (scalar + 1));
}

static uint32_t get_interval_load(GPTMState *s, int mode, int timer)
{
    switch (mode) {
        case GPTM_ONE_SHOT_CON:
        case GPTM_PERIODIC_CON:
        case GPTM_RTC:
            return s->load[timer];
        default:
            return (s->load[timer] & 0xffff) | ((s->prescale[timer] & 0xff) << 16);
    }
}

static uint32_t calc_time(uint64_t cycles, uint64_t load, bool direction)
{
    return direction
        ? cycles % load
        : load - (cycles % load);
}

static uint64_t get_time(GPTMState *s, int mode, int timer, bool v)
{
    bool direction = get_direction(s, timer);
    uint64_t raw_time = get_raw_time(s, timer);
    uint64_t cycles = raw_time / s->ns_per_cycle;
    uint64_t scaled_cycles = cycles / s->prescale[timer];
    uint32_t interval_load = get_interval_load(s, mode, timer);
    switch (mode) {
        case GPTM_ONE_SHOT_IND:
        case GPTM_PERIODIC_IND:
            return calc_time(
                direction ? cycles : scaled_cycles, 
                interval_load, 
                direction 
            );
        case GPTM_ONE_SHOT_CON:
        case GPTM_PERIODIC_CON:
            return calc_time(cycles, interval_load, direction);
        case GPTM_RTC:
            return interval_load + (raw_time / 1000000000);
        case GPTM_EDGE_COUNT:
            return s->count[timer];
        case GPTM_EDGE_TIME:
            return v 
                ? calc_time(cycles, interval_load, direction) 
                : s->capture[timer];
        case GPTM_PWM_OS:
        case GPTM_PWM_PR:
            return calc_time(cycles, interval_load, false);
        default:
            return 0;
    }
}

static void disable_timer(GPTMState *s, int mode, int timer, uint64_t time)
{
    s->ctl &= timer == GPTM_A ? ~GPTM_CTL_TAEN : ~GPTM_CTL_TBEN;
    s->time[timer] = time == -1 ? get_time(s, mode, timer, true) : time;
}

static uint32_t get_time_register(GPTMState *s, int mode, int timer, bool v)
{
    uint32_t time = get_time(s, mode, timer, v);
    switch (mode) {
        case GPTM_ONE_SHOT_CON:
        case GPTM_PERIODIC_CON:
        case GPTM_RTC:
            return time & 0xffffffff;
        case GPTM_ONE_SHOT_IND:
        case GPTM_PERIODIC_IND:
            return time & (v ? 0xffffff : 0xffff);
        case GPTM_EDGE_COUNT:
            return time & 0xffffff;
        case GPTM_EDGE_TIME:
        case GPTM_PWM_OS:
        case GPTM_PWM_PR:
            return time & 0xffff;
        default:
            return 0;
    }
}

static uint32_t get_prescale_register(GPTMState *s, int mode, int timer, bool v)
{
    uint32_t time = get_time(s, mode, timer, v);
    switch (mode) {
        case GPTM_ONE_SHOT_IND:
        case GPTM_PERIODIC_IND:
            if (get_direction(s, timer)) {
                return calculate_prescale_true(s, timer);
            }
        case GPTM_EDGE_COUNT:
        case GPTM_EDGE_TIME:
        case GPTM_PWM_OS:
        case GPTM_PWM_PR:
            return (time >> 16) & 0xff;
        case GPTM_ONE_SHOT_CON:
        case GPTM_PERIODIC_CON:
        case GPTM_RTC:
        default:
            return 0;
    }
}

static void count(GPTMState *s, int mode, int timer) 
{
    if (!timer_enabled(s, timer)) {
        return;
    }

    uint64_t reset_val;
    if (get_direction(s, timer)) {
        s->count[timer]++;
        reset_val = 0;
    } else {
        s->count[timer]--;
        reset_val = get_interval_load(s, mode, timer);
    }

    if (s->count[timer] == get_match(s, timer)) {
        s->count[timer] = reset_val;
        if (timer == GPTM_A) {
            s->ris |= GPTM_INT_CAM_M;
            s->ctl &= ~GPTM_CTL_TAEN;
        } else {
            s->ris |= GPTM_INT_CBM_M;
            s->ctl &= ~GPTM_CTL_TBEN;
        }
    } else {
        s->count[timer] &= 0xffffff;
    }
}

static void input_ccp(void *opaque, int irq, int level)
{
    GPTMState *s = opaque;
    if (!level || timer_enabled(s, irq)) {
        return;
    }

    int mode = get_mode(s, irq);
    switch (mode) {
        case GPTM_EDGE_COUNT:
            count(s, mode, irq);
            break;
        case GPTM_EDGE_TIME:
            s->capture[irq] = get_time(s, mode, irq, false);
            break;
        default:
            return;
    }
    s->ris |= irq == GPTM_A ? GPTM_INT_CAE_M : GPTM_INT_CBE_M;
}

static uint64_t matching_timer_reload(GPTMState *s, int mode, int timer, bool reset)
{
    uint64_t match = get_match(s, mode, timer);
    uint64_t load = get_interval_load(s, mode, timer);
    if (reset) {
        if (match > load) {
            return load * s->ns_per_cycle;
        }
        return (load - match) * s->ns_per_cycle;
    }

    bool matched = s->matched[timer];
    bool oneshot = false;
    if (mode == GPTM_PWM_OS || mode == GPTM_ONE_SHOT_IND || mode == GPTM_ONE_SHOT_CON) {
        oneshot = true;
    }

    if (match > load) {
        if (mode != GPTM_EDGE_TIME) {
            s->ris |= timer == GPTM_A ? GPTM_INT_TATO_M : GPTM_INT_TBTO_M;
        }
        if (oneshot) {
            disable_timer(s, mode, timer, get_direction(s, timer) ? load : 0);
            return -1;
        }
        return load * s->ns_per_cycle;
    }

    uint64_t ns;
    if (matched) {
        if (mode != GPTM_EDGE_TIME) {
            s->ris |= timer == GPTM_A ? GPTM_INT_TATO_M : GPTM_INT_TBTO_M;
        }
        if (oneshot) {
            disable_timer(s, mode, timer, get_direction(s, timer) ? load : 0);
            return -1;
        }
        ns = (load - match) * s->ns_per_cycle;
    } else {
        ns = match * s->ns_per_cycle;
        switch (mode) {
            case GPTM_ONE_SHOT_IND:
            case GPTM_ONE_SHOT_CON:
            case GPTM_PERIODIC_IND:
            case GPTM_PERIODIC_CON:
                s->ris |= timer == GPTM_A ? GPTM_INT_TAM_M : GPTM_INT_TBM_M;
            case GPTM_EDGE_TIME:
                s->ris |= timer == GPTM_A ? GPTM_INT_CAM_M : GPTM_INT_CBM_M;
        }
    }

    s->matched[timer] = !matched;
    return ns;
}

static void stop(GPTMState *s, int timer)
{
    timer_del(s->timer[timer]);
}

static void reload(GPTMState *s, int mode, int timer, bool reset)
{
    uint64_t tick;
    if (reset) {
        tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->start[timer] = tick;
        s->matched[timer] = false;
    } else {
        tick = s->tick[timer];
    }

    switch (mode) {
        case GPTM_PERIODIC_IND:
        case GPTM_PERIODIC_CON:
        case GPTM_ONE_SHOT_IND:
        case GPTM_ONE_SHOT_CON:
        case GPTM_EDGE_TIME:
            if (s->matched[timer]) {
                s->start[timer] = tick;
            }
            tick += matching_timer_reload(s, mode, timer, reset);
            break;
        case GPTM_RTC:
            tick += NANOSECONDS_PER_SECOND;
            // check for match
            if (get_match(s, mode, timer) == get_time(s, mode, timer, false)) {
                s->ris |= GPTM_INT_RTC_M;
            }
            break;
        case GPTM_PWM_OS:
        case GPTM_PWM_PR:
            if (s->matched[timer]) {
                s->start[timer] = tick;
            }
            tick += matching_timer_reload(s, mode, timer, reset);
            qemu_set_irq(timer, !s->matched[timer]);
            return;
        default:
            return;
    }

    s->tick[timer] = tick;
    timer_mod(s->timer[timer], tick);
}

static void timer_tick(void *opaque)
{
    // get pointer to this timer's opaque array slot
    GPTMState **ptr = (GPTMState **) opaque;
    // Deref pointer to get state
    GPTMState *s = *ptr;
    // s->opaque is the array, p points to either 0 or 1 of that array,
    // difference is the timer index
    int timer = ptr - s->opaque;

    int mode = get_mode(s, timer);
    uint64_t time = get_time(s, mode, timer, true);
    // TODO: timer_tick, starting timers, setting time when disabled, getting time when disabled

    reload(s, mode, timer, false);
}

static uint64_t gptm_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    GPTMState *s = opaque;

    uint8_t len = calc_len(s);
    int mode = get_mode(s);

    switch (offset) {
    case 0x000: /* CFG */
        return s->cfg;
    case 0x004: /* TAMR */
        return s->mode[0];
    case 0x008: /* TBMR */
        return s->mode[1];
    case 0x00c: /* CTL */
        return s->ctl;
    case 0x010:
        return s->sync;
    case 0x018: /* IMR */
        return s->imr;
    case 0x01c: /* RIS */
        return s->ris;
    case 0x020: /* MIS */
        return s->imr & s->ris;
    case 0x028: /* TAILR */
        return read_a_from_len(s->load, s->load_64, len);
    case 0x02c: /* TBILR */
        return read_b_from_len(s->load, s->load_64, len);
    case 0x030: /* TAMATCHR */
        return read_a_from_len(s->match, s->match_64, len);
    case 0x034: /* TBMATCHR */
        return read_b_from_len(s->match, s->match_64, len);
    case 0x038: /* TAPR */
        return s->prescale[0];
    case 0x03c: /* TBPR */
        return s->prescale[1];
    case 0x040: /* TAPMR */
        return s->prescale_match[0];
    case 0x044: /* TBPMR */
        return s->prescale_match[1];
    case 0x048: /* TAR */
        return get_time_register(s, get_mode(s, GPTM_A), GPTM_A, false);
    case 0x04c: /* TBR */
        uint32_t time = get_time_register(s, get_mode(s, GPTM_B), GPTM_B, false);
        if (s->cfg != GPTM_CFG_SHORT) {
            return time >> 16;
        }
        return time;
    case 0x050: // TAV
        return get_time_register(s, get_mode(s, GPTM_A), GPTM_A, true);
    case 0x054: // TBV
        uint32_t time = get_time_register(s, get_mode(s, GPTM_B), GPTM_B, true);
        if (s->cfg != GPTM_CFG_SHORT) {
            return time >> 16;
        }
        return time;
    case 0x058:
        return s->rtcpd;
    case 0x05c:
        return s->prescale_snapshot[0];
    case 0x060:
        return s->prescale_snapshot[1];
    case 0x064:
        return s->prescale_value[0];
    case 0x068:
        return s->prescale_value[1];
    case 0xfc0:
        return s->pp;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPTM: read at bad offset 0x%x\n",
                      offset);
        return 0;
    }
}

static void gptm_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    GPTMState *s = opaque;
    uint32_t oldval;

    uint8_t mode = 0;
    if (s->width == GPTM_WIDTH_32_64) {
        mode++;
    }
    if (s->cfg != GPTM_CFG_SHORT) {
        mode++;
    }

    /*
     * The timers should be disabled before changing the configuration.
     * We take advantage of this and defer everything until the timer
     * is enabled.
     */
    switch (offset) {
    case 0x000: /* CFG */
        s->cfg = value;
        break;
    case 0x004: /* TAMR */
        s->mode[0] = value;
        break;
    case 0x008: /* TBMR */
        s->mode[1] = value;
        break;
    case 0x00c: /* CTL */
        // TODO!!!!!
        oldval = s->ctl;
        s->ctl = value;
        /* TODO: Implement pause.  */
        if ((oldval ^ value) & GPTM_CTL_TAEN) {
            if (value & 1) {
                gptm_reload(s, 0, 1);
            } else {
                gptm_stop(s, 0);
            }
        }
        if (((oldval ^ value) & GPTM_CTL_TBEN) && s->cfg == GPTM_CFG_SHORT) {
            if (value & 0x100) {
                gptm_reload(s, 1, 1);
            } else {
                gptm_stop(s, 1);
            }
        }
        break;
    case 0x010:
        s->sync = value;
        break;
    case 0x018: /* IMR */
        s->imr = value;
        gptm_update_irq(s);
        break;
    case 0x024: /* CR */
        s->ris &= ~value;
        break;
    case 0x028: /* TAILR */
        switch (mode) {
            case GPTM_LEN_32:
                // Loads B with the top 16 bits of the input, fall through to loading A
                s->load[1] = (value >> 16) & 0xffff;
            case GPTM_LEN_16:
                s->load[0] = value & 0xffff;
                break;
            case GPTM_LEN_64:
                // Store 32-bit input 'value' into bottom 32 bits of load_64
                s->load_64 = (s->load_64 & 0xffffffff00000000) | (value & 0xffffffff);
                break;
        }
        break;
    case 0x02c: /* TBILR */
        switch (mode) {
            case GPTM_LEN_16:
                s->load[1] = value & 0xffff;
                break;
            case GPTM_LEN_32:
                // Writes ignored in 32-bit mode; controlled by TAILR
                break;
            case GPTM_LEN_64:
                // Store 32-bit input 'value' into top 32 bits of load_64
                s->load_64 = (s->load_64 & 0x00000000ffffffff) | ((value & 0xffffffff) << 32);
                break;
        }
        break;
    case 0x030: /* TAMATCHR */
        s->match[0] = value & 0xffff;
        // TODO
        if (s->cfg < 4) {
            s->match[1] = value >> 16;
        }
        break;
    case 0x034: /* TBMATCHR */
        s->match[1] = value >> 16;
        // TODO
        break;
    case 0x038: /* TAPR */
        s->prescale[0] = value;
        break;
    case 0x03c: /* TBPR */
        s->prescale[1] = value;
        break;
    case 0x040: /* TAPMR */
        s->prescale_match[0] = value;
        break;
    case 0x044: /* TBPMR */
        s->prescale_match[1] = value;
        break;
    case 0x050: // TAV
        // TODO Writes to TAR
        break;
    case 0x054: // TBV
        // TODO Writes to TBR
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPTM: write at bad offset 0x%x\n", (int)offset);
    }
    gptm_update_irq(s);
}

static const MemoryRegionOps gptm_ops = {
    .read = gptm_read,
    .write = gptm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_gptm = {
    .name = TYPE_TM4_TIMER,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, GPTMState),
        VMSTATE_UINT32_ARRAY(mode, GPTMState, 2),
        VMSTATE_UINT32(ctl, GPTMState),
        VMSTATE_UINT32(ris, GPTMState),
        VMSTATE_UINT32(imr, GPTMState),
        VMSTATE_UNUSED(8),
        VMSTATE_UINT32_ARRAY(load, GPTMState, 2),
        VMSTATE_UINT32_ARRAY(match, GPTMState, 2),
        VMSTATE_UINT32_ARRAY(prescale, GPTMState, 2),
        VMSTATE_UINT32_ARRAY(prescale_match, GPTMState, 2),
        VMSTATE_UINT32(rtcpd, GPTMState),
        VMSTATE_INT64_ARRAY(tick, GPTMState, 2),
        VMSTATE_TIMER_PTR_ARRAY(timer, GPTMState, 2),
        VMSTATE_CLOCK(clk, GPTMState),
        VMSTATE_END_OF_LIST()
    }
};

static void gptm_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    GPTMState *s = TM4_GPTM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->nvic_irq);
    qdev_init_gpio_out(dev, &s->ccp_gpio, 2);
    qdev_init_gpio_out(dev, &s->trigger_adc, 1);

    // TODO Input CCP

    memory_region_init_io(&s->iomem, obj, &gptm_ops, s,
                          TYPE_TM4_TIMER, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    s->opaque[0] = s->opaque[1] = s;

    /*
     * TODO: in an ideal world we would model the effects of changing
     * the input clock frequency while the countdown timer is active.
     * The best way to do this would be to convert the device to use
     * ptimer instead of hand-rolling its own timer. This would also
     * make it easy to implement reading the current count from the
     * TAR and TBR registers.
     */
    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
}

static void gptm_realize(DeviceState *dev, Error **errp)
{
    GPTMState *s = TM4_GPTM(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "stellaris-gptm: clk must be connected");
        return;
    }

    s->ns_per_cycle = clock_ticks_to_ns(s->clk, 1);

    s->timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, gptm_tick, &s->opaque[0]);
    s->timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, gptm_tick, &s->opaque[1]);
}

static void gptm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_gptm;
    dc->realize = gptm_realize;
}

static const TypeInfo gptm_info = {
    .name          = TYPE_TM4_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GPTMState),
    .instance_init = gptm_init,
    .class_init    = gptm_class_init,
};

static void gptm_register_types(void)
{
    type_register_static(&gptm_info);
}

type_init(gptm_register_types)
