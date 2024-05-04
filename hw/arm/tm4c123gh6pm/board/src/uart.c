#include "hw/arm/tm4c123gh6pm/board/include/uart.h"

static const unsigned char uart_id[8] =
  { 0x11, 0x00, 0x18, 0x01, 0x0d, 0xf0, 0x05, 0xb1 };

static const uint8_t trigger_counts[5] = {
    2, 4, 8, 12, 14
};

static void uart_update(UARTState *s)
{
    qemu_set_irq(s->nvic_irq, (s->ris & s->im) != 0);
}

static bool uart_fifo_enabled(UARTState *s)
{
    return (s->lcrh & UART_LCRH_FEN) != 0;
}

static inline unsigned uart_get_fifo_depth(UARTState *s)
{
    // if fifo disabled, the fifos are 1 byte deep
    return uart_fifo_enabled(s) ? UART_FIFO_DEPTH : 1;
}

// "Variable precision SWAR algorithm" from SO
// (https://stackoverflow.com/a/109025)
// Should be replaced by compiler with a popcount instruction if available
static uint32_t count_set_bits(uint32_t i) {
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    i = (i + (i >> 4)) & 0x0f0f0f0f;
    i *= 0x01010101;
    return i >> 24;
}

#define UART_FIFO_READ 0
#define UART_FIFO_WRITE 1

static void uart_put_fifo(UARTState *s, uint32_t value, int direction)
{
    uint32_t *fifo;
    int pos, *count;
    uint32_t flag_full, flag_empty;

    if (direction == UART_FIFO_READ) {
        fifo = s->read_fifo;
        pos = s->read_pos;
        count = &s->read_count;
        flag_full = UART_FLAG_TXFF;
        flag_empty = UART_FLAG_RXFF;
    } else if (direction == UART_FIFO_WRITE) {
        fifo = s->write_fifo;
        pos = s->write_pos;
        count = &s->write_count;
        flag_full = UART_FLAG_TXFF;
        flag_empty = UART_FLAG_RXFF;
    }

    int depth = uart_get_fifo_depth(s);
    int slot = (pos + *count) & (UART_FIFO_DEPTH - 1);
    fifo[slot] = value;
    (*count)++;
    s->fr &= ~flag_empty;
    if (*count >= depth) {
        s->fr |= flag_full;
    }
}

static void uart_put_fifo_read(UARTState *s, uint32_t value)
{
    int pipe_depth = uart_get_fifo_depth(s);
    if (s->read_count >= pipe_depth) {
        // Data is trashed, fifo overrun
        s->dr |= UART_DR_OE;
        s->ris |= UART_INT_OE;
        uart_update(s);
        return;
    }

    uart_put_fifo(s, value, UART_FIFO_READ);

    if (s->read_count >= s->read_trigger) {
        s->ris |= UART_INT_RX;
        uart_update(s);
    }
}

static void uart_put_fifo_write(UARTState *s, uint32_t value)
{
    int pipe_depth = uart_get_fifo_depth(s);
    if (s->write_count >= pipe_depth) {
        return;
    }

    uart_put_fifo(s, value, UART_FIFO_WRITE);

    if (s->write_count > s->write_trigger) {
        s->ris &= ~UART_INT_TX;
        uart_update(s);
    }
    s->fr |= UART_FLAG_BUSY;
}

static void parse_frame(UARTState *s)
{
    uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
    uint8_t parity = (s->lcrh >> 1) & 0x1;
    uint8_t stop_count = 1 + ((s->lcrh >> 3) & 0x1);

    int frame_size = s->frame_size;
    uint32_t frame = s->receive_frame;

    uint8_t start = (frame >> (frame_size - 1)) & 0x1;
    frame_size--;
    if (start != 0) {
        goto frame_error;
    }
    uint8_t wlen_mask = (1 << wlen) - 1;
    uint8_t reverse_word = (frame >> (frame_size - wlen)) & wlen_mask;
    uint8_t word = 0;
    for (int i = 0; i < wlen; i++) {
        word = ((word << 1) | (reverse_word & 0x1)) & wlen_mask;
        reverse_word >>= 1;
    }
    frame_size -= wlen;
    if (parity) {
        parity = (frame >> (frame_size - 1)) & 0x1;
        uint8_t calc_parity = count_set_bits(word) & 0x1;
        if (s->lcrh & UART_LCRH_SPS) {
            calc_parity = !(s->lcrh & UART_LCRH_EPS);
        } else if (!(s->lcrh & UART_LCRH_EPS)) {
            calc_parity = !calc_parity;
        }

        if (parity != calc_parity) {
            s->dr |= UART_DR_PE;
            s->ris |= UART_INT_PE;
        }

        frame_size--;
        // todo check parity
    }
    uint8_t stop_mask = ((1 << stop_count) - 1);
    uint8_t stop = frame & stop_mask;
    frame_size -= stop_count;
    if (frame_size != 0) {
        goto frame_error;
    }
    if ((stop ^ stop_mask) != 0 || stop == 0) {
        goto frame_error;
    }

    if (s->debug) {
        printf("[UART %d RX FRAME] 0x%.3X\n", s->uart, s->receive_frame);
    }
    uart_put_fifo_read(s, word);
    return;
    
    frame_error:
    s->dr |= UART_DR_FE;
    s->ris |= UART_INT_FE;
    return;
}

static void uart_stop(QEMUTimer *timer)
{
    timer_del(timer);
}

static uint64_t get_baud_time_ns(UARTState *s)
{
    return (uint64_t)(
        (double)(s->ibrd + ((s->fbrd - (double)0.5) / 64)) * 1000
    );
}

static void uart_reload(QEMUTimer *timer, int64_t *pTick, uint64_t timeout, bool reset)
{
    if (reset) {
        (*pTick) = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }

    (*pTick) += timeout;

    timer_mod(timer, *pTick);
}

static void uart_tick_write(void *opaque)
{
    UARTState *s = opaque;

    // UART by default sends 1
    bool bit = true;
    if (s->write_count > 0) {
        s->write_bit++;
        uint32_t write_frame = s->write_fifo[s->write_pos];
        uint32_t write_mask = 1 << (s->frame_size - s->write_bit);
        bit = (write_frame & write_mask) != 0;
        if (s->write_bit == s->frame_size) {
            s->write_bit = 0;
            s->write_count--;
            s->write_pos = (s->write_pos + 1) & 0xf;
            s->fr &= ~UART_FLAG_TXFF;
            if (s->write_count <= s->write_trigger) {
                qemu_irq_pulse(s->tx_gpio);
                uart_update(s);
            }
            if (s->write_count <= 0) {
                s->fr &= ~UART_FLAG_BUSY;
                s->fr |= UART_FLAG_TXFE;
            }
        }
    }
    qemu_set_irq(s->tx_gpio, bit);
    uart_reload(s->write_timer, &s->write_tick, get_baud_time_ns(s), false);
}

static void uart_tick_read(void *opaque)
{
    UARTState *s = opaque;

    if (!s->reading && s->rx_state != 0) {
        return;
    }
    s->reading = true;
    s->read_bit++;
    s->receive_frame = (s->receive_frame << 1) | s->rx_state;

    if (s->read_bit >= s->frame_size) {
        s->reading = false;
        s->read_bit = 0;
        parse_frame(s);
        s->receive_frame = 0;
    } else {
        uart_reload(s->read_timer, &s->read_tick, get_baud_time_ns(s), false);
    }

}

DeviceState *uart_create(bool debug, hwaddr addr, uint8_t uart, qemu_irq nvic_irq, qemu_irq tx_gpio, qemu_irq rts, qemu_irq cts, Clock *clk)
{
    DeviceState *dev = qdev_new(TYPE_TM4_UART);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_prop_set_uint8(dev, "uart", uart);
    qdev_prop_set_bit(dev, "debug", debug);
    qdev_connect_clock_in(dev, "clk", clk);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, addr);
    sysbus_connect_irq(sbd, 0, nvic_irq);

    if (rts != NULL && cts != NULL) {
        // Do nothing for now, unimplemented
        // TODO
    }

    qdev_connect_gpio_out(dev, 0, tx_gpio);

    return dev;
}

static inline void uart_reset_fifo(UARTState *s)
{
    s->read_count = 0;
    s->read_pos = 0;

    // reset flags
    s->fr &= ~(UART_FLAG_RXFF | UART_FLAG_TXFF);
    s->fr |= UART_FLAG_RXFE | UART_FLAG_TXFE;
}

static uint64_t uart_read(void *opaque, hwaddr offset, unsigned size)
{
    UARTState *s = opaque;

    switch (offset) {
        case 0x000:
            uint32_t c = s->read_fifo[s->read_pos];
            if (s->read_count > 0) {
                s->read_count--;
                s->read_pos = (s->read_pos + 1) & (uart_get_fifo_depth(s) - 1);
            }
            if (s->read_count == 0) {
                s->fr |= UART_FLAG_RXFE;
            }
            if (s->read_count == s->read_trigger - 1) {
                s->ris &= ~UART_INT_RX;
            }
            s->rsrecr = c >> 8;
            s->fr &= ~UART_FLAG_RXFF;
            uart_update(s);
            // qemu_chr_fe_accept_input(&s->chr);
            return c;
        case 0x004:
            return s->rsrecr;
        case 0x018:
            return s->fr;
        case 0x020:
            return s->ilpr;
        case 0x024:
            return s->ibrd;
        case 0x028:
            return s->fbrd;
        case 0x02c:
            return s->lcrh;
        case 0x030:
            return s->ctl;
        case 0x034:
            return s->ifls;
        case 0x038:
            return s->im;
        case 0x03c:
            return s->ris;
        case 0x040:
            return s->ris & s->im;
        case 0x048:
            return s->dmactl;
        case 0x0a4:
            return s->addr;
        case 0x0a8:
            return s->amask;
        case 0x0fc0:
            return 0x3;
        case 0x0fc8:
            return s->cc;
        case 0xfd0 ... 0xffc:
            return s->id[(offset - 0xfd0) >> 2];
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "uart_read: Bad offset 0x%x\n", (int)offset);
            return 0;
    }
}

static void uart_send(UARTState *s) {
    // Check send and uart are enabled
    if ((~s->ctl & UART_CTL_UARTEN) | (~s->ctl & UART_CTL_TXE)) {
        return;
    }
    uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
    uint8_t wlen_mask = ((1 << wlen) - 1);
    uint8_t reverse_data = s->dr & wlen_mask;
    uint16_t word = 0;
    for (int i = 0; i < wlen; i++) {
        word = ((word << 1) | (reverse_data & 0x1)) & wlen_mask;
        reverse_data >>= 1;
    }
    // Calculate parity
    if (s->lcrh & UART_LCRH_PEN) {
        wlen++;
        uint8_t parity = count_set_bits(word) & 0x1;
        if (s->lcrh & UART_LCRH_SPS) {
            word = (word << 1) | !(s->lcrh & UART_LCRH_EPS);
        } else if (s->lcrh & UART_LCRH_EPS) {
            // Even parity
            word = (word << 1) | (parity);
        } else {
            // Odd parity
            word = (word << 1) | !(parity);
        }
    }
    // Calculate number of stop bits
    if (s->lcrh & UART_LCRH_STP2) {
        word = (word << 2) | 0x3;
        wlen += 2;
    } else {
        word = (word << 1) | 0x1;
        wlen++;
    }

    if (s->debug) {
        printf("[UART %d TX FRAME] 0x%.3X\n", s->uart, word);
    }
    uart_put_fifo_write(s, word);
}

static void uart_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    UARTState *s = opaque;

    if (!s->clock_active) {
        return;
    }

    const char *reg = "BAD OFFSET";
    uint32_t output = value;

    switch (offset) {
        case 0x000:
            reg = "DATA";
            s->dr = value;

            if (~s->ctl & 0x8000) {
                // CTS disabled
                uart_send(s);
            }
            break;
        case 0x004:
            reg = "RSR/ECR";
            output = 0;
            s->rsrecr = 0;
            break;
        case 0x020:
            reg = "ILPR";
            s->ilpr = value;
            break;
        // Figure out if we need to do something with baudrate
        case 0x024:
            reg = "IBRD";
            s->ibrd = value;
            break;
        case 0x028:
            reg = "FBRD";
            s->fbrd = value;
            break;
        case 0x02c:
            reg = "LCRH";
            // Reset fifo on enable / disable
            if ((s->lcrh ^ value) & UART_LCRH_FEN) {
                uart_reset_fifo(s);
            }
            // Maybe do something with send break
            s->lcrh = value;
            uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
            uint8_t parity = (s->lcrh >> 1) & 0x1;
            uint8_t stop_count = 1 + ((s->lcrh >> 3) & 0x1);
            s->frame_size = 1 + wlen + parity + stop_count;
            break;
        case 0x030:
            reg = "CTL";
            uint32_t ctl_delta = s->ctl ^ value;
            s->ctl = value;
            if (ctl_delta & UART_CTL_UARTEN) {
                if (value & UART_CTL_UARTEN) {
                    if (value & UART_CTL_TXE) {
                        uart_reload(s->write_timer, &s->write_tick, 0, true);
                    }
                } else {
                    uart_stop(s->read_timer);
                    uart_stop(s->write_timer);
                }
            } else {
                if (ctl_delta & UART_CTL_TXE) {
                    if ((value & UART_CTL_UARTEN) && (value & UART_CTL_TXE)) {
                        uart_reload(s->write_timer, &s->write_tick, 0, true);
                    } else {
                        uart_stop(s->write_timer);
                    }
                }
                if (ctl_delta & UART_CTL_RXE) {
                    if (!((value & UART_CTL_UARTEN) && (value & UART_CTL_RXE))) {
                        uart_stop(s->read_timer);
                    }
                }
            }
            // TODO: might want to stop timer if both RX and TX are disabled
            break;
        case 0x034:
            reg = "IFLS";
            if (s->lcrh & UART_LCRH_FEN) {
                uint8_t txifls = value & 0x7;
                uint8_t rxifls = (value >> 3) & 0x7;
                if (txifls > 4) {
                    txifls = 4;
                }
                if (rxifls > 4) {
                    rxifls = 4;
                }
                s->write_trigger = (s->ctl & UART_CTL_EOT) ? 0 : trigger_counts[txifls];
                s->read_trigger = (s->ctl & UART_CTL_EOT) ? UART_FIFO_DEPTH : trigger_counts[rxifls];
            } else {
                s->write_trigger = 0;
                s->read_trigger = 1;
            }
            s->ifls = value;
            break;
        case 0x038:
            reg = "IM";
            s->im = value;
            uart_update(s);
            break;
        case 0x044:
            reg = "ICR";
            s->ris &= ~value;
            uart_update(s);
            break;
        case 0x048:
            reg = "DMACTL";
            s->dmactl = value;
            break;
        case 0x0a4:
            reg = "9BITADDR";
            s->addr = value;
            break;
        case 0x0a8:
            reg = "9BITAMASK";
            s->amask = value;
            break;
        case 0x0fc8:
            reg = "CC";
            s->cc = value;
            break;
        default:
            printf("0x%x\n", (int) offset);
            qemu_log_mask(LOG_GUEST_ERROR, "uart_write: Bad offset 0x%x\n", (int)offset);
    }

    if (s->debug) {
        printf("[UART %d %s] 0x%.8X\n", s->uart, reg, output);
    }
}


static void check_clock(void *opaque, ClockEvent event)
{
    // do nothing for now
    // UARTState *s = TM4_UART(opaque);
    UARTState *s = TM4_UART(opaque);
    s->clock_active = clock_get(s->clk) != 0;
}

static const MemoryRegionOps uart_ops =
{
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.max_access_size = 4,
    .impl.min_access_size = 4
};

static bool uart_clock_needed(void *opaque)
{
    UARTState *s = TM4_UART(opaque);

    return s->migrate_clk;
}

static const VMStateDescription vmstate_uart_clock =
{
    .name = TYPE_TM4_UART "/clock",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = uart_clock_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, UARTState),
        VMSTATE_END_OF_LIST()
    }
};

static int uart_post_load(void *opaque, int version_id)
{
    UARTState *s = opaque;

    if (s->read_pos >= UART_FIFO_DEPTH || s->read_count > UART_FIFO_DEPTH) {
        return -1;
    }
    if (s->write_pos >= UART_FIFO_DEPTH) {
        return -1;
    }

    return 0;
}

static const VMStateDescription vmstate_uart =
{
    .name = TYPE_TM4_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = uart_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dr, UARTState),
        VMSTATE_UINT32(rsrecr, UARTState),
        VMSTATE_UINT32(fr, UARTState),
        VMSTATE_UINT32(ilpr, UARTState),
        VMSTATE_UINT32(ibrd, UARTState),
        VMSTATE_UINT32(fbrd, UARTState),
        VMSTATE_UINT32(lcrh, UARTState),
        VMSTATE_UINT32(ctl, UARTState),
        VMSTATE_UINT32(ifls, UARTState),
        VMSTATE_UINT32(im, UARTState),
        VMSTATE_UINT32(ris, UARTState),
        VMSTATE_UINT32(dmactl, UARTState),
        VMSTATE_UINT32(addr, UARTState),
        VMSTATE_UINT32(amask, UARTState),
        VMSTATE_UINT32(pp, UARTState),
        VMSTATE_UINT32(cc, UARTState),
        VMSTATE_UINT32_ARRAY(read_fifo, UARTState, UART_FIFO_DEPTH),
        VMSTATE_INT32(write_pos, UARTState),
        VMSTATE_UINT32_ARRAY(write_fifo, UARTState, UART_FIFO_DEPTH),
        VMSTATE_INT32(read_pos, UARTState),
        VMSTATE_INT32(read_count, UARTState),
        VMSTATE_INT32(read_trigger, UARTState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_uart_clock,
        NULL
    }
};

static Property uart_properties[] =
{
    DEFINE_PROP_UINT8("uart", UARTState, uart, 0),
    DEFINE_PROP_BOOL("migrate-clk", UARTState, migrate_clk, true),
    DEFINE_PROP_BOOL("debug", UARTState, debug, false),
    DEFINE_PROP_END_OF_LIST()
};

static void uart_rx_gpio(void *opaque, int irq, int level)
{
    UARTState *s = opaque;

    if (~s->ctl & (UART_CTL_RXE | UART_CTL_UARTEN)) {
        return;
    }

    if (!s->reading && level == 0) {
        // start read
        uart_reload(s->read_timer, &s->read_tick, get_baud_time_ns(s) / 2, true);
    }
    s->rx_state = (level != 0);
}

static void uart_init(Object *obj)
{
    UARTState *s = TM4_UART(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s, TYPE_TM4_UART, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(sbd, &s->nvic_irq);

    qdev_init_gpio_out(dev, &s->tx_gpio, 1);
    qdev_init_gpio_in(dev, uart_rx_gpio, 1);

    s->clk = qdev_init_clock_in(dev, "clk", check_clock, s, ClockUpdate);

    s->id = uart_id;
}



static void uart_realize(DeviceState *dev, Error **errp)
{
    UARTState *s = TM4_UART(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "tm4-uart: clk must be connected");
        return;
    }

    s->read_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_tick_read, s);
    s->write_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_tick_write, s);

    // nothing for now
}

static void uart_reset(DeviceState *dev)
{
    UARTState *s = TM4_UART(dev);
    s->dr = 0;
    s->rsrecr = 0;
    s->fr = 0;
    s->ilpr = 0;
    s->ibrd = 0;
    s->fbrd = 0;
    s->lcrh = 0;
    s->ctl = 0x300;
    s->ifls = 0x12;
    s->im = 0;
    s->ris = 0;
    s->dmactl = 0;
    s->addr = 0;
    s->amask = 0xff;
    s->pp = 0x3;
    s->cc = 0;

    s->read_trigger = 1;
    s->write_trigger = 0;

    s->frame_size = 0;
    s->read_pos = 0;
    s->read_count = 0;
    s->read_bit = 0;
    s->receive_frame = 0;
    s->rx_state = false;
    s->reading = false;
    s->write_pos = 0;
    s->write_count = 0;
    s->write_bit = 0;
    for (int i = 0; i < UART_FIFO_DEPTH; i++) {
        s->read_fifo[i] = 0;
        s->write_fifo[i] = 0;
    }


    uart_reset_fifo(s);
}

static void uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = uart_realize;
    dc->reset = uart_reset;
    dc->vmsd = &vmstate_uart;
    device_class_set_props(dc, uart_properties);
}

static const TypeInfo uart_info =
{
    .name = TYPE_TM4_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(UARTState),
    .instance_init = uart_init,
    .class_init = uart_class_init
};

static void uart_register_types(void)
{
    type_register_static(&uart_info);
}
type_init(uart_register_types)
