#include "hw/arm/tm4c123gh6pm/board/include/uart.h"

static const unsigned char uart_id[8] =
  { 0x11, 0x00, 0x18, 0x01, 0x0d, 0xf0, 0x05, 0xb1 };

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

DeviceState *uart_create(hwaddr addr, uint8_t uart, qemu_irq nvic_irq, qemu_irq tx_gpio, qemu_irq rts, qemu_irq cts)
{
    DeviceState *dev = qdev_new(TYPE_TM4_UART);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_prop_set_uint8(dev, "uart", uart);
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

static void uart_update(UARTState *s)
{
    qemu_set_irq(s->nvic_irq, (s->ris & s->im) != 0);
}

static bool uart_fifo_enabled(UARTState *s)
{
    return (s->lcrh & 0x10) != 0;
}

static inline unsigned uart_get_fifo_depth(UARTState *s)
{
    // if fifo disabled, the fifos are 1 byte deep
    return uart_fifo_enabled(s) ? UART_FIFO_DEPTH : 1;
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
            s->fr &= ~UART_FLAG_RXFF;
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

// static unsigned int uart_get_baudrate(const UARTState *s)
// {
//     uint64_t clk;

//     if (s->ibrd == 0) {
//         return 0;
//     }

//     clk = clock_get_hz(s->clk);
//     return (clk / ((s->ibrd << 6) + s->fbrd)) << 2;
// }

static void uart_send(UARTState *s) {
    // Check send and uart are enabled
    if ((~s->ctl & 0x1) | (~s->ctl & 0x100)) {
        return;
    }
    uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
    uint8_t wlen_mask = ((1 << wlen) - 1);
    // wlen field adds up to 3 bits from 5 base bits
    uint8_t reverse_data = s->dr & wlen_mask;
    uint16_t word;
    for (int i = 0; i < wlen; i++) {
        word = ((word << 1) | (reverse_data & 0x1)) & wlen_mask;
        reverse_data >>= 1;
    }
    // Calculate parity
    if (s->lcrh & 0x2) {
        wlen++;
        uint8_t parity = count_set_bits(word) & 0x1;
        // Stick Parity Bit
        // TODO
        /*
        if (s->lcrh & 0x80) {
            parity = ()
        }
        */
        if (s->lcrh & 0x4) {
            // Even parity
            word = (word << 1) | (parity == 0);
        } else {
            // Odd parity
            word = (word << 1) | (parity == 1);
        }
    }
    // Calculate number of stop bits
    if (s->lcrh & 0x8) {
       word = (word << 2) | 0x3;
        wlen += 2;
    } else {
        word = (word << 1) | 0x1;
        wlen++;
    }
    uint32_t output = word;
    wlen++;

    // TODO: End of transmission
    // s->ctl & 0x10;
    s->ris |= UART_INT_TX;
    // TODO: add waits?
    printf("[UART %d TX FRAME] 0x%.3X\n", s->uart, output);
    for (int i = 0; i < wlen; i++) {
        // Output the frame from right to left to irq
        qemu_set_irq(s->tx_gpio, (word & (1 << (wlen - 1 - i))) != 0);
    }
}

static void uart_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    UARTState *s = opaque;

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
            if ((s->lcrh ^ value) & 0x10) {
                uart_reset_fifo(s);
            }
            // Maybe do something with send break
            s->lcrh = value;
            // TODO: might be broken in the future?
            s->read_trigger = 1;
            break;
        case 0x030:
            reg = "CTL";
            s->ctl = value;
            break;
        case 0x034:
            reg = "IFLS";
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

    printf("[UART %d %s] 0x%.8X\n", s->uart, reg, output);
}

static void uart_put_fifo(void *opaque, uint32_t value)
{
    UARTState *s = opaque;

    int pipe_depth = uart_get_fifo_depth(s);
    if (s->read_count >= pipe_depth) {
        // Data is trashed, fifo overrun
        s->dr |= 0x800;
        s->ris |= UART_INT_OE;
        uart_update(s);
        return;
    }

    int slot = (s->read_pos + s->read_count) & (pipe_depth - 1);
    s->read_fifo[slot] = value;
    s->read_count++;
    s->fr &= ~UART_FLAG_RXFE;
    if (s->read_count == pipe_depth) {
        s->fr |= UART_FLAG_RXFF;
    }
    if (s->read_count == s->read_trigger) {
        s->ris |= UART_INT_RX;
        uart_update(s);
    }
}

static void reset_frame(UARTState *s)
{
    s->receive_frame = 0;
    s->read_bit = 0;
    // clear busy bit
    s->fr &= ~0x8;
}

static void uart_receive(void *opaque, int irq, int level)
{
    UARTState *s = opaque;

    // int pipe_depth = uart_get_fifo_depth(s);
    // int slot = (s->read_pos + s->read_count) & (pipe_depth - 1);

    // set busy bit
    s->fr |= 0x8;

    uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
    uint8_t parity = (s->lcrh >> 1) & 0x1;
    uint8_t stop_count = 1 + ((s->lcrh >> 3) & 0x1);
    uint8_t frame_size = 1 + wlen + parity + stop_count;
    uint8_t bit = level != 0;

    s->receive_frame = (s->receive_frame << 1) | bit;
    s->read_bit++;

    if (s->read_bit >= frame_size) {
        uint16_t frame = s->receive_frame;

        uint8_t start = (frame >> (frame_size - 1)) & 0x1;
        frame_size--;
        if (start != 0) {
            goto frame_error;
        }
        uint8_t wlen_mask = (1 << wlen) - 1;
        uint8_t reverse_word = (frame >> (frame_size - wlen)) & wlen_mask;
        uint8_t word;
        for (int i = 0; i < wlen; i++) {
            word = ((word << 1) | (reverse_word & 0x1)) & wlen_mask;
            reverse_word >>= 1;
        }
        frame_size -= wlen;
        if (parity) {
            parity = (frame >> (frame_size - 1)) & 0x1;
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

        printf("[UART %d RX FRAME] 0x%.3X\n", s->uart, s->receive_frame);
        uart_put_fifo(s, word);
        goto reset;
    }
    return;
    
    frame_error:
    s->dr |= 0x100;
    s->ris |= UART_INT_FE;
    // fall through and reset and return
    reset:
    reset_frame(s);
    return;

    // // Start reading on a start bit
    // if (!s->reading && level == 0) {
    //     s->read_stop_count = (s->lcrh & 0x8) ? 1 : 0;
    //     s->receive_frame = 0;
    //     s->temp_read = 0;
    //     s->read_bit = 1;
    //     s->reading = true;
    //     // Set BUSY bit
    //     s->fr |= 0x8;
    //     return;
    // }
    // if (!s->reading) {
    //     s->dr |= 0x100;
    //     s->ris |= UART_INT_FE;
    //     uart_update(s);
    //     return;
    // }
    // // Data bits
    // if (s->read_bit < wlen + 1) {
    //     s->temp_read |= bit << (s->read_bit - 1);
    //     s->read_bit++;
    //     return;
    // }
    // // Parity bit
    // if ((s->lcrh & 0x2) && s->read_bit == wlen + 1) {
    //     // uint8_t bits_set_parity = count_set_bits(s->read_fifo[slot]) & 0x1;
    //     // if (s->lcrh & 0x4) {
    //         // Check even parity

    //     // }
    //     s->read_bit++;
    //     // TODO: parity checking
    //     // s->dr |= 0x200;
    //     // s->ris |= UART_INT_PE;
    //     // uart_update(s);
    //     return;
    // }
    // // Stop bit(s)
    // if (level == 1) {
    //     if (s->read_stop_count != 0) {
    //         s->read_stop_count--;
    //         return;
    //     }
    //     s->reading = false;
    //     s->read_bit = 0;
    //     s->read_count++;
    //     // Clear BUSY bit
    //     s->fr &= ~0x8;
    //     printf("[UART %d RX FRAME] 0x%.3X\n", s->uart, s->receive_frame);
    //     uart_put_fifo(s, s->temp_read);
    //     s->temp_read = 0;
    //     s->receive_frame = 0;
    //     return;
    // }
    // // Framing error, no stop bit?
    // s->dr |= 0x100;
    // s->ris |= UART_INT_FE;
    // uart_update(s);
}

static void uart_clock_update(void *opaque, ClockEvent event)
{
    // do nothing for now
    // UARTState *s = TM4_UART(opaque);
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
    DEFINE_PROP_END_OF_LIST()
};

static void uart_init(Object *obj)
{
    UARTState *s = TM4_UART(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s, TYPE_TM4_UART, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(sbd, &s->nvic_irq);

    qdev_init_gpio_out(dev, &s->tx_gpio, 1);
    qdev_init_gpio_in(dev, uart_receive, 1);

    s->clk = qdev_init_clock_in(dev, "clk", uart_clock_update, s, ClockUpdate);

    s->id = uart_id;
}

static void uart_realize(DeviceState *dev, Error **errp)
{
    // UARTState *s = TM4_UART(dev);

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
