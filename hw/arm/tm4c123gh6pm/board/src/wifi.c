#include "hw/arm/tm4c123gh6pm/board/include/wifi.h"

DeviceState *wifi_create(hwaddr addr, qemu_irq tx_gpio, Clock *clk)
{
    DeviceState *dev = qdev_new(TYPE_CYBOT_WIFI);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_connect_clock_in(dev, "clk", clk);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, addr);

    qdev_connect_gpio_out(dev, 0, tx_gpio);

    return dev;
}

static int read_rx_state(WIFIState *s)
{
    if (s->rx_count <= 0) {
        return -1;
    }
    uint32_t mask = 1 << (s->rx_pos % 32);
    int array_pos = s->rx_pos / 32;
    bool bit = s->rx_state[array_pos] & mask;
    s->rx_pos = (s->rx_pos + 1) % (32 * WIFI_RX_FIFO_DEPTH);
    s->rx_count--;
    return bit;
}

static void write_rx_state(WIFIState *s, bool bit)
{
    if (!s->reading || s->rx_count >= (32 * WIFI_RX_FIFO_DEPTH)) {
        return;
    }
    // print_rx_state(s, "WIFI");
    int pos = s->rx_pos + s->rx_count;
    uint64_t mask = 1 << (pos % 32);
    int array_pos = pos / 32;
    if (bit) {
        s->rx_state[array_pos] |= mask;
    } else {
        s->rx_state[array_pos] &= ~mask;
    }
    s->rx_count++;
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

static void parse_frame(WIFIState *s)
{
    uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
    uint8_t parity = (s->lcrh >> 1) & 0x1;
    uint8_t stop_count = 1 + ((s->lcrh >> 3) & 0x1);

    int frame_size = s->frame_size;
    uint32_t frame = s->receive_frame;
    printf("WIFI Recvd: %08X\n", frame);

    uint8_t start = (frame >> (frame_size - 1)) & 0x1;
    frame_size--;
    if (start != 0) {
        return;
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
            return;
        }

        frame_size--;
    }
    uint8_t stop_mask = ((1 << stop_count) - 1);
    uint8_t stop = frame & stop_mask;
    frame_size -= stop_count;
    if (frame_size != 0) {
        return;
    }
    if ((stop ^ stop_mask) != 0 || stop == 0) {
        return;
    }

    wifi_send(word, true);
    return;
}

static uint64_t get_baud_time_ns(WIFIState *s)
{
    return (uint64_t)(
        (double)(s->ibrd + ((s->fbrd - (double)0.5) / 64)) * 1000
    );
}

static void wifi_reload(QEMUTimer *timer, uint64_t *pTick, uint64_t timeout, bool reset)
{
    if (reset) {
        (*pTick) = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }

    (*pTick) += timeout;

    timer_mod(timer, *pTick);
}

static void wifi_new_write_frame(WIFIState *s) {
    uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
    uint8_t wlen_mask = ((1 << wlen) - 1);
    char data = wifi_recv(SERVER_STUDENT);
    if (data == -1) {
        return;
    }
    printf("WiFi Send Data %d\n", data);
    uint8_t reverse_data = data & wlen_mask;
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

    s->write_frame = word;
    printf("[WiFi WRITE] 0x%08X\n", word);
}

static void tick_write(void *opaque)
{
    WIFIState *s = opaque;

    // UART by default sends 1
    bool bit = true;
    if (s->write_frame == 0 && !is_wifi_empty(SERVER_STUDENT)) {
        wifi_new_write_frame(s);
    }
    if (s->write_frame != 0) {
        s->write_bit++;
        uint32_t write_mask = 1 << (s->frame_size - s->write_bit);
        bit = (s->write_frame & write_mask) != 0;
        if (s->write_bit == s->frame_size) {
            s->write_bit = 0;
            s->write_frame = 0;
        }
    }
    qemu_set_irq(s->tx_gpio, bit);
    wifi_reload(s->write_timer, &s->write_tick, get_baud_time_ns(s), false);
}

static void tick_read(void *opaque)
{
    WIFIState *s = opaque;

    int bit = read_rx_state(s);
    // bool bit = s->rx_state;
    if (bit == -1) {
        return;
    }
    if (!s->reading && bit) {
        return;
    }
    s->reading = true;
    s->read_bit++;
    s->receive_frame = (s->receive_frame << 1) | bit;

    if (s->read_bit >= s->frame_size) {
        s->rx_count = 0;
        s->reading = false;
        s->read_bit = 0;
        parse_frame(s);
        s->receive_frame = 0;
    } else {
        wifi_reload(s->read_timer, &s->read_tick, get_baud_time_ns(s), false);
    }
}

static uint64_t cybot_wifi_read(void *opaque, hwaddr offset, unsigned size)
{
    WIFIState *s = opaque;

    switch (offset) {
        case 0x000:
            return s->lcrh;
        case 0x004:
            return s->ibrd;
        case 0x008:
            return s->fbrd;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "cybot_wifi_read: Bad offset 0x%x\n", (int)offset);
            return 0;
    }
}

static void cybot_wifi_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    WIFIState *s = opaque;

    switch (offset) {
        case 0x000:
            s->lcrh = value;
            uint8_t wlen = 5 + ((s->lcrh >> 5) & 0x3);
            uint8_t parity = (s->lcrh >> 1) & 0x1;
            uint8_t stop_count = 1 + ((s->lcrh >> 3) & 0x1);
            s->frame_size = 1 + wlen + parity + stop_count;
            return;
        case 0x004:
            s->ibrd = value;
            return;
        case 0x008:
            s->fbrd = value;
            return;
        default:
            printf("0x%x\n", (int) offset);
            qemu_log_mask(LOG_GUEST_ERROR, "cybot_wifi_write: Bad offset 0x%x\n", (int)offset);
    }
}

static const MemoryRegionOps wifi_ops =
{
    .read = cybot_wifi_read,
    .write = cybot_wifi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.max_access_size = 4,
    .impl.min_access_size = 4
};

static bool cybot_clock_needed(void *opaque)
{
    WIFIState *s = CYBOT_WIFI(opaque);

    return s->migrate_clk;
}

static const VMStateDescription vmstate_wifi_clock =
{
    .name = TYPE_CYBOT_WIFI "/clock",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = cybot_clock_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, WIFIState),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_wifi =
{
    .name = TYPE_CYBOT_WIFI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ibrd, WIFIState),
        VMSTATE_UINT32(fbrd, WIFIState),
        VMSTATE_UINT32(lcrh, WIFIState),
        VMSTATE_UINT32(receive_frame, WIFIState),
        VMSTATE_UINT32(write_frame, WIFIState),
        VMSTATE_UINT32_ARRAY(rx_state, WIFIState, WIFI_RX_FIFO_DEPTH),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_wifi_clock,
        NULL
    }
};

static Property wifi_properties[] =
{
    DEFINE_PROP_BOOL("migrate-clk", WIFIState, migrate_clk, true),
    DEFINE_PROP_UINT32("port", WIFIState, port, 10000),
    DEFINE_PROP_END_OF_LIST()
};

static void wifi_rx_gpio(void *opaque, int irq, int level)
{
    WIFIState *s = opaque;

    if (!s->reading && level == 0) {
        // start read
        wifi_reload(s->read_timer, &s->read_tick, get_baud_time_ns(s) / 2, true);
        s->reading = true;
        printf("Wifi reading...\n");
    }
    // s->rx_state = level != 0;
    write_rx_state(s, level != 0);
}

static void wifi_init(Object *obj)
{
    WIFIState *s = CYBOT_WIFI(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &wifi_ops, s, TYPE_CYBOT_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_out(dev, &s->tx_gpio, 1);
    qdev_init_gpio_in(dev, wifi_rx_gpio, 1);

    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
}

static void student_connect(void *opaque)
{
    WIFIState *s = opaque;
    // Start write timer
    wifi_reload(s->write_timer, &s->write_tick, 10000, true);
}

static void wifi_realize(DeviceState *dev, Error **errp)
{
    WIFIState *s = CYBOT_WIFI(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "cybot-wifi: clk must be connected");
        return;
    }

    s->read_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tick_read, s);
    s->write_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tick_write, s);

    create_server(SERVER_STUDENT, s->port, s, student_connect);
}

static void wifi_reset(DeviceState *dev)
{
    WIFIState *s = CYBOT_WIFI(dev);
    s->ibrd = 500;
    s->fbrd = 0;
    s->lcrh = 0x70;

    s->frame_size = 10;

    s->read_bit = 0;
    s->receive_frame = 0;
    s->rx_pos = 0;
    s->rx_count = 0;
    for (int i = 0; i < WIFI_RX_FIFO_DEPTH; i++) {
        s->rx_state[i] = 0;
    }
    // s->rx_state = true;
    s->reading = false;
    s->write_bit = 0;
    s->write_frame = 0;

}

static void wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = wifi_realize;
    dc->reset = wifi_reset;
    dc->vmsd = &vmstate_wifi;
    device_class_set_props(dc, wifi_properties);
}

static const TypeInfo wifi_info =
{
    .name = TYPE_CYBOT_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WIFIState),
    .instance_init = wifi_init,
    .class_init = wifi_class_init
};

static void wifi_register_types(void)
{
    type_register_static(&wifi_info);
}
type_init(wifi_register_types)
