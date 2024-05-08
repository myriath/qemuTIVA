#ifndef CYBOT_WIFI_H_
#define CYBOT_WIFI_H_

#include "uart.h"
#include "socket_handler.h"

#define WIFI_RX_FIFO_DEPTH 2

struct WIFIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // uint64_t BaudRate;
    uint32_t ibrd;
    uint32_t fbrd;
    uint32_t lcrh;

    bool migrate_clk;

    int frame_size;

    int read_bit;
    uint32_t receive_frame;
    int rx_pos;
    int rx_count;
    uint32_t rx_state[WIFI_RX_FIFO_DEPTH];
    // bool rx_state;
    bool reading;

    int write_bit;
    uint32_t write_frame;

    // Output IRQ
    qemu_irq tx_gpio;
    // Input IRQ
    qemu_irq rx_gpio;

    // Stuff for timing uart send / receive
    uint64_t write_tick;
    uint64_t read_tick;
    QEMUTimer *write_timer;
    QEMUTimer *read_timer;
    Clock *clk;

    uint32_t port;
};

#define TYPE_CYBOT_WIFI "cybot-wifi"
OBJECT_DECLARE_SIMPLE_TYPE(WIFIState, CYBOT_WIFI)

DeviceState *wifi_create(hwaddr addr, qemu_irq tx_gpio, Clock *clk);

#endif
