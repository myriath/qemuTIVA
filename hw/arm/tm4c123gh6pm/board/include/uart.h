#ifndef TM4_UART_H_
#define TM4_UART_H_

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define UART_FLAG_TXFE  0x0080
#define UART_FLAG_RXFF  0x0040
#define UART_FLAG_TXFF  0x0020
#define UART_FLAG_RXFE  0x0010
#define UART_FLAG_BUSY  0x0008
#define UART_FLAG_CTS   0x0001

#define UART_INT_9BIT   0x1000
#define UART_INT_OE     0x0400
#define UART_INT_BE     0x0200
#define UART_INT_PE     0x0100
#define UART_INT_FE     0x0080
#define UART_INT_RT     0x0040
#define UART_INT_TX     0x0020
#define UART_INT_RX     0x0010
#define UART_INT_CTS    0x0002

#define UART_FIFO_DEPTH 16

#define COUNT_UART      8

struct UARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t dr;
    uint32_t rsrecr;
    uint32_t fr;
    uint32_t ilpr;
    uint32_t ibrd;
    uint32_t fbrd;
    uint32_t lcrh;
    uint32_t ctl;
    uint32_t ifls;
    uint32_t im;
    uint32_t ris;
    uint32_t dmactl;
    uint32_t addr;
    uint32_t amask;
    uint32_t pp;
    uint32_t cc;

    bool migrate_clk;

    int read_pos;
    int read_count;
    int read_trigger;
    int read_bit;
    int read_stop_count;
    bool reading;
    uint32_t receive_frame;
    uint32_t temp_read;
    uint32_t read_fifo[UART_FIFO_DEPTH];

    int write_pos;
    int write_count;
    uint32_t write_fifo[UART_FIFO_DEPTH];

    // Output IRQs
    qemu_irq nvic_irq;
    qemu_irq tx_gpio;
    // Input IRQ
    qemu_irq rx_gpio;

    Clock *clk;

    const uint8_t *id;
    const uint8_t uart;
};

#define TYPE_TM4_UART "tm4-uart"
OBJECT_DECLARE_SIMPLE_TYPE(UARTState, TM4_UART)

DeviceState *uart_create(hwaddr addr, uint8_t uart, qemu_irq nvic_irq, qemu_irq tx_gpio, qemu_irq rts, qemu_irq cts);

#endif
