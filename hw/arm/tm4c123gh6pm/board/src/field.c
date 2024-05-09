#include "hw/arm/tm4c123gh6pm/board/include/field.h"

DeviceState *create_field_device(Clock *clk)
{
    DeviceState *dev = qdev_new(TYPE_CYBOT_FIELD);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_connect_clock_in(dev, "clk", clk);

    sysbus_realize_and_unref(sbd, &error_fatal);

    return dev;
}

static void reload(FieldState *s, bool reset)
{
    uint64_t tick;
    if (reset) {
        tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    } else {
        tick = s->tick;
    }

    tick += s->timeout;
    timer_mod(s->timer, tick);
    s->tick = tick;
}

static void respond(Packet *packet, int data_bytes)
{
    wifi_send(packet->ID, SERVER_INSTRUCTOR);
    for (int i = 0; i < data_bytes; i++) {
        wifi_send(packet->data[i], SERVER_INSTRUCTOR);
    }
}

static void parse_packet(FieldState *s)
{
    Packet response;
    Packet *packet = &s->packet;
    int packet_size;

    switch (packet->ID) {
        case ID_EXIT:
            printf("Instructor Exit: %d\n", packet->data[0]);
            s->exit = true;
            return;
        case ID_RETRIEVE_SERVO:
            // TODO: might need endianness stuff
            printf("Responding!\n");
            response.ID = ID_SERVO_RESPONSE;
            struct ServoResponse response_data = {
                .direction = (uint8_t) s->last_servo
            };
            response.data = (uint8_t *) &response_data;
            packet_size = BYTES_SERVO_RESPONSE;
            respond(&response, packet_size);
            return;
        default:
            return;
    }
}

static void new_packet(FieldState *s, char id)
{
    s->read_count = 0;
    s->packet.ID = id;
    switch (id) {
        case ID_EXIT:
            s->bytes_to_read = BYTES_EXIT;
            break;
        case ID_RETRIEVE_SERVO:
            s->bytes_to_read = BYTES_RETRIEVE_SERVO;
            break;
        default:
            return;
    }
    free(s->packet.data);
    s->packet.data = malloc(s->bytes_to_read * sizeof(uint8_t));
    if (s->packet.data == NULL) {
        perror("Malloc failed");
        exit(EXIT_FAILURE);
    }
    if (s->bytes_to_read == 0) {
        parse_packet(s);
        return;
    }
    s->reading = true;
}

static void tick(void *opaque)
{
    FieldState *s = opaque;

    char data = wifi_recv(SERVER_INSTRUCTOR);
    if (data != -1) {
        if (!s->reading) {
            new_packet(s, data);
        } else {
            s->packet.data[s->read_count++] = data;
            if (s->read_count == s->bytes_to_read) {
                s->reading = false;
                parse_packet(s);
            }
        }
    }

    if (!s->exit) {
        reload(s, false);
    }
}

static void servo_handler(void *opaque, int n, int level)
{
    FieldState *s = opaque;
    s->last_servo = level;
}

static void field_init(Object *obj)
{
    FieldState *s = CYBOT_FIELD(obj);
    DeviceState *dev = DEVICE(obj);
    
    qdev_init_gpio_in(dev, servo_handler, 1);
    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
}

static void instructor_connect(void *opaque)
{
    FieldState *s = opaque;
    // Start timer for reading data from server
    reload(s, true);
}

static void field_realize(DeviceState *dev, Error **errp)
{
    FieldState *s = CYBOT_FIELD(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "cybot-field: clk must be connected");
        return;
    }

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tick, s);

    create_server(SERVER_INSTRUCTOR, s->port, s, instructor_connect);
}

static Property field_properties[] = {
    DEFINE_PROP_UINT32("port", FieldState, port, 10001),
    DEFINE_PROP_UINT64("timeout", FieldState, timeout, 10000000), // 10 ms timeout
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_field = {
    .name = TYPE_CYBOT_FIELD,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(last_servo, FieldState),
        VMSTATE_UINT64(tick, FieldState),
        VMSTATE_CLOCK(clk, FieldState),
        VMSTATE_END_OF_LIST()
    }
};

static void field_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    /* No state to reset or migrate */
    device_class_set_props(dc, field_properties);
    dc->realize = field_realize;
    dc->vmsd = &vmstate_field;

    /* Reason: Needs to be wired up to work */
    dc->user_creatable = false;
}

static const TypeInfo field_info = {
   .name = TYPE_CYBOT_FIELD,
   .parent = TYPE_SYS_BUS_DEVICE,
   .instance_size = sizeof(FieldState),
   .instance_init = field_init,
   .class_init = field_class_init,
};

static void field_register_types(void)
{
    type_register_static(&field_info);
}
type_init(field_register_types)
