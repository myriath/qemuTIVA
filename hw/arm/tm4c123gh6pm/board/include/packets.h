#ifndef CYBOT_FIELD_PACKETS_H_
#define CYBOT_FIELD_PACKETS_H_

#include <stdint.h>

#define ID_EXIT                 0x00
#define BYTES_EXIT              1
#define ID_RETRIEVE_SERVO       0x01
#define BYTES_RETRIEVE_SERVO    0
#define ID_SERVO_RESPONSE       0x81
#define BYTES_SERVO_RESPONSE    sizeof(struct ServoResponse)

typedef struct Packet {
    uint8_t ID;
    uint8_t *data;
} Packet;

struct ServoResponse {
    uint8_t direction;
};

#endif
