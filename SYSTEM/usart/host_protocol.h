#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H

#include <stdint.h>

#define HOST_SYNC_0       0xA5u
#define HOST_SYNC_1       0x5Au
#define HOST_COMMAND_TYPE 0x11u
#define HOST_STATE_TYPE   0x12u
#define HOST_COMMAND_SIZE 12u
#define HOST_STATE_SIZE   28u

enum {
    HOST_CMD_DISARM = 0,
    HOST_CMD_ARM    = 1,
    HOST_CMD_EFFORT = 2
};

enum {
    HOST_FLAG_ARMED       = 1u << 0,
    HOST_FLAG_LOCAL_STOP  = 1u << 1,
    HOST_FLAG_RAIL_LIMIT  = 1u << 2,
    HOST_FLAG_LOW_VOLTAGE = 1u << 3,
    HOST_FLAG_TIMEOUT     = 1u << 4
};

/* The wire codec writes individual fields. Struct padding stays local. */
typedef struct {
    uint8_t opcode;
    uint32_t sample_tick;
    int16_t effort;              /* Signed thousandths of maximum drive. */
} HostCommand;

typedef struct {
    uint8_t flags;
    uint32_t tick;
    uint32_t time_us;
    uint16_t angle_adc;
    int32_t encoder;
    uint16_t voltage_cv;
    int16_t applied_effort;
    uint32_t last_command_tick;
} HostState;

int host_decode_command(const uint8_t frame[HOST_COMMAND_SIZE], HostCommand *command);
void host_encode_state(const HostState *state, uint8_t frame[HOST_STATE_SIZE]);

#endif
