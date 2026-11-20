#include "host_protocol.h"

/* CRC-16/CCITT-FALSE, including the sync bytes and type. */
static uint16_t crc16(const uint8_t *bytes, unsigned length)
{
    uint16_t crc = 0xFFFFu;
    for (unsigned i = 0; i < length; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p)
{
    return read_u16(p) | ((uint32_t)read_u16(p + 2) << 16);
}

static void write_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *p, uint32_t value)
{
    write_u16(p, (uint16_t)value);
    write_u16(p + 2, (uint16_t)(value >> 16));
}

int host_decode_command(const uint8_t frame[HOST_COMMAND_SIZE], HostCommand *command)
{
    if (frame[0] != HOST_SYNC_0 || frame[1] != HOST_SYNC_1 ||
        frame[2] != HOST_COMMAND_TYPE || frame[3] > HOST_CMD_EFFORT ||
        read_u16(frame + 10) != crc16(frame, 10)) {
        return 0;
    }

    int16_t effort = (int16_t)read_u16(frame + 8);
    if (effort < -1000 || effort > 1000 || (frame[3] != HOST_CMD_EFFORT && effort != 0)) {
        return 0;
    }

    command->opcode = frame[3];
    command->sample_tick = read_u32(frame + 4);
    command->effort = effort;
    return 1;
}

void host_encode_state(const HostState *state, uint8_t frame[HOST_STATE_SIZE])
{
    frame[0] = HOST_SYNC_0;
    frame[1] = HOST_SYNC_1;
    frame[2] = HOST_STATE_TYPE;
    frame[3] = state->flags;
    write_u32(frame + 4, state->tick);
    write_u32(frame + 8, state->time_us);
    write_u16(frame + 12, state->angle_adc);
    write_u32(frame + 14, (uint32_t)state->encoder);
    write_u16(frame + 18, state->voltage_cv);
    write_u16(frame + 20, (uint16_t)state->applied_effort);
    write_u32(frame + 22, state->last_command_tick);
    write_u16(frame + 26, crc16(frame, 26));
}
