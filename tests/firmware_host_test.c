#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "host_control.h"
#include "host_protocol.h"

static void test_protocol(void)
{
    /* Fixed wire vectors also exercised by the Python client tests. */
    uint8_t command_wire[] = {
        0xa5, 0x5a, 0x11, 0x02, 0x78, 0x56, 0x34, 0x12, 0x06, 0xff, 0x6f, 0xe8
    };
    const uint8_t state_wire[] = {
        0xa5, 0x5a, 0x12, 0x01, 0x78, 0x56, 0x34, 0x12,
        0xff, 0xde, 0xbc, 0xfa, 0x1b, 0x0c, 0xc7, 0xcf,
        0xff, 0xff, 0xc4, 0x04, 0x06, 0xff, 0x77, 0x56,
        0x34, 0x12, 0x97, 0xa0
    };
    HostCommand command;
    uint8_t encoded[HOST_STATE_SIZE];
    HostState state = {
        .flags = HOST_FLAG_ARMED,
        .tick = 0x12345678u, .time_us = 0xfabcdeffu,
        .angle_adc = 3099, .encoder = -12345, .voltage_cv = 1220,
        .applied_effort = -250, .last_command_tick = 0x12345677u
    };

    assert(sizeof(command_wire) == HOST_COMMAND_SIZE);
    assert(host_decode_command(command_wire, &command));
    assert(command.opcode == HOST_CMD_EFFORT && command.sample_tick == state.tick);
    assert(command.effort == -250);
    for (unsigned i = 0; i < sizeof(command_wire); ++i) {
        command_wire[i] ^= 1;
        assert(!host_decode_command(command_wire, &command));
        command_wire[i] ^= 1;
    }
    host_encode_state(&state, encoded);
    assert(sizeof(state_wire) == sizeof(encoded));
    assert(memcmp(state_wire, encoded, sizeof(encoded)) == 0);
}

static void send_command(HostControl *control, uint32_t now, uint8_t opcode,
                         uint32_t sample, int16_t effort)
{
    HostCommand command = {.opcode = opcode, .sample_tick = sample, .effort = effort};
    host_control_command(control, now, &command);
}

static void assert_stopped(const HostControl *control)
{
    assert(!(control->flags & HOST_FLAG_ARMED));
    assert(control->applied_effort == 0);
}

static void tick_at_center(HostControl *control, uint32_t now)
{
    host_control_tick(control, now, 8000, 1200, 0);
}

static void test_authority_and_timeout(void)
{
    HostControl control;
    host_control_init(&control);
    tick_at_center(&control, 1);
    send_command(&control, 2, HOST_CMD_EFFORT, 1, 700);
    assert_stopped(&control);
    tick_at_center(&control, 3);
    send_command(&control, 3, HOST_CMD_ARM, 1, 0);
    assert(control.flags & HOST_FLAG_ARMED);
    send_command(&control, 3, HOST_CMD_EFFORT, 2, 800); /* queued before arm acknowledgement */
    assert(control.applied_effort == 0);
    tick_at_center(&control, 4);
    send_command(&control, 4, HOST_CMD_EFFORT, 3, -250);
    assert(control.applied_effort == -250 && control.last_command_tick == 3);

    send_command(&control, 4, HOST_CMD_EFFORT, 3, 900); /* duplicate */
    send_command(&control, 4, HOST_CMD_EFFORT, 2, 900); /* out of order */
    send_command(&control, 4, HOST_CMD_EFFORT, 5, 900); /* future */
    assert(control.applied_effort == -250 && control.last_command_tick == 3);
    tick_at_center(&control, 12);
    assert(control.flags & HOST_FLAG_ARMED);
    tick_at_center(&control, 13);
    assert_stopped(&control);
    assert(control.flags & HOST_FLAG_TIMEOUT);
    send_command(&control, 13, HOST_CMD_ARM, 3, 0); /* stale */
    assert_stopped(&control);
    tick_at_center(&control, 15);
    send_command(&control, 15, HOST_CMD_ARM, 14, 0);
    assert(control.flags & HOST_FLAG_ARMED);
    assert(!(control.flags & HOST_FLAG_TIMEOUT));
    send_command(&control, 15, HOST_CMD_DISARM, 1, 0);
    assert_stopped(&control);
    send_command(&control, 15, HOST_CMD_ARM, 14, 0);
    assert_stopped(&control);
}

static void test_local_cutoff(int32_t encoder, uint16_t voltage, int pressed,
                              uint8_t expected_flag)
{
    HostControl control;
    host_control_init(&control);
    tick_at_center(&control, 1);
    tick_at_center(&control, 2);
    send_command(&control, 2, HOST_CMD_ARM, 1, 0);
    tick_at_center(&control, 3);
    send_command(&control, 3, HOST_CMD_EFFORT, 2, 1000);
    assert(control.applied_effort == 1000);
    host_control_tick(&control, 4, encoder, voltage, pressed);
    assert_stopped(&control);
    assert(control.flags & expected_flag);
    send_command(&control, 4, HOST_CMD_ARM, 3, 0);
    assert_stopped(&control);
    tick_at_center(&control, 5);
    send_command(&control, 5, HOST_CMD_ARM, 3, 0); /* pre-fault sample */
    assert_stopped(&control);
    tick_at_center(&control, 7);
    send_command(&control, 7, HOST_CMD_ARM, 6, 0);
    assert(control.flags & HOST_FLAG_ARMED);
}

static void test_tick_rollover(void)
{
    HostControl control;
    host_control_init(&control);
    tick_at_center(&control, UINT32_MAX - 2u);
    send_command(&control, UINT32_MAX - 2u, HOST_CMD_ARM, UINT32_MAX - 3u, 0);
    tick_at_center(&control, 0);
    send_command(&control, 0, HOST_CMD_EFFORT, UINT32_MAX, -1000);
    assert(control.applied_effort == -1000);
    tick_at_center(&control, 1);
    send_command(&control, 1, HOST_CMD_EFFORT, 0, 1000);
    assert(control.applied_effort == 1000 && control.last_command_tick == 0);
    tick_at_center(&control, 10);
    assert_stopped(&control);
    assert(control.flags & HOST_FLAG_TIMEOUT);
}

int main(void)
{
    test_protocol();
    test_authority_and_timeout();
    test_local_cutoff(5900, 1200, 0, HOST_FLAG_RAIL_LIMIT);
    test_local_cutoff(9900, 1200, 0, HOST_FLAG_RAIL_LIMIT);
    test_local_cutoff(8000, 699, 0, HOST_FLAG_LOW_VOLTAGE);
    test_local_cutoff(8000, 1200, 1, HOST_FLAG_LOCAL_STOP);
    test_tick_rollover();
    return 0;
}
