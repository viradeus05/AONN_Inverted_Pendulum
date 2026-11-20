#include "host_control.h"

#define HOST_INTERLOCKS (HOST_FLAG_LOCAL_STOP | HOST_FLAG_RAIL_LIMIT | HOST_FLAG_LOW_VOLTAGE)

static void disarm(HostControl *control, uint32_t oldest_sample)
{
    control->flags &= (uint8_t)~HOST_FLAG_ARMED;
    control->applied_effort = 0;
    control->oldest_sample = oldest_sample;
}

void host_control_init(HostControl *control)
{
    control->flags = 0;
    control->last_command_tick = UINT32_MAX;
    disarm(control, 1);  /* First published tick after reset. */
}

void host_control_tick(HostControl *control, uint32_t tick, int32_t encoder,
                       uint16_t voltage_cv, int user_stop)
{
    control->flags &= (uint8_t)~HOST_INTERLOCKS;
    if (user_stop)
        control->flags |= HOST_FLAG_LOCAL_STOP;
    if (encoder <= HOST_RAIL_MIN || encoder >= HOST_RAIL_MAX)
        control->flags |= HOST_FLAG_RAIL_LIMIT;
    if (voltage_cv < HOST_MIN_VOLTAGE_CV)
        control->flags |= HOST_FLAG_LOW_VOLTAGE;

    if (control->flags & HOST_INTERLOCKS) {
        disarm(control, tick + 1u);  /* Require a sample taken after the fault. */
    } else if ((control->flags & HOST_FLAG_ARMED) &&
               tick - control->last_command_tick >= HOST_TIMEOUT_TICKS) {
        control->flags |= HOST_FLAG_TIMEOUT;
        disarm(control, tick);
    } else if (tick - control->oldest_sample >= HOST_TIMEOUT_TICKS) {
        control->oldest_sample = tick - (HOST_TIMEOUT_TICKS - 1u);
    }
}

void host_control_command(HostControl *control, uint32_t tick, const HostCommand *command)
{
    uint32_t age = tick - command->sample_tick;
    /* The tick has already stopped the motor and invalidated pending commands. */
    if (control->flags & HOST_INTERLOCKS)
        return;
    if (command->opcode == HOST_CMD_DISARM) {
        /* Disarm even when the host's last observation is old. */
        disarm(control, tick);
        return;
    }
    if (age == 0 || age >= HOST_TIMEOUT_TICKS || age > tick - control->oldest_sample)
        return;

    if (command->opcode == HOST_CMD_ARM && command->effort == 0 &&
        !(control->flags & HOST_FLAG_ARMED)) {
        control->flags = HOST_FLAG_ARMED;
        /* Only an observation published while armed may authorize motion. */
        control->oldest_sample = tick;
    } else if (command->opcode == HOST_CMD_EFFORT && (control->flags & HOST_FLAG_ARMED) &&
               command->effort >= -1000 && command->effort <= 1000) {
        control->applied_effort = command->effort;
        control->oldest_sample = command->sample_tick + 1u;
    } else {
        return;
    }
    control->last_command_tick = command->sample_tick;
}
