#ifndef HOST_CONTROL_H
#define HOST_CONTROL_H

#include "host_protocol.h"

/* TIM4 is set to 10000 with the cart at the left end on reset. */
#define HOST_RAIL_MIN 5900
#define HOST_RAIL_MAX 9900
#define HOST_MIN_VOLTAGE_CV 700
#define HOST_TIMEOUT_TICKS 10u

typedef struct {
    uint32_t last_command_tick;
    uint32_t oldest_sample;  /* Earliest state still eligible for a command. */
    int16_t applied_effort;
    uint8_t flags;
} HostControl;

void host_control_init(HostControl *control);
void host_control_tick(HostControl *control, uint32_t tick, int32_t encoder,
                       uint16_t voltage_cv, int user_stop);
void host_control_command(HostControl *control, uint32_t tick, const HostCommand *command);

#endif
