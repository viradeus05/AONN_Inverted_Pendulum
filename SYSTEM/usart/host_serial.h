#ifndef HOST_SERIAL_H
#define HOST_SERIAL_H

#include "host_protocol.h"

void host_serial_init(uint32_t baud);

/* Call these from one context. A partial command is kept for the next poll. */
int host_read_command(HostCommand *command);
void host_send_state(const HostState *state);

#endif
