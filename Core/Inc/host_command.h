#ifndef HOST_COMMAND_H
#define HOST_COMMAND_H

#include <stdint.h>

typedef enum
{
    HOST_COMMAND_NONE = 0,
    HOST_COMMAND_READ_04,
    HOST_COMMAND_READ_06,
    HOST_COMMAND_INVALID,
    HOST_COMMAND_RX_OVERFLOW
} HostCommand;

HostCommand HostCommand_Poll(void);

#endif /* HOST_COMMAND_H */
