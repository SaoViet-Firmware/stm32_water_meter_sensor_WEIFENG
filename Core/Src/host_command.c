#include "host_command.h"
#include "debug_uart.h"

HostCommand HostCommand_Poll(void)
{
    uint8_t byte;

    if (DebugUart_TakeRxOverflow() != 0u)
    {
        return HOST_COMMAND_RX_OVERFLOW;
    }

    if (DebugUart_ReadByte(&byte) == 0u)
    {
        return HOST_COMMAND_NONE;
    }

    switch (byte)
    {
        case 0x04u:
            return HOST_COMMAND_READ_04;

        case 0x06u:
            return HOST_COMMAND_READ_06;

        default:
            return HOST_COMMAND_INVALID;
    }
}
