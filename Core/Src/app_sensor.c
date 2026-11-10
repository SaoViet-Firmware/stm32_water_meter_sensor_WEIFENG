#include "app_sensor.h"

#include "debug_uart.h"
#include "host_command.h"
#include "sensor_decode.h"
#include "T_protocol_sensor.h"

#ifndef APP_SENSOR_AUTO_READ04_PERIOD_MS
#define APP_SENSOR_AUTO_READ04_PERIOD_MS 0u
#endif

#ifndef APP_SENSOR_COUNT_BYTE_ORDER
#define APP_SENSOR_COUNT_BYTE_ORDER SENSOR_BYTE_ORDER_LITTLE_ENDIAN
#endif

static UART_HandleTypeDef *s_sensor_uart;
static uint8_t s_initialized;
static uint32_t s_last_auto_read04_tick;

static void AppSensor_PrintProtocolError(T_ProtocolStatus status)
{
    switch (status)
    {
        case T_PROTOCOL_ERROR_TIMEOUT:
            (void)DebugUart_WriteText("Sensor response timeout\r\n");
            break;

        case T_PROTOCOL_ERROR_FRAME:
            (void)DebugUart_WriteText("Invalid sensor frame (check HEX frame above)\r\n");
            break;

        case T_PROTOCOL_ERROR_UART:
            (void)DebugUart_WriteText("Sensor UART error/configuration mismatch\r\n");
            break;

        default:
            (void)DebugUart_WriteText("Sensor transaction failed\r\n");
            break;
    }
}

static void AppSensor_HandleRead04(void)
{
    T_ProtocolFrame frame;
    T_ProtocolStatus protocol_status;
    SensorData04 decoded;
    SensorDecodeStatus decode_status;

    frame.length = 0u;
    protocol_status = T_ProtocolSensor_Read04(s_sensor_uart, &frame);

    if (frame.length != 0u)
    {
        (void)DebugUart_PrintFrameHex(frame.bytes, frame.length);
    }

    if (protocol_status != T_PROTOCOL_OK)
    {
        AppSensor_PrintProtocolError(protocol_status);
        return;
    }

    decode_status = SensorDecode_04(&frame.bytes[3], T_PROTOCOL_04_DATA_SIZE, APP_SENSOR_COUNT_BYTE_ORDER, &decoded);
    if (decode_status != SENSOR_DECODE_OK)
    {
        (void)DebugUart_WriteText("Could not decode Control Code 0x04 data\r\n");
        return;
    }

    (void)DebugUart_WriteText("[CONTROL CODE 0x04 DATA]\r\n");
    (void)DebugUart_PrintSensor04(&decoded);
}

static void AppSensor_HandleRead06(void)
{
    T_ProtocolFrame frame;
    T_ProtocolStatus protocol_status;
    SensorData06 decoded;
    SensorDecodeStatus decode_status;

    frame.length = 0u;
    protocol_status = T_ProtocolSensor_Read06(s_sensor_uart, &frame);

    if (frame.length != 0u)
    {
        (void)DebugUart_PrintFrameHex(frame.bytes, frame.length);
    }

    if (protocol_status != T_PROTOCOL_OK)
    {
        AppSensor_PrintProtocolError(protocol_status);
        return;
    }

    decode_status = SensorDecode_06(&frame.bytes[3],
                                    T_PROTOCOL_06_DATA_SIZE,
                                    APP_SENSOR_COUNT_BYTE_ORDER,
                                    &decoded);
    if (decode_status != SENSOR_DECODE_OK)
    {
        (void)DebugUart_WriteText("Could not decode Control Code 0x06 data\r\n");
        return;
    }

    (void)DebugUart_WriteText("[CONTROL CODE 0x06 DATA]\r\n");
    (void)DebugUart_PrintSensor06(&decoded);
}

HAL_StatusTypeDef AppSensor_Init(UART_HandleTypeDef *sensor_uart, UART_HandleTypeDef *debug_uart)
{
    HAL_StatusTypeDef status;

    if ((sensor_uart == NULL) || (sensor_uart->Instance != USART3) ||
        (debug_uart == NULL) || (debug_uart->Instance != USART2))
    {
        return HAL_ERROR;
    }

    s_sensor_uart = sensor_uart;
    status = DebugUart_Init(debug_uart);
    if (status != HAL_OK)
    {
        s_sensor_uart = NULL;
        return status;
    }

    s_initialized = 1u;
    s_last_auto_read04_tick = HAL_GetTick();
    (void)DebugUart_WriteText(
        "Sensor app ready. Send one HEX byte: 04 or 06.\r\n");
    return HAL_OK;
}

void AppSensor_Task(void)
{
    HostCommand command;

    if ((s_initialized == 0u) || (s_sensor_uart == NULL))
    {
        return;
    }

    command = HostCommand_Poll();
    switch (command)
    {
        case HOST_COMMAND_READ_04:
            AppSensor_HandleRead04();
            break;

        case HOST_COMMAND_READ_06:
            AppSensor_HandleRead06();
            break;

        case HOST_COMMAND_INVALID:
            (void)DebugUart_WriteText(
                "Unknown command. Send one HEX byte: 04 or 06.\r\n");
            break;

        case HOST_COMMAND_RX_OVERFLOW:
            (void)DebugUart_WriteText("Host command RX queue overflow; command discarded.\r\n");
            break;

        case HOST_COMMAND_NONE:
            if ((APP_SENSOR_AUTO_READ04_PERIOD_MS != 0u) &&((uint32_t)(HAL_GetTick() - s_last_auto_read04_tick) >= APP_SENSOR_AUTO_READ04_PERIOD_MS))
            {
                s_last_auto_read04_tick = HAL_GetTick();
                AppSensor_HandleRead04();
            }
            break;

        default:
            break;
    }
}
