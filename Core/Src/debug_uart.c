#include "debug_uart.h"
#include <stddef.h>
#include <string.h>

#define DEBUG_UART_TX_TIMEOUT_MS 1000u

static UART_HandleTypeDef *s_debug_uart;
static uint8_t s_rx_byte;
static uint8_t s_rx_queue[DEBUG_UART_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_rx_overflow;

static HAL_StatusTypeDef DebugUart_WriteBytes(const uint8_t *data,  uint16_t length)
{
    if ((s_debug_uart == NULL) || ((data == NULL) && (length != 0u)))
    {
        return HAL_ERROR;
    }

    if (length == 0u)
    {
        return HAL_OK;
    }

    return HAL_UART_Transmit(s_debug_uart, (uint8_t *)data,length, DEBUG_UART_TX_TIMEOUT_MS);
}

static HAL_StatusTypeDef DebugUart_PrintUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0u;
    uint8_t i;

    do
    {
        digits[count++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while ((value != 0u) && (count < sizeof(digits)));

    for (i = 0u; i < (count / 2u); ++i)
    {
        char temp = digits[i];
        digits[i] = digits[count - 1u - i];
        digits[count - 1u - i] = temp;
    }

    return DebugUart_WriteBytes((const uint8_t *)digits, count);
}

static HAL_StatusTypeDef DebugUart_PrintSigned(int32_t value)
{
    uint32_t magnitude;
    HAL_StatusTypeDef status;

    if (value < 0)
    {
        status = DebugUart_WriteText("-");
        if (status != HAL_OK)
        {
            return status;
        }
        /* Unsigned subtraction also handles INT32_MIN without signed overflow. */
        magnitude = 0u - (uint32_t)value;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    return DebugUart_PrintUnsigned(magnitude);
}

static HAL_StatusTypeDef DebugUart_PrintFlag(const char *label, uint8_t enabled)
{
    HAL_StatusTypeDef status = DebugUart_WriteText(label);
    if (status != HAL_OK)
    {
        return status;
    }

    return DebugUart_WriteText(enabled ? "Yes\r\n" : "No\r\n");
}

HAL_StatusTypeDef DebugUart_Init(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return HAL_ERROR;
    }

    s_debug_uart = huart;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_overflow = 0u;

    return HAL_UART_Receive_IT(s_debug_uart, &s_rx_byte, 1u);
}

void DebugUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t byte;

    if ((s_debug_uart == NULL) || (huart != s_debug_uart))
    {
        return;
    }

    byte = s_rx_byte;
    {
        uint8_t next = (uint8_t)((s_rx_head + 1u) % DEBUG_UART_RX_QUEUE_SIZE);
        if (next != s_rx_tail)
        {
            s_rx_queue[s_rx_head] = byte;
            s_rx_head = next;
        }
        else
        {
            s_rx_overflow = 1u;
        }
    }

    /* Keep receiving raw bytes; command interpretation happens in foreground. */
    (void)HAL_UART_Receive_IT(s_debug_uart, &s_rx_byte, 1u);
}

void DebugUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_debug_uart != NULL) && (huart == s_debug_uart))
    {
        /*
         * ORE/RTO ends the current HAL reception and leaves RX ready, so a
         * new one-byte reception is required.  FE/NE can be reported while
         * the current reception is still active; calling Receive_IT in that
         * state only returns HAL_BUSY and must not replace the active receive.
         */
        if (s_debug_uart->RxState == HAL_UART_STATE_READY)
        {
            (void)HAL_UART_Receive_IT(s_debug_uart, &s_rx_byte, 1u);
        }
    }
}

uint8_t DebugUart_ReadByte(uint8_t *byte)
{
    uint8_t head;
    uint8_t tail;

    if (byte == NULL)
    {
        return 0u;
    }

    tail = s_rx_tail;
    head = s_rx_head;
    if (tail == head)
    {
        return 0u;
    }

    *byte = s_rx_queue[tail];
    s_rx_tail = (uint8_t)((tail + 1u) % DEBUG_UART_RX_QUEUE_SIZE);
    return 1u;
}

uint8_t DebugUart_TakeRxOverflow(void)
{
    uint8_t overflow = s_rx_overflow;
    s_rx_overflow = 0u;
    return overflow;
}

HAL_StatusTypeDef DebugUart_WriteText(const char *text)
{
    if (text == NULL)
    {
        return HAL_ERROR;
    }

    return DebugUart_WriteBytes((const uint8_t *)text, (uint16_t)strlen(text));
}

HAL_StatusTypeDef DebugUart_PrintFrameHex(const uint8_t *frame,
                                          uint16_t length)
{
    static const char hex[] = "0123456789ABCDEF";
    uint16_t i;
    HAL_StatusTypeDef status;

    if ((frame == NULL) && (length != 0u))
    {
        return HAL_ERROR;
    }

    status = DebugUart_WriteText("[FRAME HEX] ");
    if (status != HAL_OK)
    {
        return status;
    }

    for (i = 0u; i < length; ++i)
    {
        char pair[3];
        pair[0] = hex[(frame[i] >> 4) & 0x0Fu];
        pair[1] = hex[frame[i] & 0x0Fu];
        pair[2] = ' ';

        status = DebugUart_WriteBytes((const uint8_t *)pair,
                                      (i + 1u < length) ? 3u : 2u);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    return DebugUart_WriteText("\r\n");
}

HAL_StatusTypeDef DebugUart_PrintSensor04(const SensorData04 *data)
{
    HAL_StatusTypeDef status;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

#define PRINT_LABEL_U32(label, value)              \
    do {                                           \
        status = DebugUart_WriteText(label);       \
        if (status != HAL_OK) return status;       \
        status = DebugUart_PrintUnsigned(value);   \
        if (status != HAL_OK) return status;       \
        status = DebugUart_WriteText("\r\n");     \
        if (status != HAL_OK) return status;       \
    } while (0)

    PRINT_LABEL_U32("Major version: ", data->major_version);
    PRINT_LABEL_U32("Sub-version: ", data->sub_version);
    PRINT_LABEL_U32("Signal intensity: ", data->signal_intensity);
    PRINT_LABEL_U32("Signal indications: ", data->signal_indications);

    status = DebugUart_WriteText("Net forward pulses: ");
    if (status != HAL_OK) return status;
    status = DebugUart_PrintSigned(data->net_forward_pulses);
    if (status != HAL_OK) return status;
    status = DebugUart_WriteText("\r\n");
    if (status != HAL_OK) return status;

    PRINT_LABEL_U32("Reverse pulses: ", data->reverse_pulses);
    status = DebugUart_WriteText("Calibration: ");
    if (status != HAL_OK) return status;
    status = DebugUart_WriteText((data->calibration == SENSOR_CALIBRATION_FAIL)
                                 ? "Fail\r\n"
                                 : "Success\r\n");
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Fast sampling: ", data->fast_sampling);
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Sensor removed: ", data->sensor_removed);
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Foreign object detected: ", data->foreign_object_detected);
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Ever calibrated successfully: ",
                                 data->ever_calibrated_successfully);
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Low voltage: ", data->low_voltage);
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Filter mode: ", data->filter_mode);
    if (status != HAL_OK) return status;
    status = DebugUart_PrintFlag("Threshold translation status: ",
                                 data->threshold_translation_status);
    if (status != HAL_OK) return status;

    return HAL_OK;

#undef PRINT_LABEL_U32
}

HAL_StatusTypeDef DebugUart_PrintSensor06(const SensorData06 *data)
{
    static const char hex[] = "0123456789ABCDEF";
    HAL_StatusTypeDef status;
    uint8_t i;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

#define PRINT06_U32(label, value)                \
    do {                                         \
        status = DebugUart_WriteText(label);     \
        if (status != HAL_OK) return status;     \
        status = DebugUart_PrintUnsigned(value); \
        if (status != HAL_OK) return status;     \
        status = DebugUart_WriteText("\r\n"); \
        if (status != HAL_OK) return status;     \
    } while (0)

#define PRINT06_FLAG(label, value)               \
    do {                                         \
        status = DebugUart_PrintFlag(label, value); \
        if (status != HAL_OK) return status;     \
    } while (0)

    PRINT06_U32("Major version: ", data->major_version);
    PRINT06_U32("Sub-version: ", data->sub_version);

    status = DebugUart_WriteText("Net forward pulses: ");
    if (status != HAL_OK) return status;
    status = DebugUart_PrintSigned(data->net_forward_pulses);
    if (status != HAL_OK) return status;
    status = DebugUart_WriteText("\r\n");
    if (status != HAL_OK) return status;

    PRINT06_U32("Reverse pulses: ", data->reverse_pulses);
    PRINT06_U32("Signal intensity 0: ", data->signal_intensity[0]);
    PRINT06_U32("Signal intensity 1: ", data->signal_intensity[1]);
    PRINT06_U32("Signal intensity 2: ", data->signal_intensity[2]);
    PRINT06_U32("Sampling value 0: ", data->sampling_value[0]);
    PRINT06_U32("Sampling value 1: ", data->sampling_value[1]);
    PRINT06_U32("Sampling value 2: ", data->sampling_value[2]);
    PRINT06_U32("Filter number of interferences: ", data->filter_number_of_interferences);
    PRINT06_U32("Filter duration (64 ms units): ", data->filter_duration);
    PRINT06_U32("Total number of resets: ", data->total_number_of_resets);
    PRINT06_U32("Reason for this realignment: ", data->realignment_reason);

    status = DebugUart_WriteText("UID: ");
    if (status != HAL_OK) return status;
    for (i = 0u; i < SENSOR_UID_SIZE; ++i)
    {
        char pair[3];
        pair[0] = hex[(data->uid[i] >> 4) & 0x0Fu];
        pair[1] = hex[data->uid[i] & 0x0Fu];
        pair[2] = (i + 1u < SENSOR_UID_SIZE) ? ' ' : '\r';
        status = DebugUart_WriteBytes((const uint8_t *)pair, 3u);
        if (status != HAL_OK) return status;
    }
    status = DebugUart_WriteText("\n");
    if (status != HAL_OK) return status;

    status = DebugUart_WriteText("Calibration: ");
    if (status != HAL_OK) return status;
    status = DebugUart_WriteText((data->calibration == SENSOR_CALIBRATION_FAIL)
                                 ? "Fail\r\n" : "Success\r\n");
    if (status != HAL_OK) return status;

    PRINT06_FLAG("Fast sampling: ", data->fast_sampling);
    PRINT06_FLAG("Sensor removed: ", data->sensor_removed);
    PRINT06_FLAG("Foreign object detected: ", data->foreign_object_detected);
    PRINT06_FLAG("Currently calibrated successfully: ",
                 data->currently_calibrated_successfully);
    PRINT06_FLAG("Low voltage: ", data->low_voltage);
    PRINT06_FLAG("Channel 0 interfered: ", data->channel_interfered[0]);
    PRINT06_FLAG("Channel 1 interfered: ", data->channel_interfered[1]);
    PRINT06_FLAG("Channel 2 interfered: ", data->channel_interfered[2]);
    PRINT06_FLAG("Program validation error: ", data->program_validation_error);
    PRINT06_FLAG("Filter mode: ", data->filter_mode);
    PRINT06_U32("Minimum signal intensity for meter breakdown: ",
                data->minimum_signal_intensity_for_meter_breakdown);
    PRINT06_U32("Maximum signal intensity: ", data->maximum_signal_intensity);
    PRINT06_U32("Upper limit of filtering time (minutes): ",
                data->upper_limit_of_filtering_time);

    status = DebugUart_WriteText("MCU temperature (C): ");
    if (status != HAL_OK) return status;
    status = DebugUart_PrintSigned(data->mcu_temperature_c);
    if (status != HAL_OK) return status;
    return DebugUart_WriteText("\r\n");

#undef PRINT06_FLAG
#undef PRINT06_U32
}
