#ifndef T_PROTOCOL_SENSOR_H
#define T_PROTOCOL_SENSOR_H

#include "main.h"
#include <stdint.h>

#define T_PROTOCOL_04_DATA_SIZE     14u
#define T_PROTOCOL_06_DATA_SIZE     72u
#define T_PROTOCOL_04_FRAME_SIZE    (T_PROTOCOL_04_DATA_SIZE + 5u)
#define T_PROTOCOL_06_FRAME_SIZE    (T_PROTOCOL_06_DATA_SIZE + 5u)
#define T_PROTOCOL_MAX_FRAME_SIZE   T_PROTOCOL_06_FRAME_SIZE
#define T_PROTOCOL_CTRL_CODE_04     0x04u
#define T_PROTOCOL_CTRL_CODE_06     0x06u
#define T_PROTOCOL_LENGTH_04        (T_PROTOCOL_04_DATA_SIZE + 1u)
#define T_PROTOCOL_LENGTH_06        (T_PROTOCOL_06_DATA_SIZE + 1u)
#define T_PROTOCOL_RX_TIMEOUT_MS    1000u

#ifndef T_PROTOCOL_PORT
#define T_PROTOCOL_PORT             GPIOC
#endif
#ifndef T_PROTOCOL_PIN
#define T_PROTOCOL_PIN              GPIO_PIN_5
#endif

typedef enum
{
    T_PROTOCOL_OK = 0,
    T_PROTOCOL_ERROR_ARGUMENT,
    T_PROTOCOL_ERROR_BUSY,
    T_PROTOCOL_ERROR_UART,
    T_PROTOCOL_ERROR_TIMEOUT,
    T_PROTOCOL_ERROR_FRAME
} T_ProtocolStatus;

typedef struct
{
    uint8_t bytes[T_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t length;// do dai cua toan bo frame
} T_ProtocolFrame;

T_ProtocolStatus T_ProtocolSensor_Read04(UART_HandleTypeDef *huart, T_ProtocolFrame *frame);
T_ProtocolStatus T_ProtocolSensor_Read06(UART_HandleTypeDef *huart, T_ProtocolFrame *frame);

T_ProtocolStatus T_ProtocolSensor_Validate04(const uint8_t *frame, uint16_t length);
T_ProtocolStatus T_ProtocolSensor_Validate06(const uint8_t *frame, uint16_t length);

#endif 
