#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include "sensor_decode.h"
#include <stdint.h>

#define DEBUG_UART_RX_QUEUE_SIZE 16u

/* Initialize the debug UART and start interrupt-based one-byte reception. */
HAL_StatusTypeDef DebugUart_Init(UART_HandleTypeDef *huart);

/* Call these from the corresponding global HAL UART callbacks. */
void DebugUart_RxCpltCallback(UART_HandleTypeDef *huart);
void DebugUart_ErrorCallback(UART_HandleTypeDef *huart);

/* Read one raw received byte from the interrupt-fed queue. */
uint8_t DebugUart_ReadByte(uint8_t *byte);
/* Return 1 and clear the flag if the RX queue overflowed since the last check. */
uint8_t DebugUart_TakeRxOverflow(void);

/* ASCII/text output functions for Hercules. */
HAL_StatusTypeDef DebugUart_WriteText(const char *text);
HAL_StatusTypeDef DebugUart_PrintFrameHex(const uint8_t *frame,
                                          uint16_t length);
HAL_StatusTypeDef DebugUart_PrintSensor04(const SensorData04 *data);
HAL_StatusTypeDef DebugUart_PrintSensor06(const SensorData06 *data);

#endif /* DEBUG_UART_H */
