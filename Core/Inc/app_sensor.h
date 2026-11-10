#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include "main.h"

HAL_StatusTypeDef AppSensor_Init(UART_HandleTypeDef *sensor_uart, UART_HandleTypeDef *debug_uart);

void AppSensor_Task(void);

#endif /* APP_SENSOR_H */
