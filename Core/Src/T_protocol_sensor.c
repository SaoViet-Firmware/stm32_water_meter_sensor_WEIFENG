#include "T_protocol_sensor.h"
#include <stddef.h>

#define T_PROTOCOL_T1_UNIT_MS 125u
#define T_PROTOCOL_T_IDLE_LOW_MS 10u

static void T_ProtocolSensor_ConfigureTAsOutput(void)// dat trang thai chan T thanh gpio output
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = T_PROTOCOL_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(T_PROTOCOL_PORT, &gpio);
}

static void T_ProtocolSensor_ConfigureTAsUartRx(void)// dat trang thai chan T thanh uart rx
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = T_PROTOCOL_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(T_PROTOCOL_PORT, &gpio);
}

static void T_ProtocolSensor_SetTIdleLow(void)// tra chan T ve trang thai nghi muc thap
{
    /* Set the output latch low before changing the pin mode to avoid a high glitch. */
    HAL_GPIO_WritePin(T_PROTOCOL_PORT, T_PROTOCOL_PIN, GPIO_PIN_RESET);
    T_ProtocolSensor_ConfigureTAsOutput();
    HAL_GPIO_WritePin(T_PROTOCOL_PORT, T_PROTOCOL_PIN, GPIO_PIN_RESET);
}

static uint8_t T_ProtocolSensor_Checksum(const uint8_t *frame, uint16_t frame_size)// tinh checksum cua frame
{
    uint16_t sum = 0u;
    uint16_t i;

    for (i = 2u; i < (frame_size - 2u); ++i)
    {
        sum = (uint16_t)(sum + frame[i]);
    }
    return (uint8_t)(sum & 0xFFu);
}

static T_ProtocolStatus T_ProtocolSensor_Validate(const uint8_t *frame, uint16_t frame_size,uint8_t expected_length,uint8_t expected_control_code)// kiem tra frame co hop le hay khong
{
    if (frame == NULL)
    {
        return T_PROTOCOL_ERROR_ARGUMENT;
    }

    if ((frame_size < 5u) ||
        (frame[0] != 0x68u) ||
        (frame[1] != expected_length) ||
        (frame[2] != expected_control_code) ||
        (frame[frame_size - 2u] != T_ProtocolSensor_Checksum(frame, frame_size)) ||
        (frame[frame_size - 1u] != 0x16u))
    {
        return T_PROTOCOL_ERROR_FRAME;
    }
    return T_PROTOCOL_OK;
}

T_ProtocolStatus T_ProtocolSensor_Validate04(const uint8_t *frame, uint16_t length)// kiem tra frame 04 co hop le hay khong
{
    if (length != T_PROTOCOL_04_FRAME_SIZE)
    {
        return T_PROTOCOL_ERROR_FRAME;
    }
    return T_ProtocolSensor_Validate(frame, length, T_PROTOCOL_LENGTH_04, T_PROTOCOL_CTRL_CODE_04);
}

T_ProtocolStatus T_ProtocolSensor_Validate06(const uint8_t *frame, uint16_t length)// kiem tra frame 06 co hop le hay khong
{
    if (length != T_PROTOCOL_06_FRAME_SIZE)
    {
        return T_PROTOCOL_ERROR_FRAME;
    }
    return T_ProtocolSensor_Validate(frame, length, T_PROTOCOL_LENGTH_06, T_PROTOCOL_CTRL_CODE_06);
}

static T_ProtocolStatus T_ProtocolSensor_Read(UART_HandleTypeDef *huart, uint8_t control_code, uint16_t frame_size, T_ProtocolFrame *frame)// doc frame tu sensor
{
    HAL_StatusTypeDef hal_status;
    uint32_t start_tick;// thoi diem bat dau nhan
    uint32_t elapsed;// thoi gian da su dung
    uint32_t remaining;// thoi gian con lai de nhan
    uint8_t byte;// byte nhan duoc tu sensor

    if ((huart == NULL) || (frame == NULL))
    {
        return T_PROTOCOL_ERROR_ARGUMENT;
    }
    if ((huart->Instance != USART3) || (frame_size > T_PROTOCOL_MAX_FRAME_SIZE))
    {
        return T_PROTOCOL_ERROR_UART;
    }

    frame->length = 0u;
    T_ProtocolSensor_SetTIdleLow();
    HAL_Delay(T_PROTOCOL_T_IDLE_LOW_MS);
    HAL_GPIO_WritePin(T_PROTOCOL_PORT, T_PROTOCOL_PIN, GPIO_PIN_SET);
    HAL_Delay(T_PROTOCOL_T1_UNIT_MS * control_code);
    HAL_GPIO_WritePin(T_PROTOCOL_PORT, T_PROTOCOL_PIN, GPIO_PIN_RESET);
    T_ProtocolSensor_ConfigureTAsUartRx();

    start_tick = HAL_GetTick();
    do
    {
        elapsed = (uint32_t)(HAL_GetTick() - start_tick);
        if (elapsed >= T_PROTOCOL_RX_TIMEOUT_MS)
        {
            T_ProtocolSensor_SetTIdleLow();
            return T_PROTOCOL_ERROR_TIMEOUT;
        }

        remaining = T_PROTOCOL_RX_TIMEOUT_MS - elapsed;
        hal_status = HAL_UART_Receive(huart, &byte, 1u, remaining);
        if (hal_status != HAL_OK)
        {
            T_ProtocolSensor_SetTIdleLow();
            return (hal_status == HAL_TIMEOUT)
                ? T_PROTOCOL_ERROR_TIMEOUT
                : T_PROTOCOL_ERROR_UART;
        }
    } while (byte != 0x68u);

    frame->bytes[0] = byte;
    elapsed = (uint32_t)(HAL_GetTick() - start_tick);
    if (elapsed >= T_PROTOCOL_RX_TIMEOUT_MS)
    {
        T_ProtocolSensor_SetTIdleLow();
        return T_PROTOCOL_ERROR_TIMEOUT;
    }

    remaining = T_PROTOCOL_RX_TIMEOUT_MS - elapsed;
    hal_status = HAL_UART_Receive(huart, &frame->bytes[1], (uint16_t)(frame_size - 1u), remaining);
    if (hal_status != HAL_OK)
    {
        T_ProtocolSensor_SetTIdleLow();
        return (hal_status == HAL_TIMEOUT)
            ? T_PROTOCOL_ERROR_TIMEOUT
            : T_PROTOCOL_ERROR_UART;
    }

    frame->length = frame_size;
    T_ProtocolSensor_SetTIdleLow();
    return T_PROTOCOL_OK;
}

T_ProtocolStatus T_ProtocolSensor_Read04(UART_HandleTypeDef *huart, T_ProtocolFrame *frame)// doc frame 04 tu sensor
{
    T_ProtocolStatus status = T_ProtocolSensor_Read(huart, T_PROTOCOL_CTRL_CODE_04, T_PROTOCOL_04_FRAME_SIZE, frame);
    if (status != T_PROTOCOL_OK)
    {
        return status;
    }
    return T_ProtocolSensor_Validate04(frame->bytes, frame->length);
}

T_ProtocolStatus T_ProtocolSensor_Read06(UART_HandleTypeDef *huart, T_ProtocolFrame *frame)// doc frame 06 tu sensor
{
    T_ProtocolStatus status = T_ProtocolSensor_Read(huart, T_PROTOCOL_CTRL_CODE_06, T_PROTOCOL_06_FRAME_SIZE, frame);
    if (status != T_PROTOCOL_OK)
    {
        return status;
    }
    return T_ProtocolSensor_Validate06(frame->bytes, frame->length);
}
