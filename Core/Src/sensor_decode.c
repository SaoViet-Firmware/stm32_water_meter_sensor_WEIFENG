#include "sensor_decode.h"
#include <stddef.h>

static uint32_t SensorDecode_ReadU32(const uint8_t *bytes, SensorByteOrder byte_order)
{
    if (byte_order == SENSOR_BYTE_ORDER_LITTLE_ENDIAN)
    {
        return ((uint32_t)bytes[0])       |
               ((uint32_t)bytes[1] << 8)  |
               ((uint32_t)bytes[2] << 16) |
               ((uint32_t)bytes[3] << 24);
    }

    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8)  |
           ((uint32_t)bytes[3]);
}

static uint16_t SensorDecode_ReadU16(const uint8_t *bytes, SensorByteOrder byte_order)
{
    if (byte_order == SENSOR_BYTE_ORDER_LITTLE_ENDIAN)
    {
        return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    }
    return (uint16_t)(((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1]);
}

/* Interpret a 32-bit two's-complement bit pattern without overflowing. */
static int32_t SensorDecode_AsInt32(uint32_t value)
{
    if (value <= 0x7FFFFFFFu)
    {
        return (int32_t)value;
    }
    return (int32_t)(-1 - (int32_t)(0xFFFFFFFFu - value));
}

SensorDecodeStatus SensorDecode_04(const uint8_t *data, uint16_t data_length, SensorByteOrder count_byte_order, SensorData04 *decoded)
{
    uint32_t net_raw;

    if ((data == NULL) || (decoded == NULL))
    {
        return SENSOR_DECODE_ERROR_ARGUMENT;
    }

    if (data_length != SENSOR_DECODE_04_DATA_SIZE)
    {
        return SENSOR_DECODE_ERROR_LENGTH;
    }

    if ((count_byte_order != SENSOR_BYTE_ORDER_LITTLE_ENDIAN) &&(count_byte_order != SENSOR_BYTE_ORDER_BIG_ENDIAN))
    {
        return SENSOR_DECODE_ERROR_ARGUMENT;
    }

    decoded->major_version = data[0];
    decoded->sub_version = data[1];

    /* Data[2] contains intensity bits 0..7; Data[3].bit7 is bit 8. */
    decoded->signal_intensity = (uint16_t)data[2] |
        ((uint16_t)(data[3] & 0x80u) << 1);

    /* The remaining bits in Data[3] encode signal spread across channels. */
    decoded->signal_indications = (uint8_t)(data[3] & 0x7Fu);

    net_raw = SensorDecode_ReadU32(&data[4], count_byte_order);
    decoded->net_forward_pulses = SensorDecode_AsInt32(net_raw);
    decoded->reverse_pulses = SensorDecode_ReadU32(&data[8], count_byte_order);

    decoded->status_byte = data[12];
    decoded->calibration = ((data[12] & (1u << 0)) != 0u)
        ? SENSOR_CALIBRATION_FAIL
        : SENSOR_CALIBRATION_SUCCESS;
    decoded->fast_sampling = (data[12] & (1u << 1)) != 0u;
    decoded->sensor_removed = (data[12] & (1u << 2)) != 0u;
    decoded->foreign_object_detected = (data[12] & (1u << 3)) != 0u;
    decoded->ever_calibrated_successfully = (data[12] & (1u << 4)) != 0u;
    decoded->low_voltage = (data[12] & (1u << 5)) != 0u;

    decoded->debug_byte = data[13];
    decoded->filter_mode = (data[13] & (1u << 0)) != 0u;
    decoded->threshold_translation_status = (data[13] & (1u << 1)) != 0u;

    return SENSOR_DECODE_OK;
}

SensorDecodeStatus SensorDecode_06(const uint8_t *data, uint16_t data_length, SensorByteOrder count_byte_order, SensorData06 *decoded)
{
    uint8_t i;

    if ((data == NULL) || (decoded == NULL))
    {
        return SENSOR_DECODE_ERROR_ARGUMENT;
    }
    if (data_length != SENSOR_DECODE_06_DATA_SIZE)
    {
        return SENSOR_DECODE_ERROR_LENGTH;
    }
    if ((count_byte_order != SENSOR_BYTE_ORDER_LITTLE_ENDIAN) &&
        (count_byte_order != SENSOR_BYTE_ORDER_BIG_ENDIAN))
    {
        return SENSOR_DECODE_ERROR_ARGUMENT;
    }

    decoded->major_version = data[0];
    decoded->sub_version = data[1];
    decoded->net_forward_pulses = SensorDecode_AsInt32(
        SensorDecode_ReadU32(&data[3], count_byte_order));
    decoded->reverse_pulses = SensorDecode_ReadU32(&data[7], count_byte_order);

    decoded->signal_intensity[0] = SensorDecode_ReadU16(&data[11], count_byte_order);
    decoded->signal_intensity[1] = SensorDecode_ReadU16(&data[13], count_byte_order);
    decoded->signal_intensity[2] = SensorDecode_ReadU16(&data[15], count_byte_order);
    decoded->sampling_value[0] = SensorDecode_ReadU16(&data[23], count_byte_order);
    decoded->sampling_value[1] = SensorDecode_ReadU16(&data[25], count_byte_order);
    decoded->sampling_value[2] = SensorDecode_ReadU16(&data[27], count_byte_order);

    decoded->filter_number_of_interferences = data[35];
    decoded->filter_duration = SensorDecode_ReadU16(&data[36], count_byte_order);
    decoded->total_number_of_resets = data[44];
    decoded->realignment_reason = data[45];
    for (i = 0u; i < SENSOR_UID_SIZE; ++i)
    {
        decoded->uid[i] = data[46u + i];
    }

    decoded->status_byte_1 = data[56];
    decoded->calibration = ((data[56] & (1u << 0)) != 0u)
        ? SENSOR_CALIBRATION_FAIL
        : SENSOR_CALIBRATION_SUCCESS;
    decoded->fast_sampling = (data[56] & (1u << 1)) != 0u;
    decoded->sensor_removed = (data[56] & (1u << 2)) != 0u;
    decoded->foreign_object_detected = (data[56] & (1u << 3)) != 0u;
    decoded->currently_calibrated_successfully = (data[56] & (1u << 4)) != 0u;
    decoded->low_voltage = (data[56] & (1u << 5)) != 0u;

    decoded->status_byte_2 = data[57];
    decoded->channel_interfered[0] = (data[57] & (1u << 0)) != 0u;
    decoded->channel_interfered[1] = (data[57] & (1u << 1)) != 0u;
    decoded->channel_interfered[2] = (data[57] & (1u << 2)) != 0u;
    decoded->program_validation_error = (data[57] & (1u << 3)) != 0u;
    decoded->filter_mode = (data[57] & (1u << 4)) != 0u;

    decoded->minimum_signal_intensity_for_meter_breakdown = data[63];
    decoded->maximum_signal_intensity = SensorDecode_ReadU16(&data[64], count_byte_order);
    decoded->upper_limit_of_filtering_time = data[68];
    decoded->mcu_temperature_c = (data[70] <= 0x7Fu) ? (int8_t)data[70] : (int8_t)(-1 - (int16_t)(0xFFu - data[70]));

    return SENSOR_DECODE_OK;
}
