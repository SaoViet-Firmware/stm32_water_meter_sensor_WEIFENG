#ifndef SENSOR_DECODE_H
#define SENSOR_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_DECODE_04_DATA_SIZE 14u
#define SENSOR_DECODE_06_DATA_SIZE 72u
#define SENSOR_UID_SIZE            10u

typedef enum
{
    SENSOR_BYTE_ORDER_LITTLE_ENDIAN = 0,
    SENSOR_BYTE_ORDER_BIG_ENDIAN
} SensorByteOrder;

typedef enum
{
    SENSOR_DECODE_OK = 0,
    SENSOR_DECODE_ERROR_ARGUMENT,
    SENSOR_DECODE_ERROR_LENGTH
} SensorDecodeStatus;

typedef enum
{
    SENSOR_CALIBRATION_SUCCESS = 0,
    SENSOR_CALIBRATION_FAIL
} SensorCalibrationStatus;

typedef struct
{
    uint8_t major_version;
    uint8_t sub_version;

    uint16_t signal_intensity;
    uint8_t signal_indications;

    int32_t net_forward_pulses;
    uint32_t reverse_pulses;

    uint8_t status_byte;
    SensorCalibrationStatus calibration;
    bool fast_sampling;
    bool sensor_removed;
    bool foreign_object_detected;
    bool ever_calibrated_successfully;
    bool low_voltage;

    uint8_t debug_byte;
    bool filter_mode;
    bool threshold_translation_status;
} SensorData04;

typedef struct
{
    uint8_t major_version;
    uint8_t sub_version;
    int32_t net_forward_pulses;
    uint32_t reverse_pulses;
    uint16_t signal_intensity[3];
    uint16_t sampling_value[3];
    uint8_t filter_number_of_interferences;
    uint16_t filter_duration;
    uint8_t total_number_of_resets;
    uint8_t realignment_reason;
    uint8_t uid[SENSOR_UID_SIZE];

    uint8_t status_byte_1;
    SensorCalibrationStatus calibration;
    bool fast_sampling;
    bool sensor_removed;
    bool foreign_object_detected;
    bool currently_calibrated_successfully;
    bool low_voltage;

    uint8_t status_byte_2;
    bool channel_interfered[3];
    bool program_validation_error;
    bool filter_mode;

    uint8_t minimum_signal_intensity_for_meter_breakdown;
    uint16_t maximum_signal_intensity;
    uint8_t upper_limit_of_filtering_time;
    int8_t mcu_temperature_c;
} SensorData06;

SensorDecodeStatus SensorDecode_04(const uint8_t *data, uint16_t data_length, SensorByteOrder count_byte_order, SensorData04 *decoded);
SensorDecodeStatus SensorDecode_06(const uint8_t *data, uint16_t data_length, SensorByteOrder count_byte_order, SensorData06 *decoded);

#endif /* SENSOR_DECODE_H */
