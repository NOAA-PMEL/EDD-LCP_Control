#include "unity.h"
#include <string.h>
#include "S9_temp.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_S9_temp_parse_msg_should_return_valid_temp(void)
{
    /* Given */
    sS9_t sensor = 
    {
        .temperature = 0.0,
        .resistance = 0.0
    };

    char data_str[] = "2464.077906, 19.818897\r";
    float temperature = 19.818897;
    float resistance = 2464.077906;

    /* When */
    _parse_msg(data_str, strlen(data_str), &sensor);

    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(temperature, sensor.temperature);
    TEST_ASSERT_EQUAL_FLOAT(resistance, sensor.resistance);

}

void test_S9_temp_parse_msg_should_return_nan_for_invalid(void)
{
    /* Given */
    sS9_t sensor = 
    {
        .temperature = 0.0,
        .resistance = 0.0
    };
    char data_str[] = "19.818897\r2464.077906,";
    float temperature = NAN;
    float resistance = NAN;

    /* When */
    _parse_msg(data_str, strlen(data_str), &sensor);

    /* Then */
    TEST_ASSERT_FLOAT_IS_NAN(sensor.resistance);
    TEST_ASSERT_FLOAT_IS_NAN(sensor.temperature);
}

void test_S9_temp_parse_version_should_correctly_parse_version(void)
{
    /* Given */
    sS9_t sensor = {
        .info.C0 = 0,
        .info.C1 = 0,
        .info.C2 = 0,
        .info.C3 = 0,
        .info.R0 = 0,
    };
    char text[] = "MID=T003\rC0=0.000855\rC1=0.000293\rC2=0.000000\rC3=0.000000\rR0=10000.000\rUID=000000000F0F1A08535722E74FBC90B1\rS9T0 V0.45\rOK\r";

    char expected_mid[] = "T003";
    char expected_sensor[] = "S9T0";
    uint8_t expected_ver_major = 0;
    uint8_t expected_ver_minor = 45;
    float expected_c0 = 0.000855;
    float expected_c1 = 0.000293;
    float expected_c2 = 0.000000;
    float expected_c3 = 0.000000;
    float expected_r0 = 10000.00;
    uint8_t expected_UID[16] = {
        0x00,
        0x00,
        0x00,
        0x00,
        0x0F,
        0x0F,
        0x1A,
        0x08,
        0x53,
        0x57,
        0x22,
        0xE7,
        0x4F,
        0xBC,
        0x90,
        0xB1
    };

    char expected_status[] = "OK";

    /* When */
    _parse_version(text, &sensor);
    // printf("return %f\n", sensor.info.C0);

    /* Then */
    TEST_ASSERT_EQUAL_STRING(expected_mid, sensor.info.MID);
    TEST_ASSERT_EQUAL_FLOAT(expected_c0, sensor.info.C0);
    TEST_ASSERT_EQUAL_FLOAT(expected_c1, sensor.info.C1);
    TEST_ASSERT_EQUAL_FLOAT(expected_c2, sensor.info.C2);
    TEST_ASSERT_EQUAL_FLOAT(expected_c3, sensor.info.C3);
    TEST_ASSERT_EQUAL_FLOAT(expected_r0, sensor.info.R0);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_UID, sensor.info.UID, 16);
    TEST_ASSERT_EQUAL_UINT8(expected_ver_major, sensor.info.firmware.major);
    TEST_ASSERT_EQUAL_UINT8(expected_ver_minor, sensor.info.firmware.minor);
    TEST_ASSERT_EQUAL_STRING(expected_sensor, sensor.info.sensor);
    TEST_ASSERT_EQUAL_STRING(expected_status, sensor.info.status);
}