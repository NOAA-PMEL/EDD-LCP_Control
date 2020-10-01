#include "unity.h"
#include "Keller_9LD.h"
#include <stdbool.h>

#include "mock_am_hal_status.h"
#include "mock_am_hal_gpio.h"




void setUp(void)
{
}

void tearDown(void)
{
}

void test_Keller_9LD_convert_bytes_to_float(void)
{
    /* Given */
    uint8_t test_val[] = {0x41, 0xb8, 0xfb, 0xe7};
    float expected_val = 23.1229991912841796875;
    float returned_val = 0;

    /* When */
    returned_val = _convert_bytes_to_fp(test_val);

    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_val, returned_val);

}

void test_Keller_9LD_convert_bytes_to_sensor_date(void)
{
    /* Given */
    // 0b00010|1010|11101|00: 2|10|29|0
    // 0bYYYYYMMM 
    // 0bMDDDDDII
    // 0b00010 year (+2010)
    // 101 (month-H)
    // 0b0 (month-L)
    // 11101 (day)
    // 00 (Ignore)
    uint8_t data[] = {0b00010101, 0b01110100};
    uint8_t expected_day = 29;
    uint8_t expected_month = 10;
    uint16_t expected_year = 2012;
    uint8_t actual_day = 0;
    uint8_t actual_month = 0;
    uint16_t actual_year = 0;

    /* When */
    _convert_bytes_to_cal_date(data,
                                &actual_year,
                                &actual_month,
                                &actual_day);

    /* Then */
    TEST_ASSERT_EQUAL_UINT16(expected_year, actual_year);
    TEST_ASSERT_EQUAL_UINT8(expected_month, actual_month);
    TEST_ASSERT_EQUAL_UINT8(expected_day, actual_day);

}

void test_Keller_9LD_convert_bytes_to_unique_product_code(void)
{
    /* Given */
    uint8_t data[] = {0x04, 0x15, 0x01, 0x11};
    uint32_t actual_id = 0;
    uint32_t expected_id = 17892373;

    /* When */
    actual_id = _convert_product_codes(data);

    /* Then */
    TEST_ASSERT_EQUAL_UINT32(expected_id, actual_id);
}

void test_Keller_9LD_convert_user_information_to_struct(void)
{
    /* Given */
    sKeller_9LD_t keller = {
        .id = 0,
        .p_max = 0.0,
        .p_min = 0.0,
        .date.day = 0,
        .date.month = 0,
        .date.year = 0
    };
    sKeller_9LD_t *p_keller = &keller;
    uint8_t data[] = { 0x04, 0x15, 0x01, 0x11,
                       0x00, 0x00, 0x15, 0x74, 
                       0xBF, 0x80, 0x00, 0x00, 
                       0x41, 0x20, 0x00, 0x00};
    uint32_t expected_id = 17892373;
    uint8_t expected_day = 29;
    uint8_t expected_month = 10;
    uint16_t expected_year = 2012;
    float expected_pmin = -1.0e0;
    float expected_pmax = 1.0e1;

    /* When */
    _convert_user_information_bytes_to_struct(data, p_keller);

    /* Then */
    TEST_ASSERT_EQUAL_UINT32(expected_id, p_keller->id);
    TEST_ASSERT_EQUAL_UINT16(expected_year, p_keller->date.year);
    TEST_ASSERT_EQUAL_UINT8(expected_month, p_keller->date.month);
    TEST_ASSERT_EQUAL_UINT8(expected_day, p_keller->date.day);
    TEST_ASSERT_EQUAL_FLOAT(expected_pmin, p_keller->p_min);
    TEST_ASSERT_EQUAL_FLOAT(expected_pmax, p_keller->p_max);
    

}

void test_Keller_9LD_convert_bytes_to_temperature_minus_50C(void)
{
    /* Given */
    
    uint8_t data[] = {0x01, 0x80};
    float actual_temp = 0;
    float expected_temp = -50.0;

    /* When */
    actual_temp = _convert_bytes_to_temperature(data);

    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_temp, actual_temp);
    
}

void test_Keller_9LD_convert_bytes_to_temperature_minus_150C(void)
{
    /* Given */
    
    uint8_t data[] = {0xFB, 0x80};
    float actual_temp = 0;
    float expected_temp = 150.0;

    /* When */
    actual_temp = _convert_bytes_to_temperature(data);

    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_temp, actual_temp);
    
}


void test_Keller_9LD_convert_bytes_to_temperature_23_85C(void)
{
    /* Given */
    
    uint8_t data[] = {0x5D, 0xD1};
    float actual_temp = 0;
    float expected_temp = 23.85 ;

    /* When */
    actual_temp = _convert_bytes_to_temperature(data);

    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_temp, actual_temp);
    
}
void test_Keller_9LD_convert_bytes_to_pressure_minus_1bar(void)
{
    /* Given */
    /* Given */
    sKeller_9LD_t keller = {
        .id = 0,
        .p_max = 10.0,
        .p_min = -1.0,
        .date.day = 0,
        .date.month = 0,
        .date.year = 0
    };
    uint8_t data[] = {0x40, 0x00};
    float actual_pressure = 0;
    float expected_pressure = -1.0;

    /* When */
    actual_pressure = _convert_bytes_to_pressure(data, &keller);
    
    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_pressure, actual_pressure);
}

void test_Keller_9LD_convert_bytes_to_pressure_minus_partial_bar(void)
{
    /* Given */
    /* Given */
    sKeller_9LD_t keller = {
        .id = 0,
        .p_max = 10.0,
        .p_min = -1.0,
        .date.day = 0,
        .date.month = 0,
        .date.year = 0
    };
    uint8_t data[] = {0x4E, 0x20};
    float actual_pressure = 0;
    float expected_pressure =  0.213867;

    /* When */
    actual_pressure = _convert_bytes_to_pressure(data, &keller);
    
    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_pressure, actual_pressure);
}



 void test_Keller_9LD_convert_bytes_to_pressure_minus_3_3_bar(void)
{
    /* Given */
    /* Given */
    sKeller_9LD_t keller = {
        .id = 0,
        .p_max = 30.0,
        .p_min = 0.0,
        .date.day = 0,
        .date.month = 0,
        .date.year = 0
    };
    uint8_t data[] = {0x4E, 0x20};
    float actual_pressure = 0;
    float expected_pressure = 3.31055;

    /* When */
    actual_pressure = _convert_bytes_to_pressure(data, &keller);
    
    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_pressure, actual_pressure);
}

void test_Keller_9LD_convert_bytes_to_pressure_minus_0_33_bar(void)
{
    /* Given */
    /* Given */
    sKeller_9LD_t keller = {
        .id = 0,
        .p_max = 3.0,
        .p_min = 0.0,
        .date.day = 0,
        .date.month = 0,
        .date.year = 0
    };
    uint8_t data[] = {0x4E, 0x20};
    float actual_pressure = 0;
    float expected_pressure = 0.331055;

    /* When */
    actual_pressure = _convert_bytes_to_pressure(data, &keller);
    
    /* Then */
    TEST_ASSERT_EQUAL_FLOAT(expected_pressure, actual_pressure);
}