// #ifdef TEST

#include "unity.h"

#include "dataframe.h"
#include "mock_sysinfo.h"

void setUp(void)
{
}

void tearDown(void)
{
}

// void test_dataframe_NeedToImplement(void)
// {
//     TEST_IGNORE_MESSAGE("Need to Implement dataframe");
// }


void create_linear_dataset(uint8_t *data, uint8_t start, uint8_t end, uint16_t len)
{
    uint8_t step = (end - start) / len;
    uint8_t last = start;
    *data++ = start;
    len--;
    while(len-- > 0)
    {
        *data = last + step;
        last = *data++; 
    }
}

void test_module_convert_latitude_to_uint64_t_should_shift_by_180_and_make_uint64_t(void)
{
    // Given: A latitude < 0
    float latitude = -89.7;
    uint64_t returned_lat = 0;

    float temp = (latitude + 180.0f);
    uint64_t expectedReturnlat = (uint64_t)(temp * 1000);

    // When: function is called
    returned_lat = module_convert_latitude_to_uint64_t(latitude);

    // Shifted uint64_t is returned
    TEST_ASSERT_EQUAL(expectedReturnlat, returned_lat);

}

void test_module_convert_longitude_to_uint64_t_should_adjust_to_0_to_360(void)
{
    // Given: a longitude < 0
    float longitude = -117.432;
    uint64_t returned_lon = 0;

    float temp = (360.0 + longitude);
    uint64_t expectedReturnLon = (uint64_t) (temp * 1000);

    // When: function is called
    returned_lon = module_convert_longitude_to_uint64_t(longitude);

    // Then: Adjusted uint64_t is returned
    TEST_ASSERT_INT_WITHIN(1, expectedReturnLon, returned_lon);
}

void test_module_convert_longitude_to_uint64_t_should_adjust_to_0_to_360_at_boundary(void)
{
    // Given: a longitude of -180.0
    float longitude = -180.000;
    uint64_t returned_lon = 0;
    uint64_t expectedReturnLon = 180000;

    // When: function is called
    returned_lon = module_convert_longitude_to_uint64_t(longitude);

    // Then: Adjusted uint64_t is returned
    TEST_ASSERT_INT_WITHIN(1, expectedReturnLon, returned_lon);
}

void test_DF_create_profile_should_create_a_valid_profile(void)
{
    // Given: A valid dataset
    float lat = 89.723;
    float lon = -12.819;
    uint32_t time = 1634049297;
    float depth[60] = {0};
    float temp[60] = {0};
    uint16_t len = 60;
    uint16_t dataLen = len * 4;
    uint16_t dfLen = 0;

    uint8_t df[270] = {0};

    uint8_t major = 2;
    uint8_t minor = 27;
    uint8_t build[6];

    uint8_t expected[270] ={0};


    expected[0] = '*';      /**< Start Character */
    expected[1] = 'L';      /**< LCP Text */
    expected[2] = 'C';
    expected[3] = 'P';          
    expected[4] = 1;        /**< SN High Byte */
    expected[5] = 1;        /**< SN Low Byte */
    expected[6] = 2;        /**< FW Major */
    expected[7] = 27;       /**< FW Minor */
    expected[8] = 2;        /**< Mode 2, Profile */        
    expected[9] = 0;        /**< Len high byte */
    expected[10] = dataLen;       /**< Len Low byte */
   
    /** Create the 0degC shifted data */
    for(uint8_t i=0; i<60; i++)
    {
        uint8_t idx = (i*4) + 11;
        expected[idx+2] = 0x13;
        expected[idx+3] = 0x88;
        // printf("%u\n", idx+3);

    }
    expected[251] = 0x54;       /**< CRC High Byte */
    expected[252] = 0x80;       /**< CRC Low Byte */
    expected[253] = 0xAB;
    expected[254] = 0;       /**< Latitude Byte 1 of 5 */
    expected[255] = 0;         /**< Latitude Byte 2 of 5 */
    expected[256] = 0x04;         /**< Latitude Byte 3 of 5 */
    expected[257] = 0x1D;         /**< Latitude Byte 4 of 5 */
    expected[258] = 0x9B;         /**< Latitude Byte 5 of 5 */

    expected[259] = 0;         /**< Longitude Byte 1 of 5 */
    expected[260] = 0;         /**< Longitude Byte 2 of 5 */
    expected[261] = 0x05;         /**< Longitude Byte 3 of 5 */
    expected[262] = 0x4C;         /**< Longitude Byte 4 of 5 */
    expected[263] = 0x2D;         /**< Longitude Byte 5 of 5 */

    expected[264] = 0x61;         /**< Time Byte 1 of 4 */
    expected[265] = 0x65;         /**< Time Byte 2 of 4 */
    expected[266] = 0x9D;         /**< Time Byte 3 of 4 */
    expected[267] = 0x11;         /**< Time Byte 4 of 4 */
    
    expected[268] = 0;         /**< Page # */
    expected[269] = '!';    /**< Final page indicator */

    // When: function is called

    SYS_get_serial_num_IgnoreAndReturn(257);

    SYS_get_firmware_Expect(&major, &minor, &build[0]);
    SYS_get_firmware_IgnoreArg_build();
    SYS_get_firmware_IgnoreArg_major();
    SYS_get_firmware_IgnoreArg_minor();
    SYS_get_firmware_ReturnThruPtr_major(&major);
    SYS_get_firmware_ReturnThruPtr_minor(&minor);


    DF_create_profile_page(df, time, len, lat, lon, temp, depth, 0, true);


    // Then: result DF should be valid
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, df, 270);


}


void test_DF_create_profile_should_create_a_valid_profile_page_two(void)
{
    // Given: A valid dataset
    float lat = 89.723;
    float lon = -12.819;
    uint32_t time = 1634049297;
    float depth[60] = {0};
    float temp[60] = {0};
    uint8_t page = 2;
    uint16_t len = 60;
    uint16_t dataLen = len * 4;
    uint16_t dfLen = 0;

    uint8_t df[270] = {0};

    uint8_t major = 2;
    uint8_t minor = 27;
    uint8_t build[6];

    uint8_t expected[270] ={0};


    expected[0] = '@';      /**< Start Character */
    expected[1] = 'L';      /**< LCP Text */
    expected[2] = 'C';
    expected[3] = 'P';          
    expected[4] = 1;        /**< SN High Byte */
    expected[5] = 1;        /**< SN Low Byte */
    expected[6] = 2;        /**< FW Major */
    expected[7] = 27;       /**< FW Minor */
    expected[8] = 2;        /**< Mode 2, Profile */        
    expected[9] = 0;        /**< Len high byte */
    expected[10] = dataLen;       /**< Len Low byte */
   
    /** Create the 0degC shifted data */
    for(uint8_t i=0; i<60; i++)
    {
        uint8_t idx = (i*4) + 11;
        expected[idx+2] = 0x13;
        expected[idx+3] = 0x88;
        // printf("%u\n", idx+3);

    }
    expected[251] = 0x54;       /**< CRC High Byte */
    expected[252] = 0x80;       /**< CRC Low Byte */
    expected[253] = 0xAB;
    expected[254] = 0;       /**< Latitude Byte 1 of 5 */
    expected[255] = 0;         /**< Latitude Byte 2 of 5 */
    expected[256] = 0x04;         /**< Latitude Byte 3 of 5 */
    expected[257] = 0x1D;         /**< Latitude Byte 4 of 5 */
    expected[258] = 0x9B;         /**< Latitude Byte 5 of 5 */

    expected[259] = 0;         /**< Longitude Byte 1 of 5 */
    expected[260] = 0;         /**< Longitude Byte 2 of 5 */
    expected[261] = 0x05;         /**< Longitude Byte 3 of 5 */
    expected[262] = 0x4C;         /**< Longitude Byte 4 of 5 */
    expected[263] = 0x2D;         /**< Longitude Byte 5 of 5 */

    expected[264] = 0x61;         /**< Time Byte 1 of 4 */
    expected[265] = 0x65;         /**< Time Byte 2 of 4 */
    expected[266] = 0x9D;         /**< Time Byte 3 of 4 */
    expected[267] = 0x11;         /**< Time Byte 4 of 4 */
    
    expected[268] = 2;         /**< Page # */
    expected[269] = '!';    /**< Final page indicator */

    // When: function is called

    SYS_get_serial_num_IgnoreAndReturn(257);

    SYS_get_firmware_Expect(&major, &minor, &build[0]);
    SYS_get_firmware_IgnoreArg_build();
    SYS_get_firmware_IgnoreArg_major();
    SYS_get_firmware_IgnoreArg_minor();
    SYS_get_firmware_ReturnThruPtr_major(&major);
    SYS_get_firmware_ReturnThruPtr_minor(&minor);


    DF_create_profile_page(df, time, len, lat, lon, temp, depth, page, true);


    // Then: result DF should be valid
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, df, 270);

}



void test_DF_create_profile_should_create_single_page_and_return_done(void)
{
    // Given: A valid dataset
    float lat = 89.723;
    float lon = -12.819;
    uint32_t time = 1634049297;
    float depth[60] = {0};
    float temp[60] = {0};
    uint16_t len = 60;
    uint16_t dataLen = len * 4;
    uint16_t dfLen = 0;

    uint8_t df[270] = {0};

    uint8_t major = 2;
    uint8_t minor = 27;
    uint8_t build[6];
    bool retVal = false;


    uint8_t expected[270] ={0};


    expected[0] = '*';      /**< Start Character */
    expected[1] = 'L';      /**< LCP Text */
    expected[2] = 'C';
    expected[3] = 'P';          
    expected[4] = 1;        /**< SN High Byte */
    expected[5] = 1;        /**< SN Low Byte */
    expected[6] = 2;        /**< FW Major */
    expected[7] = 27;       /**< FW Minor */
    expected[8] = 2;        /**< Mode 2, Profile */        
    expected[9] = 0;        /**< Len high byte */
    expected[10] = dataLen;       /**< Len Low byte */
   
    /** Create the 0degC shifted data */
    for(uint8_t i=0; i<60; i++)
    {
        uint8_t idx = (i*4) + 11;
        expected[idx+2] = 0x13;
        expected[idx+3] = 0x88;
        // printf("%u\n", idx+3);

    }
    expected[251] = 0x54;       /**< CRC High Byte */
    expected[252] = 0x80;       /**< CRC Low Byte */
    expected[253] = 0xAB;
    expected[254] = 0;       /**< Latitude Byte 1 of 5 */
    expected[255] = 0;         /**< Latitude Byte 2 of 5 */
    expected[256] = 0x04;         /**< Latitude Byte 3 of 5 */
    expected[257] = 0x1D;         /**< Latitude Byte 4 of 5 */
    expected[258] = 0x9B;         /**< Latitude Byte 5 of 5 */

    expected[259] = 0;         /**< Longitude Byte 1 of 5 */
    expected[260] = 0;         /**< Longitude Byte 2 of 5 */
    expected[261] = 0x05;         /**< Longitude Byte 3 of 5 */
    expected[262] = 0x4C;         /**< Longitude Byte 4 of 5 */
    expected[263] = 0x2D;         /**< Longitude Byte 5 of 5 */

    expected[264] = 0x61;         /**< Time Byte 1 of 4 */
    expected[265] = 0x65;         /**< Time Byte 2 of 4 */
    expected[266] = 0x9D;         /**< Time Byte 3 of 4 */
    expected[267] = 0x11;         /**< Time Byte 4 of 4 */
    
    expected[268] = 0;         /**< Page # */
    expected[269] = '!';    /**< Final page indicator */

    // When: function is called

    SYS_get_serial_num_IgnoreAndReturn(257);

    SYS_get_firmware_Expect(&major, &minor, &build[0]);
    SYS_get_firmware_IgnoreArg_build();
    SYS_get_firmware_IgnoreArg_major();
    SYS_get_firmware_IgnoreArg_minor();
    SYS_get_firmware_ReturnThruPtr_major(&major);
    SYS_get_firmware_ReturnThruPtr_minor(&minor);


    retVal = DF_create_profile(true, df, time, len, lat, lon, temp, depth);
    // DF_create_profile_page(df, time, len, lat, lon, temp, depth, 0, true);


    // Then: result DF should be valid
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, df, 270);
    TEST_ASSERT_EQUAL(true, retVal);
}





void test_DF_create_profile_should_create_multi_page_and_return_done(void)
{
    // Given: A valid dataset
    float lat = 89.723;
    float lon = -12.819;
    uint32_t time = 1634049297;
    float depth[120] = {0};
    float temp[120] = {0};
    uint16_t len = 120;
    uint16_t dataLen = len * 4;
    uint16_t dfLen = 0;

    uint8_t df1[270] = {0};
    uint8_t df2[270] = {0};

    uint8_t major = 2;
    uint8_t minor = 27;
    uint8_t build[6];
    bool retVal1 = false;
    bool retVal2 = false;


    uint8_t expected1[270] ={0};
    uint8_t expected2[270] ={0};

    expected1[0] = '*';      /**< Start Character */
    expected1[1] = 'L';      /**< LCP Text */
    expected1[2] = 'C';
    expected1[3] = 'P';          
    expected1[4] = 1;        /**< SN High Byte */
    expected1[5] = 1;        /**< SN Low Byte */
    expected1[6] = 2;        /**< FW Major */
    expected1[7] = 27;       /**< FW Minor */
    expected1[8] = 2;        /**< Mode 2, Profile */        
    expected1[9] = 0;        /**< Len high byte */
    expected1[10] = 0xF0;       /**< Len Low byte */
   
    /** Create the 0degC shifted data */
    for(uint8_t i=0; i<60; i++)
    {
        uint8_t idx = (i*4) + 11;
        expected1[idx+2] = 0x13;
        expected1[idx+3] = 0x88;
        // printf("%u\n", idx+3);

    }
    expected1[251] = 0x54;       /**< CRC High Byte */
    expected1[252] = 0x80;       /**< CRC Low Byte */
    expected1[253] = 0xAB;
    expected1[254] = 0;       /**< Latitude Byte 1 of 5 */
    expected1[255] = 0;         /**< Latitude Byte 2 of 5 */
    expected1[256] = 0x04;         /**< Latitude Byte 3 of 5 */
    expected1[257] = 0x1D;         /**< Latitude Byte 4 of 5 */
    expected1[258] = 0x9B;         /**< Latitude Byte 5 of 5 */

    expected1[259] = 0;         /**< Longitude Byte 1 of 5 */
    expected1[260] = 0;         /**< Longitude Byte 2 of 5 */
    expected1[261] = 0x05;         /**< Longitude Byte 3 of 5 */
    expected1[262] = 0x4C;         /**< Longitude Byte 4 of 5 */
    expected1[263] = 0x2D;         /**< Longitude Byte 5 of 5 */

    expected1[264] = 0x61;         /**< Time Byte 1 of 4 */
    expected1[265] = 0x65;         /**< Time Byte 2 of 4 */
    expected1[266] = 0x9D;         /**< Time Byte 3 of 4 */
    expected1[267] = 0x11;         /**< Time Byte 4 of 4 */
    
    expected1[268] = 0;         /**< Page # */
    expected1[269] = '&';    /**< Final page indicator */

    // When: function is called

    SYS_get_serial_num_IgnoreAndReturn(257);

    SYS_get_firmware_Expect(&major, &minor, &build[0]);
    SYS_get_firmware_IgnoreArg_build();
    SYS_get_firmware_IgnoreArg_major();
    SYS_get_firmware_IgnoreArg_minor();
    SYS_get_firmware_ReturnThruPtr_major(&major);
    SYS_get_firmware_ReturnThruPtr_minor(&minor);

    retVal1 = DF_create_profile(true, df1, time, len, lat, lon, temp, depth);

    expected2[0] = '@';      /**< Start Character */
    expected2[1] = 'L';      /**< LCP Text */
    expected2[2] = 'C';
    expected2[3] = 'P';          
    expected2[4] = 1;        /**< SN High Byte */
    expected2[5] = 1;        /**< SN Low Byte */
    expected2[6] = 2;        /**< FW Major */
    expected2[7] = 27;       /**< FW Minor */
    expected2[8] = 2;        /**< Mode 2, Profile */        
    expected2[9] = 0;        /**< Len high byte */
    expected2[10] = 0xF0;       /**< Len Low byte */
   
    /** Create the 0degC shifted data */
    for(uint8_t i=0; i<60; i++)
    {
        uint8_t idx = (i*4) + 11;
        expected2[idx+2] = 0x13;
        expected2[idx+3] = 0x88;
        // printf("%u\n", idx+3);

    }
    expected2[251] = 0x54;       /**< CRC High Byte */
    expected2[252] = 0x80;       /**< CRC Low Byte */
    expected2[253] = 0xAB;
    expected2[254] = 0;       /**< Latitude Byte 1 of 5 */
    expected2[255] = 0;         /**< Latitude Byte 2 of 5 */
    expected2[256] = 0x04;         /**< Latitude Byte 3 of 5 */
    expected2[257] = 0x1D;         /**< Latitude Byte 4 of 5 */
    expected2[258] = 0x9B;         /**< Latitude Byte 5 of 5 */

    expected2[259] = 0;         /**< Longitude Byte 1 of 5 */
    expected2[260] = 0;         /**< Longitude Byte 2 of 5 */
    expected2[261] = 0x05;         /**< Longitude Byte 3 of 5 */
    expected2[262] = 0x4C;         /**< Longitude Byte 4 of 5 */
    expected2[263] = 0x2D;         /**< Longitude Byte 5 of 5 */

    expected2[264] = 0x61;         /**< Time Byte 1 of 4 */
    expected2[265] = 0x65;         /**< Time Byte 2 of 4 */
    expected2[266] = 0x9D;         /**< Time Byte 3 of 4 */
    expected2[267] = 0x11;         /**< Time Byte 4 of 4 */
    
    expected2[268] = 1;         /**< Page # */
    expected2[269] = '!';    /**< Final page indicator */

    // When: function is called

    SYS_get_serial_num_IgnoreAndReturn(257);

    SYS_get_firmware_Expect(&major, &minor, &build[0]);
    SYS_get_firmware_IgnoreArg_build();
    SYS_get_firmware_IgnoreArg_major();
    SYS_get_firmware_IgnoreArg_minor();
    SYS_get_firmware_ReturnThruPtr_major(&major);
    SYS_get_firmware_ReturnThruPtr_minor(&minor);


    retVal2 = DF_create_profile(false, df2, time, len, lat, lon, temp, depth);
    // retVal2 = DF_create_profile(false, df2, NULL, NULL, NULL, NULL, NULL, NULL);
    // DF_create_profile_page(df, time, len, lat, lon, temp, depth, 0, true);


    // Then: result DF should be valid
    // printf("\n\n***DF1***\n");
    // for(uint16_t i=0;i<11;i++)
    // {
    //     printf("%u, %u, %u\n", i, expected1[i], df1[i]);
    // }

    // printf("\n\n***DF2***\n");
    // for(uint16_t i=0;i<11;i++)
    // {
    //     printf("%u, %u, %u\n", i, expected2[i], df2[i]);
    // }
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected1, df1, 270);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected2, df2, 270);
    TEST_ASSERT_EQUAL(false, retVal1);
    TEST_ASSERT_EQUAL(true, retVal2);
}

void test_module_create_crc_should_generate_a_valid_crc_short_set(void)
{
    // Given: A dataset
    uint8_t data[10];
    create_linear_dataset(data, 0, 10, 10);
    
    for(uint8_t i=0 ;i<10;i++)
    {
        printf("%u, ", data[i]);
    }

    // When: create crc is called:
    uint16_t crc = module_create_crc(data, 10);
    
    printf("%u\n", crc);

    // Then: CRC should jive
    TEST_ASSERT_EQUAL(0x2DA5, crc);
}

void test_DF_create_park_page_should_create_a_valid_page_0(void)
{

    #define df_create_park_len (30)
    // Given: A valid dataset
    float lat = 89.723;
    float lon = -12.819;
    
    uint8_t data[240] = {0};
    uint32_t time[df_create_park_len];
    for(uint8_t i=0; i<df_create_park_len; i++)
    {   
        uint32_t inc = 10 * i;
        time[i] = 1634049297 + inc;
    }
    printf("Time created");
    float depth[df_create_park_len] = {0};
    float temp[df_create_park_len] = {0};
    uint16_t len = df_create_park_len;
    uint16_t dataLen = len * 8;
    uint16_t dfLen = 0;

    uint8_t df[270] = {0};

    uint8_t major = 2;
    uint8_t minor = 27;
    uint8_t build[6];

    uint8_t expected[270] ={0};

    expected[0] = '*';      /**< Start Character */
    expected[1] = 'L';      /**< LCP Text */
    expected[2] = 'C';
    expected[3] = 'P';          
    expected[4] = 1;        /**< SN High Byte */
    expected[5] = 1;        /**< SN Low Byte */
    expected[6] = 2;        /**< FW Major */
    expected[7] = 27;       /**< FW Minor */
    expected[8] = 2;        /**< Mode 2, Profile */        
    expected[9] = 0;        /**< Len high byte */
    expected[10] = dataLen;       /**< Len Low byte */
   
    for(uint16_t i=0; i<len; i++)
    {
        uint16_t idx = 11+(i*8);
        expected[idx] = time[i] >> 24;
        expected[idx+1] = time[i] >> 16;
        expected[idx+2] = time[i] >> 8;
        expected[idx+3] = time[i] & 0x00FF;
        expected[idx+4] = 0u;
        expected[idx+5] = 0u;
        expected[idx+6] = 0x13;
        expected[idx+7] = 0x88;
    }
    expected[251] = 0xC6;       /**< CRC High Byte */
    expected[252] = 0xC2;       /**< CRC Low Byte */
    expected[253] = 0xAB;
    expected[254] = 0;       /**< Latitude Byte 1 of 5 */
    expected[255] = 0;         /**< Latitude Byte 2 of 5 */
    expected[256] = 0x04;         /**< Latitude Byte 3 of 5 */
    expected[257] = 0x1D;         /**< Latitude Byte 4 of 5 */
    expected[258] = 0x9B;         /**< Latitude Byte 5 of 5 */

    expected[259] = 0;         /**< Longitude Byte 1 of 5 */
    expected[260] = 0;         /**< Longitude Byte 2 of 5 */
    expected[261] = 0x05;         /**< Longitude Byte 3 of 5 */
    expected[262] = 0x4C;         /**< Longitude Byte 4 of 5 */
    expected[263] = 0x2D;         /**< Longitude Byte 5 of 5 */

    expected[264] = 0x61;         /**< Time Byte 1 of 4 */
    expected[265] = 0x65;         /**< Time Byte 2 of 4 */
    expected[266] = 0x9D;         /**< Time Byte 3 of 4 */
    expected[267] = 0x11;         /**< Time Byte 4 of 4 */
    
    expected[268] = 0;         /**< Page # */
    expected[269] = '!';    /**< Final page indicator */

    // When: function is called
    SYS_get_serial_num_IgnoreAndReturn(257);
    SYS_get_firmware_Expect(&major, &minor, &build[0]);
    SYS_get_firmware_IgnoreArg_build();
    SYS_get_firmware_IgnoreArg_major();
    SYS_get_firmware_IgnoreArg_minor();
    SYS_get_firmware_ReturnThruPtr_major(&major);
    SYS_get_firmware_ReturnThruPtr_minor(&minor);

    // // DF_create_generic_dataframe(df, LCP_MODE_PROFILE, time, len, data, lat, lon, 0, true);
    DF_create_park_page(df, len, lat, lon, time, depth, temp, 0, true);

    // Then: result DF should be valid
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, df, 270);

}
// #endif // TEST
