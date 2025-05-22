/** @file artemis_flash.c
* @brief Flash memory library implementation for Ambiq Apollo3 (Artemis)
*
* @author Joseph Kurina, joseph.kurina@noaa.gov
* @date January 17, 2025 (Revised May 22, 2025)
* @version 0.0.11
*
* @copyright National Oceanic and Atmospheric Administration
* @copyright Pacific Marine Environmental Lab
* @copyright Environmental Development Division
*
* @note Assumes NVSTORAGE region is defined in the linker script
* starting at FLASH_NVSTORAGE_START (now 0x00080000 for Instance 1)
* and is FLASH_NVSTORAGE_SIZE bytes.
* Uses AmbiqSuite HAL functions for flash operations.
* Erase operations must be performed explicitly before writing.
* Uses critical sections (interrupt disable) during flash erase/write.
* Added read-back verification to flash_write.
* flash_erase now uses AM_HAL_FLASH_ADDR2INST and AM_HAL_FLASH_ADDR2PAGE for instance and page calculation.
*
* @bug No known bugs
*/

#include "artemis_flash.h"
#include "am_mcu_apollo.h" // Includes HAL flash functions, definitions, and critical section macros
#include <string.h>        // For memcpy, memcmp
#include <stdint.h>        // For uintptr_t
#include "artemis_debug.h" // For ARTEMIS_DEBUG_PRINTF
#include "am_hal_flash.h" // For AM_HAL_FLASH functions and macros

// Define Flash page size for Apollo3 (consult datasheet/HAL if different)
// Typically 8KB pages for Apollo3 main flash.
#define FLASH_PAGE_SIZE (AM_HAL_FLASH_PAGE_SIZE) // Use HAL definition (usually 8192)

// Static buffer for read-back verification in flash_write
// Sized to the maximum page size to handle any write size up to a full page.
static uint8_t flash_verify_buffer[FLASH_PAGE_SIZE];

// Removed hardcoded INSTANCE_0_BASE_ADDRESS and INSTANCE_1_BASE_ADDRESS
// as page calculation now relies on HAL macros AM_HAL_FLASH_ADDR2INST and AM_HAL_FLASH_ADDR2PAGE.

/**
* @brief Validate if the given relative offset and size are within the NVSTORAGE bounds.
* Uses overflow-safe check.
*
* @param offset Offset relative to FLASH_NVSTORAGE_START.
* @param size Size of the operation.
* @return 1 if valid, 0 otherwise.
*/
int flash_is_valid_range(uint32_t offset, size_t size)
{
    // Check for potential overflow when calculating end address
    if (size == 0) {
        // Allow zero size if offset is valid (within the total size)
        return (offset < FLASH_NVSTORAGE_SIZE);
    }
    // Check if offset is within bounds AND offset + size doesn't exceed bounds
    // Note: FLASH_NVSTORAGE_SIZE is the total size (e.g., 512KB)
    if (offset < FLASH_NVSTORAGE_SIZE && size <= (FLASH_NVSTORAGE_SIZE - offset))
    {
        return 1; // Valid range
    }
    return 0; // Invalid range (starts out of bounds, or ends out of bounds due to size)
}

/**
* @brief Get the Flash page size.
*
* @return Size of a Flash page in bytes.
*/
size_t flash_get_page_size(void)
{
    // Ensure HAL definition is available, otherwise provide a default
    #ifdef AM_HAL_FLASH_PAGE_SIZE
        return AM_HAL_FLASH_PAGE_SIZE;
    #else
        #warning "AM_HAL_FLASH_PAGE_SIZE not defined, using default 8192"
        return 8192; // Default Apollo3 page size
    #endif
}

/**
* @brief Write data to NVSTORAGE with read-back verification.
*
* @param data Pointer to the data to write. Must be word-aligned.
* @param size Size of the data in bytes. Must be a multiple of FLASH_ALIGNMENT (4 bytes).
* @param offset Offset within NVSTORAGE (relative to FLASH_NVSTORAGE_START) to start writing. Must be word-aligned.
* @return FLASH_SUCCESS on success (including successful verification), FLASH_ERROR or FLASH_INVALID_PARAM on failure.
*
* @note CRITICAL: Assumes the target flash area has already been erased via flash_erase().
* Flash can only transition bits from 1 to 0 during programming. Writing to
* non-erased flash will likely result in corrupted data.
* Uses critical section to disable interrupts during the HAL write operation.
* Verification read is performed immediately after the write.
*/
int flash_write(const void *data, size_t size, uint32_t offset)
{
    uint32_t ui32WriteAddrAbsolute; // Absolute address for HAL call
    uint32_t ui32NumWords;
    uint32_t ui32ReturnCode;

    // --- Parameter Validation ---
    if (data == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: Null data pointer.\n");
        return FLASH_INVALID_PARAM; // Null data pointer
    }
    if (size == 0)
    {
        return FLASH_SUCCESS; // Nothing to write
    }
    // Validate the *relative* offset and size
    if (!flash_is_valid_range(offset, size))
    {
        ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: Invalid range. Offset: %u, Size: %u\n", (unsigned int)offset, (unsigned int)size);
        return FLASH_INVALID_PARAM; // Relative offset/size out of bounds
    }
    if (!IS_ALIGNED(size, FLASH_ALIGNMENT)) // Use macro from header
    {
        ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: Size %u not aligned to %d.\n", (unsigned int)size, FLASH_ALIGNMENT);
        return FLASH_INVALID_PARAM; // Size not multiple of alignment
    }
    if (!IS_ALIGNED(offset, FLASH_ALIGNMENT)) // Use macro from header
    {
        // Relative offset must be word-aligned
        ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: Offset %u not aligned to %d.\n", (unsigned int)offset, FLASH_ALIGNMENT);
        return FLASH_INVALID_PARAM; // Offset not word-aligned
    }
    // Check if the source data pointer itself is word-aligned (required by HAL)
    if (!IS_ALIGNED((uintptr_t)data, FLASH_ALIGNMENT)) // Use macro from header
    {
    ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: Data pointer 0x%X not aligned to %d.\n", (unsigned int)(uintptr_t)data, FLASH_ALIGNMENT);
    return FLASH_INVALID_PARAM; // Data pointer not aligned
    }
    // Ensure the size for verification buffer is not exceeded
    if (size > FLASH_PAGE_SIZE) {
    ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: Size %u exceeds verification buffer size %u.\n", (unsigned int)size, (unsigned int)FLASH_PAGE_SIZE);
    return FLASH_INVALID_PARAM; // Requested size too large for static verify buffer
    }

    // Calculate absolute flash address for HAL function
    ui32WriteAddrAbsolute = FLASH_NVSTORAGE_START + offset;

    // Calculate number of 32-bit words to write
    ui32NumWords = size / sizeof(uint32_t); // Size is already validated as multiple of 4

    // --- Perform Flash Write using HAL ---
    AM_CRITICAL_BEGIN; // Disable interrupts for HAL flash operation

    // AM_HAL_FLASH_PROGRAM_KEY is required by the HAL to enable flash programming.
    ui32ReturnCode = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                            (uint32_t *)data, // Source data buffer
                                            (uint32_t *)ui32WriteAddrAbsolute, // Destination absolute flash address
                                            ui32NumWords);             // Number of words

    AM_CRITICAL_END; // Restore previous interrupt state

    // --- Check HAL Return Code & Perform Read-Back Verification ---
    if (ui32ReturnCode == AM_HAL_STATUS_SUCCESS)
    {
        // HAL reported success, now verify the content
        int read_ret = flash_read(flash_verify_buffer, size, offset); // Read back into static buffer
        if (read_ret == FLASH_SUCCESS)
        {
            if (memcmp(data, flash_verify_buffer, size) == 0)
            {
                // Data matches, write was successful and verified
                return FLASH_SUCCESS;
            }
            else
            {
                // Data mismatch
                ARTEMIS_DEBUG_PRINTF("FLASH WRITE VERIFY FAIL: Data mismatch after write at relative offset %u, size %u.\n", 
                                    (unsigned int)offset, (unsigned int)size);
                return FLASH_ERROR;
            }
        }
        else
        {
            // Read-back for verification failed
            ARTEMIS_DEBUG_PRINTF("FLASH WRITE VERIFY FAIL: Read-back for verification failed at relative offset %u, size %u. Read_ret: %d\n", 
                                (unsigned int)offset, (unsigned int)size, read_ret);
            return FLASH_ERROR;
        }
    }
    else
    {
        // HAL function failed
        ARTEMIS_DEBUG_PRINTF("FLASH WRITE FAIL: am_hal_flash_program_main failed with code %u for offset %u, size %u.\n", 
                            (unsigned int)ui32ReturnCode, (unsigned int)offset, (unsigned int)size);
        return FLASH_ERROR;
    }
}

/**
* @brief Erase a region in NVSTORAGE.
*
* @param offset Offset within NVSTORAGE (relative to FLASH_NVSTORAGE_START) to start erasing. Must align with Flash page start.
* @param size Size of the region to erase in bytes. Must be a multiple of Flash page size.
* @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
* @note Uses critical sections to disable interrupts during each page erase operation.
* Relies on AM_HAL_FLASH_ADDR2INST and AM_HAL_FLASH_ADDR2PAGE for instance and page calculation.
* FLASH_NVSTORAGE_START (0x00080000) is expected to be in Flash Instance 1.
*/
int flash_erase(uint32_t offset, size_t size)
{
    uint32_t ui32StartAddrAbsolute; // Absolute address for HAL calls
    uint32_t ui32NumPages;
    uint32_t ui32CurrentPageAddrAbsolute;
    uint32_t ui32Instance;
    uint32_t ui32PageNumRelative; // Page number relative to the instance start, as expected by HAL
    uint32_t ui32ReturnCode = AM_HAL_STATUS_SUCCESS; // Initialize success code
    size_t i;
    size_t page_size = flash_get_page_size(); // Get actual page size

    // --- Parameter Validation ---
    if (size == 0)
    {
        return FLASH_SUCCESS; // Nothing to erase
    }
    if (!flash_is_valid_range(offset, size))
    {
        ARTEMIS_DEBUG_PRINTF("FLASH ERASE FAIL: Invalid range. Offset: %u, Size: %u\n", (unsigned int)offset, (unsigned int)size);
        return FLASH_INVALID_PARAM;
    }
    if (!IS_ALIGNED(offset, page_size))
    {
        ARTEMIS_DEBUG_PRINTF("FLASH ERASE FAIL: Offset %u not page-aligned to %u.\n", (unsigned int)offset, (unsigned int)page_size);
        return FLASH_INVALID_PARAM;
    }
    if (!IS_ALIGNED(size, page_size))
    {
        ARTEMIS_DEBUG_PRINTF("FLASH ERASE FAIL: Size %u not multiple of page size %u.\n", (unsigned int)size, (unsigned int)page_size);
        return FLASH_INVALID_PARAM;
    }

    ui32StartAddrAbsolute = FLASH_NVSTORAGE_START + offset;
    ui32NumPages = size / page_size;

    am_hal_flash_delay(FLASH_CYCLES_US(10)); 

    for (i = 0; i < ui32NumPages; ++i)
    {
        ui32CurrentPageAddrAbsolute = ui32StartAddrAbsolute + (i * page_size);
        
        // Use HAL macros to determine instance and relative page number
        ui32Instance = AM_HAL_FLASH_ADDR2INST(ui32CurrentPageAddrAbsolute);
        ui32PageNumRelative = AM_HAL_FLASH_ADDR2PAGE(ui32CurrentPageAddrAbsolute);

        // Sanity check for the instance number
        if (ui32Instance >= AM_HAL_FLASH_NUM_INSTANCES) { // AM_HAL_FLASH_NUM_INSTANCES is 2
            ARTEMIS_DEBUG_PRINTF("FLASH ERASE FAIL: Invalid flash instance %u derived for address 0x%X.\n",
                                (unsigned int)ui32Instance, (unsigned int)ui32CurrentPageAddrAbsolute);
            return FLASH_ERROR;
        }
        // Additional check: Ensure the derived instance matches what we expect for FLASH_NVSTORAGE_START
        if (AM_HAL_FLASH_ADDR2INST(FLASH_NVSTORAGE_START) != ui32Instance) {
            ARTEMIS_DEBUG_PRINTF("FLASH ERASE WARNING: Address 0x%X (in NVSTORAGE) resolved to Instance %u, but FLASH_NVSTORAGE_START (0x%X) is in Instance %u.\n",
                                (unsigned int)ui32CurrentPageAddrAbsolute, (unsigned int)ui32Instance,
                                (unsigned int)FLASH_NVSTORAGE_START, (unsigned int)AM_HAL_FLASH_ADDR2INST(FLASH_NVSTORAGE_START));
            // This might indicate an issue if NVSTORAGE spans instances, or if macros are misinterpreting.
            // For now, we proceed with the HAL-derived instance and page.
        }


        AM_CRITICAL_BEGIN; 
        ui32ReturnCode = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                                ui32Instance,
                                                ui32PageNumRelative);
        AM_CRITICAL_END; 

        if (ui32ReturnCode != AM_HAL_STATUS_SUCCESS)
        {
            ARTEMIS_DEBUG_PRINTF("FLASH ERASE FAIL: am_hal_flash_page_erase failed for instance %u, page %u (abs 0x%X). Code: %u\n",
                                (unsigned int)ui32Instance, (unsigned int)ui32PageNumRelative, 
                                (unsigned int)ui32CurrentPageAddrAbsolute, (unsigned int)ui32ReturnCode);
            return FLASH_ERROR;
        }
    }

    return FLASH_SUCCESS;
}

/**
* @brief Read data from NVSTORAGE.
*
* @param buffer Pointer to the buffer to store the data.
* @param size Size of the data to read in bytes.
* @param offset Offset within NVSTORAGE (relative to FLASH_NVSTORAGE_START) to start reading.
* @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
*/
int flash_read(void *buffer, size_t size, uint32_t offset)
{
    uint32_t ui32ReadAddrAbsolute; // Absolute address for memcpy

    // --- Parameter Validation ---
    if (buffer == NULL)
    {
        ARTEMIS_DEBUG_PRINTF("FLASH READ FAIL: Null buffer pointer.\n");
        return FLASH_INVALID_PARAM; // Null buffer pointer
    }
    if (size == 0)
    {
        return FLASH_SUCCESS; // Nothing to read
    }
    // Validate the *relative* offset and size
    if (!flash_is_valid_range(offset, size))
    {
        ARTEMIS_DEBUG_PRINTF("FLASH READ FAIL: Invalid range. Offset: %u, Size: %u\n", (unsigned int)offset, (unsigned int)size);
        return FLASH_INVALID_PARAM; // Relative offset/size out of bounds
    }

    // Calculate absolute flash address for reading
    ui32ReadAddrAbsolute = FLASH_NVSTORAGE_START + offset;

    // --- Perform Read using memcpy ---
    memcpy(buffer, (const void *)ui32ReadAddrAbsolute, size);

    return FLASH_SUCCESS;
}
