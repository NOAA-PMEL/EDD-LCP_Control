/** @file flash.c
 *  @brief Flash memory library
 *
 *  @author Joseph Kurina, joseph.kurina@noaa.gov
 *  @date January 17, 2025
 *  @version 0.0.1
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */

#include "flash.h"
#include "am_hal_flash.h"
#include <string.h> // For memcpy (if needed for reading)

// Helper macros for error checking
#define IS_ALIGNED(addr, alignment) (((addr) % (alignment)) == 0)

int flash_is_valid_range(uint32_t offset, size_t size) {
    return (offset + size <= FLASH_NVSTORAGE_SIZE) ? 1 : 0;
}

size_t flash_get_page_size(void) {
    return AM_HAL_FLASH_PAGE_SIZE;
}

int flash_write(const void *data, size_t size, uint32_t offset) {
    if (!data || size == 0 || !IS_ALIGNED(size, FLASH_ALIGNMENT)) {
        return FLASH_INVALID_PARAM; // Invalid arguments
    }

    if (!flash_is_valid_range(offset, size)) {
        return FLASH_INVALID_PARAM; // Out of bounds
    }

    uint32_t start_address = FLASH_NVSTORAGE_START + offset;

    // Ensure address alignment
    if (!IS_ALIGNED(start_address, FLASH_ALIGNMENT)) {
        return FLASH_INVALID_PARAM; // Misaligned address
    }

    // Erase the flash page(s) as necessary
    int status = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                         AM_HAL_FLASH_ADDR2INST(start_address),
                                         AM_HAL_FLASH_ADDR2PAGE(start_address));
    if (status != 0) {
        return FLASH_ERROR; // Erase failed
    }

    // Program the data
    status = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                       (uint32_t *)data,
                                       (uint32_t *)start_address,
                                       size / 4); // Convert size to word count
    return (status == 0) ? FLASH_SUCCESS : FLASH_ERROR;
}

int flash_erase(uint32_t offset, size_t size) {
    if (!flash_is_valid_range(offset, size)) {
        return FLASH_INVALID_PARAM; // Out of bounds
    }

    uint32_t start_address = FLASH_NVSTORAGE_START + offset;

    // Ensure size is a multiple of the page size
    size_t page_size = flash_get_page_size();
    if (!IS_ALIGNED(size, page_size)) {
        return FLASH_INVALID_PARAM; // Size must be page-aligned
    }

    // Loop through each page to erase
    while (size > 0) {
        int status = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                             AM_HAL_FLASH_ADDR2INST(start_address),
                                             AM_HAL_FLASH_ADDR2PAGE(start_address));
        if (status != 0) {
            return FLASH_ERROR; // Erase failed
        }

        start_address += page_size;
        size -= page_size;
    }

    return FLASH_SUCCESS;
}

int flash_read(void *buffer, size_t size, uint32_t offset) {
    if (!buffer || size == 0 || !flash_is_valid_range(offset, size)) {
        return FLASH_INVALID_PARAM; // Invalid arguments
    }

    uint32_t start_address = FLASH_NVSTORAGE_START + offset;

    // Perform a memory copy from flash to the buffer
    const uint8_t *flash_ptr = (const uint8_t *)start_address;
    uint8_t *dest_ptr = (uint8_t *)buffer;

    for (size_t i = 0; i < size; i++) {
        dest_ptr[i] = flash_ptr[i];
    }

    return FLASH_SUCCESS;
}
