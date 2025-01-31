/** @file artemis_flash.h
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

#ifndef ARTEMIS_FLASH_H
#define ARTEMIS_FLASH_H

#include <stdint.h>
#include <stddef.h>

// Return codes for Flash operations
#define FLASH_SUCCESS 0
#define FLASH_ERROR   -1
#define FLASH_INVALID_PARAM -2 // Add explicit error for invalid parameters

// Define start and size of NVSTORAGE
#define FLASH_NVSTORAGE_START 0x0004A000 // Per the linker file, this is the start of the NVSTORAGE region.
#define FLASH_NVSTORAGE_SIZE  (660 * 1024)

// Flash alignment constraints
#define FLASH_ALIGNMENT 4

/**
 * @brief Write data to NVSTORAGE.
 *
 * @param data Pointer to the data to write.
 * @param size Size of the data in bytes. Must be a multiple of FLASH_ALIGNMENT.
 * @param offset Offset within NVSTORAGE to start writing.
 * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
 */
int flash_write(const void *data, size_t size, uint32_t offset);

/**
 * @brief Erase a region in NVSTORAGE.
 *
 * @param offset Offset within NVSTORAGE to start erasing.
 * @param size Size of the region to erase in bytes. Should align with Flash page size.
 * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
 */
int flash_erase(uint32_t offset, size_t size);

/**
 * @brief Read data from NVSTORAGE.
 *
 * @param buffer Pointer to the buffer to store the data.
 * @param size Size of the data to read in bytes.
 * @param offset Offset within NVSTORAGE to start reading.
 * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
 */
int flash_read(void *buffer, size_t size, uint32_t offset);

/**
 * @brief Validate if the given offset and size are within the NVSTORAGE bounds.
 *
 * @param offset Offset within NVSTORAGE.
 * @param size Size of the operation.
 * @return 1 if valid, 0 otherwise.
 */
int flash_is_valid_range(uint32_t offset, size_t size);

/**
 * @brief Get the Flash page size.
 *
 * @return Size of a Flash page in bytes.
 */
size_t flash_get_page_size(void);

#endif // ARTEMIS_FLASH_H
