/** @file artemis_flash.c
 * @brief Flash memory library implementation for Ambiq Apollo3 (Artemis)
 *
 * @author Joseph Kurina, joseph.kurina@noaa.gov
 * @date January 17, 2025 (Revised April 24, 2025)
 * @version 0.0.8
 *
 * @copyright National Oceanic and Atmospheric Administration
 * @copyright Pacific Marine Environmental Lab
 * @copyright Environmental Development Division
 *
 * @note Assumes NVSTORAGE region is defined in the linker script
 * starting at FLASH_NVSTORAGE_START and is FLASH_NVSTORAGE_SIZE bytes.
 * Uses AmbiqSuite HAL functions for flash operations.
 * Erase operations must be performed explicitly before writing.
 * Uses critical sections (interrupt disable) during flash erase/write.
 * Adjusted HAL macro names based on build errors (ADDR2INST, ADDR2PAGE).
 * Uses AM_HAL_FLASH_PROGRAM_KEY for erase based on SDK v2.4.2 header.
 * IS_ALIGNED macro moved to artemis_flash.h
 * Manually calculates relative page number for erase.
 * Added small delay before erase loop as a potential timing fix.
 *
 * @bug No known bugs
 */

 #include "artemis_flash.h"
 #include "am_mcu_apollo.h" // Includes HAL flash functions, definitions, and critical section macros
 #include <string.h>        // For memcpy
 #include <stdint.h>        // For uintptr_t
 
 // Define Flash page size for Apollo3 (consult datasheet/HAL if different)
 // Typically 8KB pages for Apollo3 main flash.
 #define FLASH_PAGE_SIZE (AM_HAL_FLASH_PAGE_SIZE) // Use HAL definition (usually 8192)
 
 // NOTE: IS_ALIGNED macro definition removed from here as it was moved to artemis_flash.h
 
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
  * @brief Write data to NVSTORAGE.
  *
  * @param data Pointer to the data to write. Must be word-aligned.
  * @param size Size of the data in bytes. Must be a multiple of FLASH_ALIGNMENT (4 bytes).
  * @param offset Offset within NVSTORAGE (relative to FLASH_NVSTORAGE_START) to start writing. Must be word-aligned.
  * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
  *
  * @note CRITICAL: Assumes the target flash area has already been erased via flash_erase().
  * Flash can only transition bits from 1 to 0 during programming. Writing to
  * non-erased flash will likely result in corrupted data.
  * Uses critical section to disable interrupts during the write operation.
  */
 int flash_write(const void *data, size_t size, uint32_t offset)
 {
     uint32_t ui32WriteAddrAbsolute; // Absolute address for HAL call
     uint32_t ui32NumWords;
     uint32_t ui32ReturnCode;
 
     // --- Parameter Validation ---
     if (data == NULL)
     {
         return FLASH_INVALID_PARAM; // Null data pointer
     }
      if (size == 0)
     {
         return FLASH_SUCCESS; // Nothing to write
     }
     // Validate the *relative* offset and size
     if (!flash_is_valid_range(offset, size))
     {
         return FLASH_INVALID_PARAM; // Relative offset/size out of bounds
     }
     if (!IS_ALIGNED(size, FLASH_ALIGNMENT)) // Use macro from header
     {
         return FLASH_INVALID_PARAM; // Size not multiple of alignment
     }
     if (!IS_ALIGNED(offset, FLASH_ALIGNMENT)) // Use macro from header
     {
         // Relative offset must be word-aligned
         return FLASH_INVALID_PARAM; // Offset not word-aligned
     }
     // Check if the source data pointer itself is word-aligned (required by HAL)
     if (!IS_ALIGNED((uintptr_t)data, FLASH_ALIGNMENT)) // Use macro from header
     {
        return FLASH_INVALID_PARAM; // Data pointer not aligned
     }
 
     // Calculate absolute flash address for HAL function
     ui32WriteAddrAbsolute = FLASH_NVSTORAGE_START + offset;
 
     // Calculate number of 32-bit words to write
     ui32NumWords = size / sizeof(uint32_t); // Size is already validated as multiple of 4
 
     // --- Perform Flash Write using HAL ---
     AM_CRITICAL_BEGIN; // Disable interrupts
 
     // AM_HAL_FLASH_PROGRAM_KEY is required by the HAL to enable flash programming.
     ui32ReturnCode = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                                (uint32_t *)data, // Source data buffer
                                                (uint32_t *)ui32WriteAddrAbsolute, // Destination absolute flash address
                                                ui32NumWords);             // Number of words
 
     AM_CRITICAL_END; // Restore previous interrupt state
 
     // --- Check HAL Return Code ---
     if (ui32ReturnCode == AM_HAL_STATUS_SUCCESS)
     {
         return FLASH_SUCCESS;
     }
     else
     {
         // Log error details if possible (e.g., ui32ReturnCode)
         return FLASH_ERROR; // HAL function failed
     }
 }
 
 /**
  * @brief Erase a region in NVSTORAGE.
  *
  * @param offset Offset within NVSTORAGE (relative to FLASH_NVSTORAGE_START) to start erasing. Must align with Flash page start.
  * @param size Size of the region to erase in bytes. Must be a multiple of Flash page size.
  * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
  * @note Uses critical sections to disable interrupts during each page erase operation.
  */
 int flash_erase(uint32_t offset, size_t size)
 {
     uint32_t ui32StartAddrAbsolute; // Absolute address for HAL calls
     uint32_t ui32NumPages;
     uint32_t ui32CurrentPageAddrAbsolute;
     uint32_t ui32Instance;
     uint32_t ui32PageNumRelative; // Page number relative to the start of the instance
     uint32_t ui32ReturnCode = AM_HAL_STATUS_SUCCESS; // Initialize success code
     size_t i;
     size_t page_size = flash_get_page_size(); // Get actual page size
 
     // --- Parameter Validation ---
      if (size == 0)
     {
         return FLASH_SUCCESS; // Nothing to erase
     }
     // Validate the *relative* offset and size
     if (!flash_is_valid_range(offset, size))
     {
         return FLASH_INVALID_PARAM; // Relative offset/size out of bounds
     }
     if (!IS_ALIGNED(offset, page_size)) // Use macro from header
     {
         return FLASH_INVALID_PARAM; // Relative offset not page-aligned
     }
     if (!IS_ALIGNED(size, page_size)) // Use macro from header
     {
         return FLASH_INVALID_PARAM; // Size not multiple of page size
     }
 
 
     // Calculate absolute start address for HAL calls
     ui32StartAddrAbsolute = FLASH_NVSTORAGE_START + offset;
 
     // Calculate number of pages to erase
     ui32NumPages = size / page_size;
 
     // --- Add a small delay before starting erase sequence ---
     // This can sometimes help with timing issues. Adjust delay if needed.
     am_hal_flash_delay(FLASH_CYCLES_US(10)); // Delay 10 microseconds
 
     // --- Perform Flash Erase using HAL ---
     // Iterate through each page and erase it within a critical section
     for (i = 0; i < ui32NumPages; ++i)
     {
         ui32CurrentPageAddrAbsolute = ui32StartAddrAbsolute + (i * page_size);
 
         // Determine flash instance using HAL macro
         ui32Instance = AM_HAL_FLASH_ADDR2INST(ui32CurrentPageAddrAbsolute);
 
         // Manually calculate the page number relative to the start of the instance
         // Check if page size is a power of 2 for potential optimization later
         if (ui32Instance == 0) {
              // Instance 0 starts at absolute address 0xC000
              // Relative page number = (absolute address - 0xC000) / page_size
              // Note: This driver is designed for NV_STORAGE in instance 1,
              // so erasing instance 0 shouldn't happen with valid relative offsets.
              // However, calculate for completeness or future use.
              if (ui32CurrentPageAddrAbsolute >= 0xC000) {
                  ui32PageNumRelative = (ui32CurrentPageAddrAbsolute - 0xC000) / page_size;
              } else {
                  // Invalid address for instance 0
                  return FLASH_ERROR; // Or FLASH_INVALID_PARAM
              }
         } else if (ui32Instance == 1) {
              // Instance 1 starts at absolute address FLASH_NVSTORAGE_START (0x8C000)
              // Relative page number = (absolute address - FLASH_NVSTORAGE_START) / page_size
              ui32PageNumRelative = (ui32CurrentPageAddrAbsolute - FLASH_NVSTORAGE_START) / page_size;
         } else {
             // Should not happen for Apollo3 which has only 2 instances
             return FLASH_ERROR; // Invalid instance
         }
 
 
         AM_CRITICAL_BEGIN; // Disable interrupts for this page erase
 
         // Erase one page at a time using the PROGRAM key and the *relative* page number.
         ui32ReturnCode = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY, // Use PROGRAM key based on header
                                                  ui32Instance,
                                                  ui32PageNumRelative); // Use manually calculated relative page number
 
         AM_CRITICAL_END; // Restore previous interrupt state
 
         if (ui32ReturnCode != AM_HAL_STATUS_SUCCESS)
         {
             // Log error details if possible (e.g., ui32Instance, ui32PageNumRelative, ui32ReturnCode)
             return FLASH_ERROR; // HAL erase failed on one of the pages, exit loop
         }
 
         // Optional: Add a small delay after each page erase if problems persist
         // am_hal_flash_delay(FLASH_CYCLES_US(5));
     }
 
     return FLASH_SUCCESS; // All requested pages erased successfully
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
         return FLASH_INVALID_PARAM; // Null buffer pointer
     }
     if (size == 0)
     {
         return FLASH_SUCCESS; // Nothing to read
     }
     // Validate the *relative* offset and size
     if (!flash_is_valid_range(offset, size))
     {
         return FLASH_INVALID_PARAM; // Relative offset/size out of bounds
     }
 
     // Calculate absolute flash address for reading
     ui32ReadAddrAbsolute = FLASH_NVSTORAGE_START + offset;
 
     // --- Perform Read using memcpy ---
     // Reading from flash is a direct memory copy operation.
     // Interrupts generally do not need to be disabled for reads.
     memcpy(buffer, (const void *)ui32ReadAddrAbsolute, size);
 
     return FLASH_SUCCESS;
 }
 