/** @file artemis_flash.c
 * @brief Flash memory library implementation for Ambiq Apollo3 (Artemis)
 *
 * @author Joseph Kurina, joseph.kurina@noaa.gov
 * @date January 17, 2025 (Revised April 24, 2025)
 * @version 0.0.3
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
 
 // Helper macro for checking alignment (optional, direct modulo check is fine too)
 #define IS_ALIGNED(addr, alignment) (((uintptr_t)(addr) % (alignment)) == 0)
 
 /**
  * @brief Validate if the given offset and size are within the NVSTORAGE bounds.
  * Uses overflow-safe check.
  *
  * @param offset Offset within NVSTORAGE.
  * @param size Size of the operation.
  * @return 1 if valid, 0 otherwise.
  */
 int flash_is_valid_range(uint32_t offset, size_t size)
 {
     // Check for potential overflow when calculating end address
     if (size == 0) {
         // Allow zero size if offset is valid
         return (offset < FLASH_NVSTORAGE_SIZE);
     }
     // Check if offset is within bounds AND offset + size doesn't exceed bounds
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
  * @param offset Offset within NVSTORAGE to start writing. Must be word-aligned.
  * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
  *
  * @note CRITICAL: Assumes the target flash area has already been erased via flash_erase().
  * Flash can only transition bits from 1 to 0 during programming. Writing to
  * non-erased flash will likely result in corrupted data.
  * Uses critical section to disable interrupts during the write operation.
  */
 int flash_write(const void *data, size_t size, uint32_t offset)
 {
     uint32_t ui32WriteAddr;
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
     if (!flash_is_valid_range(offset, size))
     {
         return FLASH_INVALID_PARAM; // Offset/size out of bounds
     }
     if (!IS_ALIGNED(size, FLASH_ALIGNMENT))
     {
         return FLASH_INVALID_PARAM; // Size not multiple of alignment
     }
     if (!IS_ALIGNED(offset, FLASH_ALIGNMENT))
     {
         // Although writing might start at word-aligned address within flash,
         // the offset itself relative to NVSTORAGE start should also be aligned.
         return FLASH_INVALID_PARAM; // Offset not word-aligned
     }
     // Check if the source data pointer itself is word-aligned (required by HAL)
     if (!IS_ALIGNED((uintptr_t)data, FLASH_ALIGNMENT))
     {
        return FLASH_INVALID_PARAM; // Data pointer not aligned
     }
 
     // Calculate absolute flash address
     ui32WriteAddr = FLASH_NVSTORAGE_START + offset;
 
     // Calculate number of 32-bit words to write
     ui32NumWords = size / sizeof(uint32_t); // Size is already validated as multiple of 4
 
     // --- Perform Flash Write using HAL ---
     AM_CRITICAL_BEGIN; // Disable interrupts
 
     // AM_HAL_FLASH_PROGRAM_KEY is required by the HAL to enable flash programming.
     ui32ReturnCode = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                                (uint32_t *)data, // Source data buffer
                                                (uint32_t *)ui32WriteAddr, // Destination flash address
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
  * @param offset Offset within NVSTORAGE to start erasing. Must align with Flash page start.
  * @param size Size of the region to erase in bytes. Must be a multiple of Flash page size.
  * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
  * @note Uses critical sections to disable interrupts during each page erase operation.
  */
 int flash_erase(uint32_t offset, size_t size)
 {
     uint32_t ui32StartAddr;
     uint32_t ui32NumPages;
     uint32_t ui32CurrentPageAddr;
     uint32_t ui32Instance;
     uint32_t ui32PageNum;
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
         return FLASH_INVALID_PARAM; // Offset/size out of bounds
     }
     if (!IS_ALIGNED(offset, page_size))
     {
         return FLASH_INVALID_PARAM; // Offset not page-aligned
     }
     if (!IS_ALIGNED(size, page_size))
     {
         return FLASH_INVALID_PARAM; // Size not multiple of page size
     }
 
 
     // Calculate absolute start address
     ui32StartAddr = FLASH_NVSTORAGE_START + offset;
 
     // Calculate number of pages to erase
     ui32NumPages = size / page_size;
 
     // --- Perform Flash Erase using HAL ---
     // Iterate through each page and erase it within a critical section
     for (i = 0; i < ui32NumPages; ++i)
     {
         ui32CurrentPageAddr = ui32StartAddr + (i * page_size);
 
         // Determine flash instance (Bank 0 or Bank 1) and page number within that instance.
         // Use HAL macros for robustness. Note: some SDK versions might use ADDR2INST/ADDR2PAGE.
         ui32Instance = AM_HAL_FLASH_ADDR_TO_INSTANCE(ui32CurrentPageAddr);
         ui32PageNum = AM_HAL_FLASH_ADDR_TO_PAGE(ui32CurrentPageAddr);
 
         AM_CRITICAL_BEGIN; // Disable interrupts for this page erase
 
         // Erase one page at a time using the correct ERASE key
         // AM_HAL_FLASH_ERASE_KEY is required by the HAL to enable flash erase.
         ui32ReturnCode = am_hal_flash_page_erase(AM_HAL_FLASH_ERASE_KEY,
                                                  ui32Instance,
                                                  ui32PageNum);
 
         AM_CRITICAL_END; // Restore previous interrupt state
 
         if (ui32ReturnCode != AM_HAL_STATUS_SUCCESS)
         {
             // Log error details if possible (e.g., ui32Instance, ui32PageNum, ui32ReturnCode)
             return FLASH_ERROR; // HAL erase failed on one of the pages, exit loop
         }
     }
 
     return FLASH_SUCCESS; // All requested pages erased successfully
 }
 
 /**
  * @brief Read data from NVSTORAGE.
  *
  * @param buffer Pointer to the buffer to store the data.
  * @param size Size of the data to read in bytes.
  * @param offset Offset within NVSTORAGE to start reading.
  * @return FLASH_SUCCESS on success, FLASH_ERROR or FLASH_INVALID_PARAM on failure.
  */
 int flash_read(void *buffer, size_t size, uint32_t offset)
 {
     uint32_t ui32ReadAddr;
 
     // --- Parameter Validation ---
     if (buffer == NULL)
     {
         return FLASH_INVALID_PARAM; // Null buffer pointer
     }
     if (size == 0)
     {
         return FLASH_SUCCESS; // Nothing to read
     }
     if (!flash_is_valid_range(offset, size))
     {
         return FLASH_INVALID_PARAM; // Offset/size out of bounds
     }
 
     // Calculate absolute flash address
     ui32ReadAddr = FLASH_NVSTORAGE_START + offset;
 
     // --- Perform Read using memcpy ---
     // Reading from flash is a direct memory copy operation.
     // Interrupts generally do not need to be disabled for reads.
     memcpy(buffer, (const void *)ui32ReadAddr, size);
 
     return FLASH_SUCCESS;
 }
 