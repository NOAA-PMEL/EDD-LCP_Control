// memory.c - Memory management module implementation

#include <string.h>         // For memcpy
#include "memory.h"
#include "artemis_debug.h"
#include "artemis_flash.h" // Include for flash operations
#include "config.h"        // May contain DATA_MAX_SAMPLES or flash layout info
#include "FreeRTOS.h"      // For pvPortMalloc, vPortFree, xPortGetFreeHeapSize, etc.
#include "semphr.h"        // For mutexes

// --- Define the magic number for flash entries ---
// This is used to identify valid entries in flash storage.
#define FLASH_ENTRY_MAGIC 0xDEADBEEF

// --- Define static queue array ---
static QueuedDataEntry_t static_transmission_queue_array[STATIC_TX_QUEUE_CAPACITY];

// --- Define single global queue instance ---
TransmissionQueue_t transmission_queue = {
    .profiles = static_transmission_queue_array,    // Point to the static array
    .capacity = STATIC_TX_QUEUE_CAPACITY,           // Set fixed capacity
    .count = 0,
    .head = 0,
    .tail = 0,
    .mutex = NULL                                   // Initialize mutex in init function
};

// --- Define a simple header structure for flash entries --- 
typedef struct __attribute__((packed)) {
    uint32_t magic;           // Magic number to identify valid entries
    uint32_t total_entry_size;// Total size of this entry in flash (including header and padding)
    uint8_t  profile_number;  // Original profile/park number
    uint16_t num_samples;     // Actual number of samples stored
    uint8_t  is_park_data;    // 1 for park, 0 for profile
} FlashEntryHeader_t;

// --- Define flash storage metadata ---
// Using start/size from artemis_flash.h
#define FLASH_TX_QUEUE_START_OFFSET  (FLASH_NVSTORAGE_START)
#define FLASH_TX_QUEUE_SIZE          (FLASH_NVSTORAGE_SIZE)
#define FLASH_TX_QUEUE_END_OFFSET    (FLASH_TX_QUEUE_START_OFFSET + FLASH_TX_QUEUE_SIZE)

/**
 * @brief Initialize the flash queue metadata and mutex.
 * Erases the flash region designated for the queue on initialization.
 */
void MEM_init_flash_queue(void) {
    flash_mutex = xSemaphoreCreateMutex();
    if (flash_mutex == NULL) {
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Failed to create flash mutex!\n");
        // Handle error: perhaps block initialization or enter error state
        return;
    }

    ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Erasing flash region 0x%X (Size: %u bytes)...\n",
                         FLASH_TX_QUEUE_START_OFFSET, FLASH_TX_QUEUE_SIZE);

    // Erase the entire flash region dedicated to the queue.
    int erase_ret = flash_erase(FLASH_TX_QUEUE_START_OFFSET, FLASH_TX_QUEUE_SIZE);

    if (erase_ret == FLASH_SUCCESS) {
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Flash region erased successfully.\n");
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: ERROR - Failed to erase flash region! ret=%d\n", erase_ret);
        // Handle error: Initialization failed, flash might be unusable.
    }

    // Reset in-memory metadata variables
    flash_queue_head_offset = FLASH_TX_QUEUE_START_OFFSET;
    flash_queue_tail_offset = FLASH_TX_QUEUE_START_OFFSET;
    flash_queue_count = 0;

    flash_initialized = true;
    ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Flash queue metadata reset.\n");
}

/**
 * @brief Initialize the static transmission queue
 *
 * Initializes the static queue structure and its mutex.
 * Calls flash queue initialization.
 */
void MEM_init_transmission_queue(void) {
    // Initialize queue structure (static array is already allocated)
    transmission_queue.count = 0;
    transmission_queue.head = 0;
    transmission_queue.tail = 0;

    // Create mutex for the RAM queue
    transmission_queue.mutex = xSemaphoreCreateMutex();
    if (transmission_queue.mutex == NULL) {
        ARTEMIS_DEBUG_PRINTF("MEMORY: Failed to create transmission queue mutex!\n");
        // Handle error: perhaps block initialization or enter error state
        return;
    }

    // Initialize flash queue part
    MEM_init_flash_queue();

    ARTEMIS_DEBUG_PRINTF("MEMORY: Static transmission queue initialized. Capacity: %u\n", transmission_queue.capacity);
}

/**
 * @brief Add data from a completed static buffer (Park or Profile)
 * to the transmission queue or flash storage.
 *
 * @param data_to_queue Pointer to the static Data_t structure containing completed data.
 * @param is_park True if park data, false if profile data.
 * @return true if successfully added to RAM queue or flash, false otherwise.
 */
bool MEM_queue_add(Data_t *data_to_queue, bool is_park) {
    if (data_to_queue == NULL || data_to_queue->p == NULL) {
         ARTEMIS_DEBUG_PRINTF("MEMORY ADD: Invalid input data pointer.\n");
        return false;
    }

    // Check if flash is initialized (should be called after MEM_init...)
    if(!flash_initialized) {
        ARTEMIS_DEBUG_PRINTF("MEMORY ADD: Flash queue not initialized!\n");
        return false; // Or handle error appropriately
    }

    bool result = false;

    // Try adding to RAM queue first
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        if (transmission_queue.count < transmission_queue.capacity) {
            // --- Copy data into the static queue array slot ---
            uint8_t insert_index = transmission_queue.tail;
            QueuedDataEntry_t *entry = &transmission_queue.profiles[insert_index];

            entry->profile_number = data_to_queue->pNumber;
            entry->is_park_data = is_park;
            entry->num_samples = data_to_queue->wLength; // Assuming wLength holds the number of valid samples
            entry->attempt_count = 0;

            // Copy metadata
            memcpy(&entry->profile_metadata, data_to_queue->p, sizeof(pData));

            // Copy measurement data (only up to num_samples)
            // Ensure num_samples does not exceed DATA_MAX_SAMPLES
            uint16_t samples_to_copy = (entry->num_samples > DATA_MAX_SAMPLES) ? DATA_MAX_SAMPLES : entry->num_samples;
            if (data_to_queue->data.pressure != NULL) {
                 memcpy(entry->pressure_measurements, data_to_queue->data.pressure, samples_to_copy * sizeof(float));
            } else {
                memset(entry->pressure_measurements, 0, DATA_MAX_SAMPLES * sizeof(float)); // Or handle error
            }
            if (data_to_queue->data.temperature != NULL) {
                memcpy(entry->temp_measurements, data_to_queue->data.temperature, samples_to_copy * sizeof(float));
            } else {
                 memset(entry->temp_measurements, 0, DATA_MAX_SAMPLES * sizeof(float)); // Or handle error
            }
            // --- End data copy ---

            transmission_queue.tail = (transmission_queue.tail + 1) % transmission_queue.capacity;
            transmission_queue.count++;
            result = true;

            ARTEMIS_DEBUG_PRINTF("MEMORY ADD: Added %s %u to RAM queue. Count: %u\n",
                                 is_park ? "Park" : "Profile", entry->profile_number, transmission_queue.count);
        }
        xSemaphoreGive(transmission_queue.mutex);
    } else {
         ARTEMIS_DEBUG_PRINTF("MEMORY ADD: Failed to take queue mutex.\n");
         return false; // Mutex error
    }


    // If RAM queue was full, try storing to flash
    if (!result) {
        ARTEMIS_DEBUG_PRINTF("MEMORY ADD: RAM queue full. Attempting to store %s %u to flash.\n",
                             is_park ? "Park" : "Profile", data_to_queue->pNumber);
        result = MEM_store_to_flash(data_to_queue, is_park);
    }

    return result;
}


/**
 * @brief Get the next profile entry to transmit from the RAM queue.
 *
 * @return Pointer to the next QueuedDataEntry_t, or NULL if queue is empty.
 * Pointer is valid only until MEM_queue_mark_transmitted is called.
 */
QueuedDataEntry_t* MEM_queue_get_next(void) {
    QueuedDataEntry_t* entry = NULL;
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        if (transmission_queue.count > 0) {
            entry = &transmission_queue.profiles[transmission_queue.head];
        }
        xSemaphoreGive(transmission_queue.mutex);
    } else {
         ARTEMIS_DEBUG_PRINTF("MEMORY GET: Failed to take queue mutex.\n");
    }
    return entry; // Returns NULL if queue is empty or mutex fails
}

/**
 * @brief Mark the head profile as successfully transmitted, remove from RAM queue,
 * and attempt to load the next entry from flash (if available and space exists).
 */
void MEM_queue_mark_transmitted(void) {
    bool load_from_flash = false;
    uint8_t removed_profile_num = 0; // For logging

    // --- Part 1: Update RAM Queue ---
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
    
    if (transmission_queue.count > 0) { // Check if queue is not empty
        removed_profile_num = transmission_queue.profiles[transmission_queue.head].profile_number; // Store for logging

        // Clear the entry at head (debugging)
        // memset(&transmission_queue.profiles[transmission_queue.head], 0, sizeof(QueuedDataEntry_t)); // Can be large

        transmission_queue.head = (transmission_queue.head + 1) % transmission_queue.capacity;
        transmission_queue.count--;
        load_from_flash = true; // Signal to check flash

        ARTEMIS_DEBUG_PRINTF("MEMORY MARK TX: Marked %u as transmitted. RAM Queue Count: %u\n",
                            removed_profile_num, transmission_queue.count);
    } else {
            ARTEMIS_DEBUG_PRINTF("MEMORY MARK TX: Queue empty, nothing to mark.\n");
    }

    xSemaphoreGive(transmission_queue.mutex);
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY MARK TX: Failed to take queue mutex.\n");
    }

     // --- Part 2: Attempt to Load from Flash ---
     // Done outside the RAM queue mutex to avoid holding it while doing flash I/O
    if(load_from_flash) {
        MEM_load_from_flash(); // Attempt to fill the potentially freed slot
    }
}


/**
 * @brief Increment the transmission attempt counter for the current profile at the head.
 */
void MEM_queue_increment_attempt(void) {
     if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        if (transmission_queue.count > 0) {
             transmission_queue.profiles[transmission_queue.head].attempt_count++;
            ARTEMIS_DEBUG_PRINTF("MEMORY INC ATTEMPT: Incremented attempt count to %u for profile %u\n",
                        transmission_queue.profiles[transmission_queue.head].attempt_count,
                        transmission_queue.profiles[transmission_queue.head].profile_number);
        } else {
             ARTEMIS_DEBUG_PRINTF("MEMORY INC ATTEMPT: Queue empty.\n");
        }
         xSemaphoreGive(transmission_queue.mutex);
     } else {
         ARTEMIS_DEBUG_PRINTF("MEMORY INC ATTEMPT: Failed to take queue mutex.\n");
     }
}

/**
 * @brief Check if the current profile at the head has reached maximum transmission attempts.
 *
 * @param max_attempts Maximum number of attempts allowed.
 * @return true if max attempts reached, false otherwise or if queue empty.
 */
bool MEM_queue_max_attempts_reached(uint8_t max_attempts) {
    bool reached = false;
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        if (transmission_queue.count > 0) {
                // Check if the current profile at the head has reached max transmission attempts
                reached = (transmission_queue.profiles[transmission_queue.head].attempt_count >= max_attempts);
        }
        xSemaphoreGive(transmission_queue.mutex); // Release mutex
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY MAX ATTEMPT: Failed to take queue mutex.\n");
    }
    return reached;
}

/**
 * @brief Get the number of profiles currently in the RAM queue.
 *
 * @return Number of profiles in the RAM queue.
 */
uint8_t MEM_queue_get_count(void) {
     uint8_t count = 0;
     if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
         count = transmission_queue.count;
         xSemaphoreGive(transmission_queue.mutex);
     } else {
         ARTEMIS_DEBUG_PRINTF("MEMORY GET COUNT: Failed to take queue mutex.\n");
     }
    return count;
}

/**
* @brief Reset the transmission attempt counter for the current profile at the head.
*
* @return true if successful, false otherwise or if queue empty.
*/
bool MEM_queue_reset_attempts(void) {
    bool success = false;
    
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
    if (transmission_queue.count > 0) {
        transmission_queue.profiles[transmission_queue.head].attempt_count = 0;
        ARTEMIS_DEBUG_PRINTF("MEMORY RESET ATTEMPT: Reset attempt count for profile %u\n",
                    transmission_queue.profiles[transmission_queue.head].profile_number);
        success = true;
    } else {
            ARTEMIS_DEBUG_PRINTF("MEMORY RESET ATTEMPT: Queue empty.\n");
    }
    xSemaphoreGive(transmission_queue.mutex);
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY RESET ATTEMPT: Failed to take queue mutex.\n");
    }
    return success;
}


/**
 * @brief Stores data from a static buffer to flash memory if RAM queue is full.
 * Treats the flash region as a circular buffer.
 */
 bool MEM_store_to_flash(Data_t *data_to_store, bool is_park) {
    if (data_to_store == NULL || data_to_store->p == NULL) {
        ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Invalid input data pointer.\n");
        return false;
    }
    if (!flash_initialized) {
        ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Flash subsystem not initialized.\n");
        return false;
    }

    bool result = false;
    uint32_t required_payload_size;
    uint32_t required_total_size;
    uint32_t aligned_size;
    uint16_t num_samples_to_store;
    uint32_t write_offset; // The offset where we will actually write

    num_samples_to_store = (data_to_store->wLength > DATA_MAX_SAMPLES) ? DATA_MAX_SAMPLES : data_to_store->wLength;
    required_payload_size = sizeof(pData) + (2 * num_samples_to_store * sizeof(float));
    required_total_size = sizeof(FlashEntryHeader_t) + required_payload_size;
    aligned_size = (required_total_size + FLASH_ALIGNMENT - 1) & ~(FLASH_ALIGNMENT - 1);

    if (aligned_size > MAX_FLASH_ENTRY_SIZE) {
        ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: ERROR - Calculated size %u exceeds max buffer %u.\n", aligned_size, MAX_FLASH_ENTRY_SIZE);
        return false;
    }
    if (aligned_size > FLASH_TX_QUEUE_SIZE) {
         ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: ERROR - Entry size %u exceeds total flash queue size %u.\n", aligned_size, FLASH_TX_QUEUE_SIZE);
        return false;
    }

    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) == pdTRUE) {
        // --- Circular Buffer Space Check ---
        bool space_available = false;
        write_offset = 0; // Initialize write_offset

        if (flash_queue_tail_offset >= flash_queue_head_offset) {
            uint32_t available_space_end = FLASH_TX_QUEUE_END_OFFSET - flash_queue_tail_offset;
            uint32_t available_space_start = flash_queue_head_offset - FLASH_TX_QUEUE_START_OFFSET;

            // Check if it fits contiguously after tail
            if (aligned_size <= available_space_end) {
                write_offset = flash_queue_tail_offset;
                space_available = true;
            }
            // Check if it fits contiguously at the beginning (before head)
            else if (aligned_size <= available_space_start) {
                 // If head is exactly at START, we can't write before it unless buffer empty
                 if (flash_queue_head_offset != FLASH_TX_QUEUE_START_OFFSET || flash_queue_count == 0) {
                    write_offset = FLASH_TX_QUEUE_START_OFFSET; // Wrap around
                    space_available = true;
                    ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Wrapping tail to start (0x%X).\n", write_offset);
                 }
            }
        } else {        // flash_queue_tail_offset < flash_queue_head_offset
            // Check space between tail and head
            uint32_t available_space_middle = flash_queue_head_offset - flash_queue_tail_offset;
             if (aligned_size <= available_space_middle) {
                 write_offset = flash_queue_tail_offset;
                 space_available = true;
             }
        }

        // Final check: If buffer is empty, space is available at the current tail
        if (flash_queue_count == 0) {
            write_offset = flash_queue_tail_offset; // Should be START_OFFSET initially
            space_available = true; // Empty buffer, so we can write at the tail
        }
        // If after all checks, space is still not found, log it
        else if (!space_available) {
              ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Flash full or too fragmented for entry size %u (Head: 0x%X, Tail: 0x%X, Count: %u).\n",
                                 aligned_size, flash_queue_head_offset, flash_queue_tail_offset, flash_queue_count);
        }


        if (!space_available) {
            result = false; // Flash considered full or too fragmented
        } else {
            // --- Prepare buffer ---
            uint8_t *ptr = flash_write_buffer;
            memset(flash_write_buffer, 0xFF, aligned_size); // Pre-fill with erased state? (Optional)

            // 1. Header
            FlashEntryHeader_t *hdr = (FlashEntryHeader_t*)ptr;
            hdr->magic = FLASH_ENTRY_MAGIC;
            hdr->total_entry_size = aligned_size;
            hdr->profile_number = data_to_store->pNumber;
            hdr->num_samples = num_samples_to_store;
            hdr->is_park_data = (uint8_t)is_park;
            ptr += sizeof(FlashEntryHeader_t);

            // 2. pData (Metadata)
            memcpy(ptr, data_to_store->p, sizeof(pData));
            ptr += sizeof(pData);

            // 3. Pressure Samples
            if (data_to_store->data.pressure != NULL) {
                memcpy(ptr, data_to_store->data.pressure, num_samples_to_store * sizeof(float));
            } else {
                memset(ptr, 0, num_samples_to_store * sizeof(float)); // Fill with zero
                ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Warning - Pressure data pointer was NULL.\n");
            }
            ptr += num_samples_to_store * sizeof(float);

            // 4. Temperature Samples
             if (data_to_store->data.temperature != NULL) {
                memcpy(ptr, data_to_store->data.temperature, num_samples_to_store * sizeof(float));
            } else {
                 memset(ptr, 0, num_samples_to_store * sizeof(float)); // Fill with zero
                 ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Warning - Temperature data pointer was NULL.\n");
            }
            ptr += num_samples_to_store * sizeof(float);

            // 5. Padding
            uint32_t padding = aligned_size - required_total_size;
            if(padding > 0) {
                memset(ptr, 0xFF, padding);
            }

            // --- Write to Flash ---
            ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Writing %u bytes (Aligned: %u) for %s %u to flash offset 0x%X.\n",
                                required_total_size, aligned_size, is_park ? "Park" : "Profile",
                                data_to_store->pNumber, write_offset);

            int ret = flash_write(flash_write_buffer, aligned_size, write_offset - FLASH_TX_QUEUE_START_OFFSET); // Pass relative offset

            if (ret == FLASH_SUCCESS) {
                // Update tail pointer using modulus arithmetic
                flash_queue_tail_offset = FLASH_TX_QUEUE_START_OFFSET + (write_offset + aligned_size - FLASH_TX_QUEUE_START_OFFSET) % FLASH_TX_QUEUE_SIZE;
                flash_queue_count++;
                result = true;
                ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Write successful. Flash Count: %u, New Tail Offset: 0x%X\n",
                                     flash_queue_count, flash_queue_tail_offset);
            } else {
                ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: flash_write failed! ret=%d\n", ret);
                result = false;
            }
        }
        xSemaphoreGive(flash_mutex);
    } else {
        ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Failed to take flash mutex.\n");
        result = false;
    }
    return result;
}

/**
 * @brief Loads the oldest entry from flash into the RAM queue if space available.
 * Treats the flash region as a circular buffer.
 */
bool MEM_load_from_flash(void) {
    
    bool result = false;
    bool ram_queue_has_space = false;
    bool flash_has_data = false;
    FlashEntryHeader_t *hdr = (FlashEntryHeader_t*)flash_read_buffer; // Point to the read buffer
    uint32_t current_read_offset = 0;
    uint32_t entry_size_from_header = 0;

    // Check if flash is initialized (should be called after MEM_init...)
    if (!flash_initialized) {
        return false;
    }

    // Take flash mutex first
    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) == pdTRUE) {
        // Check if flash has data to read
        if (flash_queue_count > 0) {
            // Check if the head offset is valid
            current_read_offset = flash_queue_head_offset;

            // Check if the offset is within the valid range
            int ret = flash_read(flash_read_buffer, sizeof(FlashEntryHeader_t), current_read_offset - FLASH_TX_QUEUE_START_OFFSET);
            
            // Check if the read was successful
            if (ret != FLASH_SUCCESS) {
                ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to read header at 0x%X! ret=%d\n", current_read_offset, ret);
                
                // Release mutex and return false
                xSemaphoreGive(flash_mutex);
                return false;
            }

            // Check if the header is valid
            if (hdr->magic != FLASH_ENTRY_MAGIC || 
                hdr->total_entry_size == 0 || 
                hdr->total_entry_size > MAX_FLASH_ENTRY_SIZE || 
                hdr->total_entry_size % FLASH_ALIGNMENT != 0) {
                
                // Invalid header, log and return false
                ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Invalid header found at offset 0x%X! Magic=0x%X, Size=%u\n",
                                     current_read_offset, hdr->magic, hdr->total_entry_size);
                xSemaphoreGive(flash_mutex);
                return false;
            }

            // Read the full entry size from the header
            flash_has_data = true;
            entry_size_from_header = hdr->total_entry_size;
        }
        
        // Keep flash mutex
    } else {
        // Failed to take flash mutex, log and return false
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to take flash mutex.\n");
        return false;
    }

    // Check if flash has data to read
    if (!flash_has_data) {
        xSemaphoreGive(flash_mutex); // Release if no data found
        return false;
    }

    // Check RAM queue space
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        // Check if RAM queue has space
        if (transmission_queue.count < transmission_queue.capacity) {
            ram_queue_has_space = true; // Space is available in RAM queue
        }
        // Keep RAM mutex
    } else {
        // Failed to take RAM mutex, log and release flash mutex
         ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to take RAM queue mutex.\n");
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to take queue mutex.\n");
        xSemaphoreGive(flash_mutex); // Release flash mutex if RAM mutex failed
        return false;
    }

    // Check if RAM queue has space
    if (!ram_queue_has_space) {
        // RAM queue is full, log and release mutexes
        xSemaphoreGive(transmission_queue.mutex);   // Release RAM mutex
        xSemaphoreGive(flash_mutex);                // Release flash mutex
        return false;
    }

    // --- Proceed with loading: Flash has data, RAM has space, both mutexes held ---
    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Loading entry from flash offset 0x%X (Size: %u)\n",
                         current_read_offset, entry_size_from_header);
    
    // Read the full entry from flash
    int ret = flash_read(flash_read_buffer, entry_size_from_header, current_read_offset - FLASH_TX_QUEUE_START_OFFSET);
    
    // Check if the read was successful
    if (ret != FLASH_SUCCESS) {
        // Failed to read full entry, log and release mutexes
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to read full entry! ret=%d\n", ret);
        xSemaphoreGive(transmission_queue.mutex);
        xSemaphoreGive(flash_mutex);
        return false;
    }

    // Index to insert into RAM queue
    uint8_t insert_index = transmission_queue.tail; 
    
    // Point to the entry in RAM queue
    QueuedDataEntry_t *ram_entry = &transmission_queue.profiles[insert_index]; 
    
    // Pointer to read buffer
    uint8_t *read_ptr = flash_read_buffer; 

    // Skip header by incrementing the read pointer by header size
    read_ptr += sizeof(FlashEntryHeader_t);                             

    // Copy metadata
    memcpy(&ram_entry->profile_metadata, read_ptr, sizeof(pData));      
    
    // Move pointer to pressure samples
    read_ptr += sizeof(pData);                                          

    ram_entry->num_samples = hdr->num_samples;                          // Set number of samples from header
    uint16_t samples_to_copy = hdr->num_samples;                        // Number of samples to copy from flash
    
    // Ensure we don't copy more than the max defined in the struct
    if (samples_to_copy > DATA_MAX_SAMPLES) {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: ERROR - Sample count %u from flash exceeds max %u.\n", samples_to_copy, DATA_MAX_SAMPLES);
        samples_to_copy = DATA_MAX_SAMPLES;
    }

    // Copy pressure samples
    memcpy(ram_entry->pressure_measurements, read_ptr, samples_to_copy * sizeof(float));
    // Move pointer to temperature samples
    read_ptr += samples_to_copy * sizeof(float);    
    // Copy temperature samples                                       
    memcpy(ram_entry->temp_measurements, read_ptr, samples_to_copy * sizeof(float));

    ram_entry->profile_number = hdr->profile_number;
    ram_entry->is_park_data = (bool)hdr->is_park_data;
    ram_entry->attempt_count = 0;

    // --- Update RAM Queue ---
    transmission_queue.tail = (transmission_queue.tail + 1) % transmission_queue.capacity;
    transmission_queue.count++;
    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Loaded %s %u into RAM queue. Count: %u\n",
                         ram_entry->is_park_data ? "Park" : "Profile", ram_entry->profile_number, transmission_queue.count);

    // --- Update Flash Queue ---
    flash_queue_head_offset = FLASH_TX_QUEUE_START_OFFSET + (current_read_offset + entry_size_from_header - FLASH_TX_QUEUE_START_OFFSET) % FLASH_TX_QUEUE_SIZE;
    flash_queue_count--;
    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Advanced flash head offset to 0x%X. Flash Count: %u\n",
                          flash_queue_head_offset, flash_queue_count);

    result = true; // Success!

    // Release mutexes
    xSemaphoreGive(transmission_queue.mutex);
    xSemaphoreGive(flash_mutex);

    // Return success
    return result;
}


/**
 * @brief Log the current memory status for debugging.
 * Modified to remove direct queue parameters.
 *
 * @param location A string indicating where in the code this was called from
 */
void MEM_log_memory_status(const char* location) {
    uint32_t free_memory = xPortGetFreeHeapSize();
    uint32_t min_free_memory = xPortGetMinimumEverFreeHeapSize();
    uint8_t queue_count = MEM_queue_get_count(); // Use the getter function

    ARTEMIS_DEBUG_PRINTF("%s: Memory status - Free: %u bytes, Min ever free: %u bytes\n",
                         location, free_memory, min_free_memory);
    ARTEMIS_DEBUG_PRINTF("%s: Queue stats - RAM Queue: %u profiles, Flash Queue: %u profiles (estimated)\n",
                         location, queue_count, flash_queue_count); // Log both RAM and estimated flash count
}