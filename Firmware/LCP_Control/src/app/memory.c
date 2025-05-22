// memory.c - Memory management module implementation

#include <string.h>
#include "memory.h"
#include "artemis_debug.h"
#include "artemis_flash.h"    // Include for flash operations and FLASH_PAGE_SIZE
#include "StateMachine.h"     // Include for DATA_MAX_SAMPLES
#include "FreeRTOS.h"
#include "semphr.h"

// --- Define the magic number for flash entries ---
#define FLASH_ENTRY_MAGIC 0xDEADBEEF

// --- Define static queue array (for RAM queue) ---
static QueuedDataEntry_t static_transmission_queue_array[STATIC_TX_QUEUE_CAPACITY];

// --- Define single global queue instance ---
TransmissionQueue_t transmission_queue = {
    .profiles = static_transmission_queue_array,
    .capacity = STATIC_TX_QUEUE_CAPACITY,
    .count = 0,
    .head = 0,
    .tail = 0,
    .mutex = NULL
};

// --- Define a simple header structure for flash entries ---
typedef struct __attribute__((packed)) {
    uint32_t magic;             // Magic number to identify valid entries
    uint32_t total_entry_size;  // Should be MAX_FLASH_ENTRY_SIZE for page-based storage
    uint8_t  profile_number;
    uint16_t num_samples;       // Actual number of samples stored within the page
    uint8_t  is_park_data;
} FlashEntryHeader_t;

// --- Define flash storage metadata ---
#define FLASH_TX_QUEUE_RELATIVE_START_OFFSET 0
#define FLASH_TX_QUEUE_SIZE                  (FLASH_NVSTORAGE_SIZE) // 512KB

// --- Static variables for flash queue management ---
static SemaphoreHandle_t flash_mutex = NULL;
static uint32_t flash_queue_head_offset = 0; // Next read offset from start of NVSTORAGE
static uint32_t flash_queue_tail_offset = 0; // Next write offset from start of NVSTORAGE
static uint32_t flash_queue_count = 0;       // Number of items (8KB pages) currently in flash
static bool flash_initialized = false;

// Define the size for a single entry in flash (must be page-aligned and page size)
#define MAX_FLASH_ENTRY_SIZE (8 * 1024) // 8KB, aligned with flash page size

static uint8_t flash_write_buffer[MAX_FLASH_ENTRY_SIZE]; // Buffer for preparing data to write (one page)
static uint8_t flash_read_buffer[MAX_FLASH_ENTRY_SIZE];  // Buffer for reading data from flash (one page)

#define FLASH_ALIGNMENT 4 // Standard flash word alignment

#define FLASH_INIT_CHECK_SAMPLE_SIZE 256 // Size of the check buffer for initialization


/**
 * @brief Initialize the flash queue metadata and mutex.
 * Checks if the flash region appears erased; if not, erases it.
 * Halts the program if a required flash erase fails.
 */
void MEM_init_flash_queue(void) {
    flash_mutex = xSemaphoreCreateMutex();
    if (flash_mutex == NULL) {
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: CRITICAL ERROR - Failed to create flash mutex! Halting.\n");
        portDISABLE_INTERRUPTS(); // Ensure no other tasks run
        while (1); // Halt
    }

    bool needs_erase = true;

    // Using a smaller check buffer to avoid large stack allocation during init.
    // Checking first 256 bytes. If these are 0xFF, good chance the page/region is erased. 
    uint32_t check_buffer[FLASH_INIT_CHECK_SAMPLE_SIZE / sizeof(uint32_t)];
    const uint32_t num_words_to_check = sizeof(check_buffer) / sizeof(uint32_t);

    ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Checking if flash region (first %u bytes) is already erased...\n", (unsigned int)sizeof(check_buffer));
    int read_ret = flash_read(check_buffer, sizeof(check_buffer), FLASH_TX_QUEUE_RELATIVE_START_OFFSET);

    if (read_ret == FLASH_SUCCESS) {
        bool region_appears_erased = true;
        for (uint32_t i = 0; i < num_words_to_check; i++) {
            if (check_buffer[i] != 0xFFFFFFFF) {
                region_appears_erased = false;
                ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: First %u bytes not fully erased. Word %u at offset %u is 0x%08X.\n",
                                     (unsigned int)sizeof(check_buffer), i, (unsigned int)(i * sizeof(uint32_t)), (unsigned int)check_buffer[i]);
                break;
            }
        }
        if (region_appears_erased) {
            ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: First %u bytes appear erased. Skipping full region erase.\n", (unsigned int)sizeof(check_buffer));
            needs_erase = false;
        }
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Failed to read start of flash region for erase check (ret=%d). Assuming erase is needed.\n", read_ret);
        needs_erase = true; // If read fails, safer to assume it needs erase
    }

    if (needs_erase) {
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Proceeding with full erase of flash region (Offset: %u, Size: %u bytes).\n",
                             (unsigned int)FLASH_TX_QUEUE_RELATIVE_START_OFFSET, (unsigned int)FLASH_TX_QUEUE_SIZE);
        int erase_ret = flash_erase(FLASH_TX_QUEUE_RELATIVE_START_OFFSET, FLASH_TX_QUEUE_SIZE);
        if (erase_ret != FLASH_SUCCESS) {
            ARTEMIS_DEBUG_PRINTF("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: SYSTEM HALT!\n");
            ARTEMIS_DEBUG_PRINTF("REASON: Failed to erase flash region for data queue.\n");
            ARTEMIS_DEBUG_PRINTF("Flash erase error code: %d\n", erase_ret);
            flash_initialized = false;
            portDISABLE_INTERRUPTS();
            while (1); // Halt
        }
        ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Flash region erased successfully.\n");
    }
    
    // Reset in-memory metadata variables for linear fill
    flash_queue_head_offset = FLASH_TX_QUEUE_RELATIVE_START_OFFSET;
    flash_queue_tail_offset = FLASH_TX_QUEUE_RELATIVE_START_OFFSET;
    flash_queue_count = 0;
    flash_initialized = true;
    ARTEMIS_DEBUG_PRINTF("MEMORY INIT FLASH: Flash queue metadata reset and initialized for linear storage.\n");
}

// RAM Queue functions (MEM_init_transmission_queue, MEM_queue_add, etc.)
// remain largely the same as they primarily manage the RAM queue and then
// call MEM_store_to_flash or MEM_load_from_flash.
// The provided versions are mostly fine, so I'll keep them brief here for relevance.

void MEM_init_transmission_queue(void) {
    transmission_queue.count = 0;
    transmission_queue.head = 0;
    transmission_queue.tail = 0;
    transmission_queue.mutex = xSemaphoreCreateMutex();
    if (transmission_queue.mutex == NULL) {
        ARTEMIS_DEBUG_PRINTF("MEMORY: Failed to create transmission queue semaphore! Halting.\n");
        portDISABLE_INTERRUPTS(); while(1);
    }
    MEM_init_flash_queue(); // Initialize flash queue part
    ARTEMIS_DEBUG_PRINTF("MEMORY: Static transmission queue initialized. Capacity: %u\n", transmission_queue.capacity);
}

bool MEM_queue_add(Data_t *data_to_queue, bool is_park) {
    if (data_to_queue == NULL || data_to_queue->p == NULL) { /* ... */ return false; }
    if (!flash_initialized) { ARTEMIS_DEBUG_PRINTF("MEMORY ADD: Flash subsystem not initialized!\n"); return false; }

    bool result = false;
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        if (transmission_queue.count < transmission_queue.capacity) {
            // ... (Copy data to RAM queue - code omitted for brevity, same as provided) ...
            uint8_t insert_index = transmission_queue.tail;
            QueuedDataEntry_t *entry = &transmission_queue.profiles[insert_index];
            entry->profile_number = data_to_queue->pNumber;
            entry->is_park_data = is_park;
            entry->num_samples = data_to_queue->wLength;
            entry->attempt_count = 0;
            memcpy(&entry->profile_metadata, data_to_queue->p, sizeof(pData));
            uint16_t samples_to_copy = (entry->num_samples > DATA_MAX_SAMPLES) ? DATA_MAX_SAMPLES : entry->num_samples;
            if (data_to_queue->data.pressure != NULL) { memcpy(entry->pressure_measurements, data_to_queue->data.pressure, samples_to_copy * sizeof(float)); }
            else { memset(entry->pressure_measurements, 0, DATA_MAX_SAMPLES * sizeof(float)); }
            if (data_to_queue->data.temperature != NULL) { memcpy(entry->temp_measurements, data_to_queue->data.temperature, samples_to_copy * sizeof(float)); }
            else { memset(entry->temp_measurements, 0, DATA_MAX_SAMPLES * sizeof(float)); }
            transmission_queue.tail = (transmission_queue.tail + 1) % transmission_queue.capacity;
            transmission_queue.count++;
            result = true;
            ARTEMIS_DEBUG_PRINTF("MEMORY ADD: Added %s %u to RAM queue. Count: %u\n", is_park ? "Park" : "Profile", entry->profile_number, transmission_queue.count);
        }
        xSemaphoreGive(transmission_queue.mutex);
    } else { /* ... */ return false; }

    if (!result) { // RAM queue was full
        ARTEMIS_DEBUG_PRINTF("MEMORY ADD: RAM queue full. Attempting to store %s %u to flash.\n", is_park ? "Park" : "Profile", data_to_queue->pNumber);
        result = MEM_store_to_flash(data_to_queue, is_park);
    }
    return result;
}

QueuedDataEntry_t* MEM_queue_get_next(void) { /* ... same as provided ... */ return NULL;}
void MEM_queue_mark_transmitted(void) { /* ... same as provided, calls MEM_load_from_flash ... */ }
void MEM_queue_increment_attempt(void) { /* ... same as provided ... */ }
bool MEM_queue_max_attempts_reached(uint8_t max_attempts) { /* ... same as provided ... */ return false;}
uint8_t MEM_queue_get_count(void) { /* ... same as provided ... */ return 0;}
bool MEM_queue_reset_attempts(void) { /* ... same as provided ... */ return false;}


/**
 * @brief Stores data to flash memory using linear storage.
 * Each entry occupies one page (MAX_FLASH_ENTRY_SIZE).
 * If flash is full, an error is displayed and write fails.
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
    uint16_t num_samples_to_store = (data_to_store->wLength > DATA_MAX_SAMPLES) ? DATA_MAX_SAMPLES : data_to_store->wLength;
    // Calculate actual size of data payload + our header
    uint32_t actual_data_plus_header_size = sizeof(FlashEntryHeader_t) + sizeof(pData) + (2 * num_samples_to_store * sizeof(float));
    uint32_t entry_storage_page_size = MAX_FLASH_ENTRY_SIZE; // Each entry uses a full page

    if (actual_data_plus_header_size > entry_storage_page_size) {
        ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: ERROR - Actual data size with header (%u bytes) exceeds flash entry storage page size (%u bytes).\n",
                             (unsigned int)actual_data_plus_header_size, (unsigned int)entry_storage_page_size);
        return false;
    }

    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) == pdTRUE) {
        // --- Linear Storage Full Check ---
        if ((flash_queue_tail_offset + entry_storage_page_size) > FLASH_TX_QUEUE_SIZE) {
            ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: ERROR - Flash storage full. Cannot store new entry.\n");
            ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Tail at %u, Entry page size %u, Total flash capacity %u.\n",
                                 (unsigned int)flash_queue_tail_offset, (unsigned int)entry_storage_page_size, (unsigned int)FLASH_TX_QUEUE_SIZE);
            result = false; // Flash is full
        } else {
            uint32_t current_write_offset = flash_queue_tail_offset; // Relative offset for this new page

            ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Erasing page at RELATIVE offset %u (size %u) before writing.\n",
                                 (unsigned int)current_write_offset, (unsigned int)entry_storage_page_size);
            int erase_ret = flash_erase(current_write_offset, entry_storage_page_size); // Erase one page

            if (erase_ret == FLASH_SUCCESS) {
                // Prepare the 8KB buffer
                memset(flash_write_buffer, 0xFF, entry_storage_page_size); // Pre-fill page with erased state

                FlashEntryHeader_t *hdr = (FlashEntryHeader_t*)flash_write_buffer;
                hdr->magic = FLASH_ENTRY_MAGIC;
                hdr->total_entry_size = entry_storage_page_size; // Entry occupies full page for offset calculation
                hdr->profile_number = data_to_store->pNumber;
                hdr->num_samples = num_samples_to_store; // Actual number of samples within the data
                hdr->is_park_data = (uint8_t)is_park;
                
                uint8_t *ptr = flash_write_buffer + sizeof(FlashEntryHeader_t);
                memcpy(ptr, data_to_store->p, sizeof(pData)); ptr += sizeof(pData);
                
                if (data_to_store->data.pressure != NULL) {
                    memcpy(ptr, data_to_store->data.pressure, num_samples_to_store * sizeof(float));
                } else {
                     ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Warning - Pressure data pointer was NULL. Filling with zeros.\n");
                    memset(ptr, 0, num_samples_to_store * sizeof(float)); // Or 0xFF if preferred for unwritten samples
                }
                ptr += num_samples_to_store * sizeof(float);
                
                if (data_to_store->data.temperature != NULL) {
                    memcpy(ptr, data_to_store->data.temperature, num_samples_to_store * sizeof(float));
                } else {
                    ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Warning - Temperature data pointer was NULL. Filling with zeros.\n");
                    memset(ptr, 0, num_samples_to_store * sizeof(float)); // Or 0xFF
                }
                // The rest of flash_write_buffer (up to entry_storage_page_size) is already 0xFF

                ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Writing %s %u (actual data+hdr %u bytes) into flash page (size %u) at RELATIVE offset %u.\n",
                                     is_park ? "Park" : "Profile", data_to_store->pNumber,
                                     (unsigned int)actual_data_plus_header_size, (unsigned int)entry_storage_page_size, (unsigned int)current_write_offset);

                int write_ret = flash_write(flash_write_buffer, entry_storage_page_size, current_write_offset);

                if (write_ret == FLASH_SUCCESS) {
                    flash_queue_tail_offset += entry_storage_page_size;
                    flash_queue_count++;
                    result = true;
                    ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Write successful. Flash Count: %u, New Tail Offset: %u\n",
                                         (unsigned int)flash_queue_count, (unsigned int)flash_queue_tail_offset);
                } else {
                    ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: flash_write failed! ret=%d. Page was erased but data not written.\n", write_ret);
                    result = false;
                }
            } else {
                ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: flash_erase failed at offset %u! ret=%d\n", (unsigned int)current_write_offset, erase_ret);
                result = false;
            }
        }
        xSemaphoreGive(flash_mutex);
    } else {
        ARTEMIS_DEBUG_PRINTF("MEM STORE FLASH: Failed to take flash semaphore.\n");
        result = false;
    }
    return result;
}

/**
 * @brief Loads the oldest entry from linear flash storage into the RAM queue if space available.
 * Each entry is assumed to occupy a full page (MAX_FLASH_ENTRY_SIZE).
 */
bool MEM_load_from_flash(void) {
    if (!flash_initialized) {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Flash subsystem not initialized.\n");
        return false;
    }

    bool result = false;
    bool ram_queue_has_space = false;
    bool flash_has_entry_to_load = false;
    FlashEntryHeader_t *hdr = (FlashEntryHeader_t*)flash_read_buffer; // Point to the global read buffer
    uint32_t current_read_offset_relative = 0;
    uint32_t entry_page_size_from_header = 0; // Expected to be MAX_FLASH_ENTRY_SIZE

    // Take flash mutex to check flash state and potentially read
    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) == pdTRUE) {
        if (flash_queue_count > 0) {
            current_read_offset_relative = flash_queue_head_offset;
            // Sanity check for linear storage: head should be less than tail if count > 0
            if (current_read_offset_relative >= flash_queue_tail_offset) {
                 ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Inconsistent State! Head %u >= Tail %u but Flash Count %u > 0.\n",
                                     (unsigned int)current_read_offset_relative, (unsigned int)flash_queue_tail_offset, (unsigned int)flash_queue_count);
                 // This state implies corruption or a bug in offset/count management for linear store.
                 // To be safe, treat as no valid data to load.
            } else {
                // Read only the header first to validate
                int read_hdr_ret = flash_read(flash_read_buffer, sizeof(FlashEntryHeader_t), current_read_offset_relative);
                if (read_hdr_ret == FLASH_SUCCESS) {
                    if (hdr->magic == FLASH_ENTRY_MAGIC && hdr->total_entry_size == MAX_FLASH_ENTRY_SIZE) {
                        flash_has_entry_to_load = true;
                        entry_page_size_from_header = hdr->total_entry_size; // Should be MAX_FLASH_ENTRY_SIZE
                    } else {
                        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Invalid header at relative offset %u! Magic=0x%X, Size=%u (Expected %u for page).\n",
                                             (unsigned int)current_read_offset_relative, (unsigned int)hdr->magic, 
                                             (unsigned int)hdr->total_entry_size, (unsigned int)MAX_FLASH_ENTRY_SIZE);
                        // This is a critical error. The entry is corrupted.

                    }
                } else {
                    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to read header at relative offset %u! ret=%d\n", (unsigned int)current_read_offset_relative, read_hdr_ret);
                }
            }
        }
        
        if (!flash_has_entry_to_load) { // If no data to load or error reading header
            xSemaphoreGive(flash_mutex);
        }
        // Else, keep flash_mutex as we've validated a potential entry and will proceed if RAM has space
    } else {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to take flash mutex for initial check.\n");
        return false;
    }

    if (!flash_has_entry_to_load) {
        return false; // flash_mutex was released if this path is taken
    }

    // Now check RAM queue space
    if (xSemaphoreTake(transmission_queue.mutex, portMAX_DELAY) == pdTRUE) {
        if (transmission_queue.count < transmission_queue.capacity) {
            ram_queue_has_space = true;
        }
        if (!ram_queue_has_space) { // If no RAM space, release RAM mutex
            xSemaphoreGive(transmission_queue.mutex);
        }
        // Else, keep RAM mutex
    } else {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to take RAM queue Semaphore.\n");
        xSemaphoreGive(flash_mutex); // Release flash_mutex as we are aborting
        return false;
    }

    if (!ram_queue_has_space) {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: RAM queue full. Cannot load from flash.\n");
        xSemaphoreGive(flash_mutex); // Release flash_mutex as we are aborting
        return false;
    }

    // --- Both mutexes held (flash_mutex & transmission_queue.mutex) ---
    // --- Flash has a validated entry header, RAM has space ---

    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Loading entry from flash relative offset %u (Page Size: %u)\n",
                         (unsigned int)current_read_offset_relative, (unsigned int)entry_page_size_from_header);

    // Read the full entry page from flash into flash_read_buffer.
    // The header part is already there from the initial read, but re-reading the whole page is safer.
    int read_full_ret = flash_read(flash_read_buffer, entry_page_size_from_header, current_read_offset_relative);
    if (read_full_ret != FLASH_SUCCESS) {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Failed to read full entry page! ret=%d\n", read_full_ret);
        xSemaphoreGive(transmission_queue.mutex);
        xSemaphoreGive(flash_mutex);
        return false;
    }

    // Re-validate header integrity from the full buffer (hdr points to start of flash_read_buffer)
     if (hdr->magic != FLASH_ENTRY_MAGIC || hdr->total_entry_size != MAX_FLASH_ENTRY_SIZE) {
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Header inconsistency after full page read! Magic=0x%X, Size=%u. Aborting load.\n",
                            (unsigned int)hdr->magic, (unsigned int)hdr->total_entry_size);
        xSemaphoreGive(transmission_queue.mutex); 
        xSemaphoreGive(flash_mutex); 
        return false;
    }

    // Copy data to RAM queue
    uint8_t insert_index = transmission_queue.tail;
    QueuedDataEntry_t *ram_entry = &transmission_queue.profiles[insert_index];
    uint8_t *read_ptr = flash_read_buffer + sizeof(FlashEntryHeader_t); // Skip header

    memcpy(&ram_entry->profile_metadata, read_ptr, sizeof(pData)); read_ptr += sizeof(pData);
    
    ram_entry->num_samples = hdr->num_samples; // Actual number of samples stored
    uint16_t samples_to_copy = (hdr->num_samples > DATA_MAX_SAMPLES) ? DATA_MAX_SAMPLES : hdr->num_samples;

    memcpy(ram_entry->pressure_measurements, read_ptr, samples_to_copy * sizeof(float));
    read_ptr += samples_to_copy * sizeof(float);

    memcpy(ram_entry->temp_measurements, read_ptr, samples_to_copy * sizeof(float));
    
    ram_entry->profile_number = hdr->profile_number;
    ram_entry->is_park_data = (bool)hdr->is_park_data;
    ram_entry->attempt_count = 0;

    transmission_queue.tail = (transmission_queue.tail + 1) % transmission_queue.capacity;
    transmission_queue.count++;
    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Loaded %s %u into RAM queue. RAM Count: %u\n",
                         ram_entry->is_park_data ? "Park" : "Profile", ram_entry->profile_number, transmission_queue.count);

    // Update Flash Queue metadata
    flash_queue_head_offset += entry_page_size_from_header; // Advance by one page
    flash_queue_count--;
    ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Advanced flash head offset to %u. Flash Count: %u\n",
                         (unsigned int)flash_queue_head_offset, (unsigned int)flash_queue_count);

    // If flash becomes logically empty, reset offsets.
    if (flash_queue_count == 0) {
        // This also implies flash_queue_head_offset should now equal flash_queue_tail_offset
        ARTEMIS_DEBUG_PRINTF("MEM LOAD FLASH: Flash queue is now empty. Resetting flash head/tail offsets to start.\n");
        flash_queue_head_offset = FLASH_TX_QUEUE_RELATIVE_START_OFFSET;
        flash_queue_tail_offset = FLASH_TX_QUEUE_RELATIVE_START_OFFSET;
    }

    result = true;
    xSemaphoreGive(transmission_queue.mutex);
    xSemaphoreGive(flash_mutex);
    return result;
}


void MEM_log_memory_status(const char* location) {
    uint32_t free_heap = xPortGetFreeHeapSize();
    uint32_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
    uint8_t ram_q_count = 0; 
    uint32_t flash_pages_count_local = 0; // Changed name to reflect it counts pages
    uint32_t local_flash_head = 0;
    uint32_t local_flash_tail = 0;

    // Get RAM Queue count
    if (xSemaphoreTake(transmission_queue.mutex, (TickType_t)10) == pdTRUE) {
        ram_q_count = transmission_queue.count;
        xSemaphoreGive(transmission_queue.mutex);
    } else {
        ARTEMIS_DEBUG_PRINTF("%s: Could not get RAM queue mutex for status.\n", location);
    }

    // Get Flash Queue status
    if (xSemaphoreTake(flash_mutex, (TickType_t)10) == pdTRUE) {
        flash_pages_count_local = flash_queue_count;
        local_flash_head = flash_queue_head_offset;
        local_flash_tail = flash_queue_tail_offset;
        xSemaphoreGive(flash_mutex);
    } else {
        ARTEMIS_DEBUG_PRINTF("%s: Could not get flash mutex for status.\n", location);
    }

    ARTEMIS_DEBUG_PRINTF("%s: HeapFree: %u, HeapMinFree: %u\n",
                         location, (unsigned int)free_heap, (unsigned int)min_free_heap);
    ARTEMIS_DEBUG_PRINTF("%s: Queues - RAM: %u items, Flash: %u pages (HeadOffset: %u, TailOffset: %u)\n",
                         location, ram_q_count, (unsigned int)flash_pages_count_local,
                         (unsigned int)local_flash_head, (unsigned int)local_flash_tail);
}