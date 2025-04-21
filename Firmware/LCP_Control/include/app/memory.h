// memory.h - Memory management module header

#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"       // Is this needed?
#include "semphr.h"         // Include for SemaphoreHandle_t
#include "data.h"           // Include for pData definition
#include "StateMachine.h"   // Include for DATA_MAX_SAMPLES

// Define DATA_MAX_SAMPLES based on the larger of Park/Profile
#if DATA_PARK_SAMPLES_MAX > DATA_PROFILE_SAMPLES_MAX
    #define DATA_MAX_SAMPLES DATA_PARK_SAMPLES_MAX
#else
    #define DATA_MAX_SAMPLES DATA_PROFILE_SAMPLES_MAX
#endif

// --- Define structure for entries in the static RAM queue ---
typedef struct {
    uint8_t profile_number;                             // The profile number this data represents
    bool is_park_data;                                  // Flag to distinguish Park (true) vs Profile (false)
    pData profile_metadata;                             // Store metadata directly (start/stop time, GPS, etc.)
    float pressure_measurements[DATA_MAX_SAMPLES];      // Store measurements directly
    float temp_measurements[DATA_MAX_SAMPLES];          // Store measurements directly
    uint16_t num_samples;                               // Store actual number of samples recorded for this entry
    uint8_t attempt_count;                              // Number of transmission attempts for this entry
} QueuedDataEntry_t;

// --- TransmissionQueue_t for static queue ---
typedef struct sTransmissionQueue_t {
    QueuedDataEntry_t *profiles;    // Pointer to the static array (will hold QueuedDataEntry_t)
    uint8_t capacity;               // Fixed capacity (STATIC_TX_QUEUE_CAPACITY)
    uint8_t count;                  // Current number of profiles in the queue
    uint8_t head;                   // Index of the oldest profile (for reading)
    uint8_t tail;                   // Index of the next insertion point
    SemaphoreHandle_t mutex;        // Mutex for safety
} TransmissionQueue_t;

// --- Define static queue capacity ---
#define STATIC_TX_QUEUE_CAPACITY 20

// --- Function Declarations ---
void MEM_init_transmission_queue(void); // Renamed from MEM_init_transmission_queues
void MEM_init_flash_queue(void);        // Initialize flash metadata

// Queue management functions
bool MEM_queue_add(Data_t *data_to_queue, bool is_park);    // Adds data from static buffer to queue/flash
QueuedDataEntry_t* MEM_queue_get_next(void);                // Returns pointer to QueuedDataEntry_t
void MEM_queue_mark_transmitted(void);                      // Marks head entry transmitted, triggers flash load check
void MEM_queue_increment_attempt(void);                     // Operates on the single queue head
bool MEM_queue_max_attempts_reached(uint8_t max_attempts);  // Operates on the single queue head
uint8_t MEM_queue_get_count(void);                          // Returns count of the single queue
bool MEM_queue_reset_attempts(void);                        // Operates on single queue head

// Flash storage functions
bool MEM_store_to_flash(Data_t *data_to_store, bool is_park);   // Stores data to flash if queue is full
bool MEM_load_from_flash(void);                                 // Loads oldest data from flash to queue if space

// Logging function for heap memory status
void MEM_log_memory_status(const char* location);

// --- Declare single global transmission queue ---
extern TransmissionQueue_t transmission_queue; // Single queue instance

#endif // MEMORY_H