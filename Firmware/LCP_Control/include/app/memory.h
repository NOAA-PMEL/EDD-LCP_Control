// memory.h - Memory management module header

#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "data.h"

// Define the structures needed for the transmission queue system
typedef struct sProfileData_t {
    uint8_t profile_number;      // The profile number this data represents
    Data_t *data;                // Pointer to the dynamically allocated data structure
    bool transmitted;            // Flag indicating if this profile has been transmitted
    uint8_t attempt_count;       // Number of transmission attempts for this profile
} ProfileData_t;

typedef struct sTransmissionQueue_t {
    ProfileData_t *profiles;     // Array of profile data entries
    uint8_t capacity;            // Maximum number of profiles the queue can hold
    uint8_t count;               // Current number of profiles in the queue
    uint8_t head;                // Index of the oldest profile in the queue
} TransmissionQueue_t;

// Memory threshold definitions
#define MEMORY_LOW_THRESHOLD       (4096)   // 4KB threshold to consider memory low
#define MEMORY_CRITICAL_THRESHOLD  (2048)   // 2KB threshold for critical memory situation

// Queue management functions
void MEM_init_transmission_queues(void);
void MEM_cleanup_transmission_queues(void);
bool MEM_queue_reset_attempts(TransmissionQueue_t *queue);
bool MEM_queue_add_profile(TransmissionQueue_t *queue, uint8_t profile_number, Data_t *data);
ProfileData_t* MEM_queue_get_next_profile(TransmissionQueue_t *queue);
void MEM_queue_mark_transmitted(TransmissionQueue_t *queue);
void MEM_queue_increment_attempt(TransmissionQueue_t *queue);
bool MEM_queue_max_attempts_reached(TransmissionQueue_t *queue, uint8_t max_attempts);
uint8_t MEM_queue_get_count(TransmissionQueue_t *queue);

// Memory management functions
bool MEM_is_memory_low(void);
bool MEM_is_memory_critical(void);
bool MEM_queue_remove_oldest(TransmissionQueue_t *queue);
void MEM_manage_memory_before_allocation(size_t required_size, TransmissionQueue_t *primary_queue, TransmissionQueue_t *secondary_queue);
void MEM_log_memory_status(const char* location, TransmissionQueue_t *park_queue, TransmissionQueue_t *prof_queue);

// Global transmission queues (declared externally, defined in memory.c)
extern TransmissionQueue_t park_queue;
extern TransmissionQueue_t prof_queue;

#endif // MEMORY_H