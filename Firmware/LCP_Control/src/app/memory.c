// memory.c - Memory management module implementation

#include <string.h>
#include "memory.h"
#include "artemis_debug.h"
#include "config.h"

// Global transmission queues
TransmissionQueue_t park_queue = {0};
TransmissionQueue_t prof_queue = {0};

/**
 * @brief Initialize the transmission queues
 * 
 * Allocates memory for the profile arrays and initializes queue structures.
 */
void MEM_init_transmission_queues(void) {
    // Allocate memory for the profile arrays (When SYSTEM_PROFILE_NUMBER=100, this is about 1.6KB)
    park_queue.profiles = (ProfileData_t*)pvPortMalloc(SYSTEM_PROFILE_NUMBER * sizeof(ProfileData_t)); // 8 bytes per park
    prof_queue.profiles = (ProfileData_t*)pvPortMalloc(SYSTEM_PROFILE_NUMBER * sizeof(ProfileData_t)); // 8 bytes per profile
    
    if (park_queue.profiles != NULL) {
        memset(park_queue.profiles, 0, SYSTEM_PROFILE_NUMBER * sizeof(ProfileData_t));
        park_queue.capacity = SYSTEM_PROFILE_NUMBER;
        park_queue.count = 0;
        park_queue.head = 0;
        ARTEMIS_DEBUG_PRINTF("MEMORY: Successfully initialized park queue\n");
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY: Failed to allocate memory for park queue\n");
    }
    
    if (prof_queue.profiles != NULL) {
        memset(prof_queue.profiles, 0, SYSTEM_PROFILE_NUMBER * sizeof(ProfileData_t));
        prof_queue.capacity = SYSTEM_PROFILE_NUMBER;
        prof_queue.count = 0;
        prof_queue.head = 0;
        ARTEMIS_DEBUG_PRINTF("MEMORY: Successfully initialized profile queue\n");
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY: Failed to allocate memory for profile queue\n");
    }
}

/**
 * @brief Clean up the transmission queues
 * 
 * Frees all allocated memory in the queues and resets their structures.
 */
void MEM_cleanup_transmission_queues(void) {
    // Free any allocated data in the queues
    if (park_queue.profiles != NULL) {
        for (uint8_t i = 0; i < park_queue.capacity; i++) {
            if (park_queue.profiles[i].data != NULL) {
                DATA_free(park_queue.profiles[i].data);
                park_queue.profiles[i].data = NULL;
            }
        }
        vPortFree(park_queue.profiles);
        park_queue.profiles = NULL;
        ARTEMIS_DEBUG_PRINTF("MEMORY: Park queue cleaned up\n");
    }
    
    if (prof_queue.profiles != NULL) {
        for (uint8_t i = 0; i < prof_queue.capacity; i++) {
            if (prof_queue.profiles[i].data != NULL) {
                DATA_free(prof_queue.profiles[i].data);
                prof_queue.profiles[i].data = NULL;
            }
        }
        vPortFree(prof_queue.profiles);
        prof_queue.profiles = NULL;
        ARTEMIS_DEBUG_PRINTF("MEMORY: Profile queue cleaned up\n");
    }
    
    // Reset queue structures
    park_queue.capacity = 0;
    park_queue.count = 0;
    park_queue.head = 0;
    
    prof_queue.capacity = 0;
    prof_queue.count = 0;
    prof_queue.head = 0;
}

/**
 * @brief Add a profile to a transmission queue
 * 
 * @param queue Pointer to the queue
 * @param profile_number The profile number to add
 * @param data Pointer to the profile data
 * @return true if successfully added, false otherwise
 */
bool MEM_queue_add_profile(TransmissionQueue_t *queue, uint8_t profile_number, Data_t *data) {
    if (queue == NULL || queue->profiles == NULL || queue->count >= queue->capacity) {
        return false;
    }
    
    // Calculate the insert position (circular buffer)
    uint8_t insert_pos = (queue->head + queue->count) % queue->capacity;
    
    // Store the profile data
    queue->profiles[insert_pos].profile_number = profile_number;
    queue->profiles[insert_pos].data = data;
    queue->profiles[insert_pos].transmitted = false;
    queue->profiles[insert_pos].attempt_count = 0;
    
    // Increment count
    queue->count++;
    
    ARTEMIS_DEBUG_PRINTF("MEMORY: Added profile %u to queue, count now %u\n", profile_number, queue->count);
    return true;
}

/**
 * @brief Get the next profile to transmit from a queue
 * 
 * @param queue Pointer to the queue
 * @return Pointer to the next profile, or NULL if queue is empty
 */
ProfileData_t* MEM_queue_get_next_profile(TransmissionQueue_t *queue) {
    if (queue == NULL || queue->profiles == NULL || queue->count == 0) {
        return NULL;
    }
    
    // Return the profile at the head of the queue
    return &queue->profiles[queue->head];
}

/**
 * @brief Mark a profile as successfully transmitted and remove it from the queue
 * 
 * @param queue Pointer to the queue
 */
void MEM_queue_mark_transmitted(TransmissionQueue_t *queue) {
    if (queue == NULL || queue->profiles == NULL || queue->count == 0) {
        return;
    }
    
    // Free the data for the transmitted profile
    if (queue->profiles[queue->head].data != NULL) {
        DATA_free(queue->profiles[queue->head].data);
        queue->profiles[queue->head].data = NULL;
    }
    
    // Mark as transmitted
    queue->profiles[queue->head].transmitted = true;
    
    // Move head to next profile and decrement count
    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;
    
    ARTEMIS_DEBUG_PRINTF("MEMORY: Marked profile as transmitted, queue count now %u\n", queue->count);
}

/**
 * @brief Increment the transmission attempt counter for the current profile
 * 
 * @param queue Pointer to the queue
 */
void MEM_queue_increment_attempt(TransmissionQueue_t *queue) {
    if (queue == NULL || queue->profiles == NULL || queue->count == 0) {
        return;
    }
    
    queue->profiles[queue->head].attempt_count++;
    ARTEMIS_DEBUG_PRINTF("MEMORY: Incremented attempt count to %u for profile %u\n", 
                        queue->profiles[queue->head].attempt_count,
                        queue->profiles[queue->head].profile_number);
}

/**
 * @brief Check if the current profile has reached maximum transmission attempts
 * 
 * @param queue Pointer to the queue
 * @param max_attempts Maximum number of attempts allowed
 * @return true if max attempts reached, false otherwise
 */
bool MEM_queue_max_attempts_reached(TransmissionQueue_t *queue, uint8_t max_attempts) {
    if (queue == NULL || queue->profiles == NULL || queue->count == 0) {
        return false;
    }
    
    return (queue->profiles[queue->head].attempt_count >= max_attempts);
}

/**
 * @brief Get the number of profiles in the queue
 * 
 * @param queue Pointer to the queue
 * @return Number of profiles in the queue
 */
uint8_t MEM_queue_get_count(TransmissionQueue_t *queue) {
    if (queue == NULL) {
        return 0;
    }
    return queue->count;
}

/**
 * @brief Check if available memory is below the low threshold
 * 
 * @return true if memory is low, false otherwise
 */
bool MEM_is_memory_low(void) {
    uint32_t free_memory = xPortGetFreeHeapSize();
    return (free_memory < MEMORY_LOW_THRESHOLD);
}

/**
 * @brief Check if available memory is below the critical threshold
 * 
 * @return true if memory is critically low, false otherwise
 */
bool MEM_is_memory_critical(void) {
    uint32_t free_memory = xPortGetFreeHeapSize();
    return (free_memory < MEMORY_CRITICAL_THRESHOLD);
}

/**
 * @brief Remove the oldest profile from a queue to free memory
 * 
 * @param queue Pointer to the queue
 * @return true if a profile was removed, false otherwise
 */
bool MEM_queue_remove_oldest(TransmissionQueue_t *queue) {
    if (queue == NULL || queue->profiles == NULL || queue->count == 0) {
        return false;
    }
    
    // Log that we're dropping a profile due to memory constraints
    ARTEMIS_DEBUG_PRINTF("MEMORY: Removing oldest profile %u due to low memory\n", 
                         queue->profiles[queue->head].profile_number);
    
    // Free the data for the oldest profile
    if (queue->profiles[queue->head].data != NULL) {
        DATA_free(queue->profiles[queue->head].data);
        queue->profiles[queue->head].data = NULL;
    }
    
    // Remove the profile from the queue
    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;
    
    return true;
}

/**
 * @brief Manage memory before attempting an allocation
 * * Checks if memory is sufficient for the required size OR if general memory is low.
 * Removes profiles from queues if necessary until enough space is freed for the 
 * required allocation plus a critical buffer, or queues are empty.
 * * @param required_size Size of the memory allocation that will be attempted
 * @param primary_queue Pointer to the primary queue to remove from first
 * @param secondary_queue Pointer to the secondary queue to remove from if necessary
 */
 void MEM_manage_memory_before_allocation(size_t required_size, 
    TransmissionQueue_t *primary_queue, 
    TransmissionQueue_t *secondary_queue) {
        
    uint32_t current_free_memory = xPortGetFreeHeapSize();
    uint32_t target_free_memory = required_size + MEMORY_CRITICAL_THRESHOLD; // Target: space for allocation + critical buffer

    bool needs_cleanup = false;

    // Check 1: Is there definitely not enough space for the required allocation + critical buffer?
    if (current_free_memory < target_free_memory) {
        ARTEMIS_DEBUG_PRINTF("MEMORY: Insufficient free memory (%u bytes) for request (%u bytes + %u buffer). Cleanup required.\n", 
        current_free_memory, required_size, MEMORY_CRITICAL_THRESHOLD);
        needs_cleanup = true;
    } 
    
    // Check 2: Even if enough for current request, is general memory below the LOW threshold?
    else if (MEM_is_memory_low()) { // MEM_is_memory_low() checks < MEMORY_LOW_THRESHOLD
        ARTEMIS_DEBUG_PRINTF("MEMORY: Low memory detected (%u bytes free, threshold %u). Opportunistic cleanup.\n", 
        current_free_memory, MEMORY_LOW_THRESHOLD);
        needs_cleanup = true; 
        // In this case, the target is just to get above the critical threshold, 
        // as we already know the required_size fits.
        // We could potentially adjust target_free_memory here if desired, e.g., 
        // target_free_memory = MEMORY_LOW_THRESHOLD; // Aim to get back above LOW threshold
    }

    // If cleanup is needed (either not enough space OR general low memory)
    if (needs_cleanup) {

        ARTEMIS_DEBUG_PRINTF("MEMORY: Starting cleanup. Target free memory: %u bytes.\n", target_free_memory);

        // Loop while free memory is below the target AND there are profiles to remove
        while ((xPortGetFreeHeapSize() < target_free_memory) && 
        (MEM_queue_get_count(primary_queue) > 0 || 
        MEM_queue_get_count(secondary_queue) > 0)) {

            bool removed_primary = false;
            // Prioritize removing from primary queue
            if (MEM_queue_get_count(primary_queue) > 0) {
                removed_primary = MEM_queue_remove_oldest(primary_queue);
            } 

            // If primary was empty or removal failed (shouldn't happen if count > 0), try secondary
            if (!removed_primary && MEM_queue_get_count(secondary_queue) > 0) {
                MEM_queue_remove_oldest(secondary_queue);
            }
        }

        // Log the result after cleanup attempt
        ARTEMIS_DEBUG_PRINTF("MEMORY: Cleanup finished. %u bytes free (Target was %u).\n", 
        xPortGetFreeHeapSize(), target_free_memory);

        // Final check if allocation is still likely to fail
        if(xPortGetFreeHeapSize() < required_size) {
            ARTEMIS_DEBUG_PRINTF("MEMORY: WARNING - Still insufficient memory for required size %u after cleanup!\n", required_size);
        }
    } else {
        ARTEMIS_DEBUG_PRINTF("MEMORY: Sufficient memory (%u bytes free) for request (%u bytes). No cleanup needed.\n",
        current_free_memory, required_size);
    }
}

/**
 * @brief Log the current memory status for debugging
 * 
 * @param location A string indicating where in the code this was called from
 * @param park_queue Pointer to the park queue
 * @param prof_queue Pointer to the profile queue
 */
void MEM_log_memory_status(const char* location, 
                           TransmissionQueue_t *park_queue, 
                           TransmissionQueue_t *prof_queue) {
    uint32_t free_memory = xPortGetFreeHeapSize();
    uint32_t min_free_memory = xPortGetMinimumEverFreeHeapSize();
    
    ARTEMIS_DEBUG_PRINTF("%s: Memory status - Free: %u bytes, Min ever free: %u bytes\n", 
                         location, free_memory, min_free_memory);
    ARTEMIS_DEBUG_PRINTF("%s: Queue stats - Park: %u profiles, Profile: %u profiles\n",
                         location, MEM_queue_get_count(park_queue), 
                         MEM_queue_get_count(prof_queue));
}

/**
 * @brief Reset the transmission attempt counter for the current profile in a queue
 * 
 * @param queue Pointer to the queue
 * @return true if successful, false otherwise
 */
 bool MEM_queue_reset_attempts(TransmissionQueue_t *queue) {
    if (queue == NULL || queue->profiles == NULL || queue->count == 0) {
        return false;
    }
    
    // Reset the attempt counter for the current head profile
    queue->profiles[queue->head].attempt_count = 0;
    
    ARTEMIS_DEBUG_PRINTF("MEMORY: Reset attempt count for profile %u\n", 
                        queue->profiles[queue->head].profile_number);
    
    return true;
}