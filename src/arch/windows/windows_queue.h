#pragma once

/**
   @file

   Queue implemented using Windows synchronization primitives.
   Similar to pthread_queue but using CRITICAL_SECTION and CONDITION_VARIABLE.
*/

#include <stdint.h>
#include <stddef.h>
#include <windows.h>

#include <csp/arch/csp_queue.h>

/**
   Queue error codes.
   @{
*/
#define WINDOWS_QUEUE_ERROR CSP_QUEUE_ERROR
#define WINDOWS_QUEUE_EMPTY CSP_QUEUE_ERROR
#define WINDOWS_QUEUE_FULL CSP_QUEUE_ERROR
#define WINDOWS_QUEUE_OK CSP_QUEUE_OK
/** @} */

/**
   Queue handle.
*/
typedef struct windows_queue_s {
    void * buffer;              /**< Memory area */
    int size;                   /**< Memory size (number of items) */
    int item_size;              /**< Item/element size */
    int items;                  /**< Items/elements in queue */
    int in;                     /**< Insert point */
    int out;                    /**< Extract point */
    CRITICAL_SECTION mutex;     /**< Lock */
    CONDITION_VARIABLE cond_full;   /**< Wait because queue is full (insert) */
    CONDITION_VARIABLE cond_empty;  /**< Wait because queue is empty (extract) */
} windows_queue_t;

/**
   Create queue.
*/
windows_queue_t * windows_queue_create(int length, size_t item_size);

/**
   Delete queue.
*/
void windows_queue_delete(windows_queue_t * q);

/**
   Enqueue/insert element.
*/
int windows_queue_enqueue(windows_queue_t * queue, const void * value, uint32_t timeout);

/**
   Dequeue/extract element.
*/
int windows_queue_dequeue(windows_queue_t * queue, void * buf, uint32_t timeout);

/**
   Return number of elements in the queue.
*/
int windows_queue_items(windows_queue_t * queue);

/**
   Return number of free slots in the queue.
*/
int windows_queue_free(windows_queue_t * queue);

/**
   Empty the queue.
*/
void windows_queue_empty(windows_queue_t * queue);
