/*
   Queue implementation using Windows synchronization primitives.
   Similar to pthread_queue but using CRITICAL_SECTION and CONDITION_VARIABLE.
*/

#include "windows_queue.h"

#include <stdlib.h>
#include <string.h>

#include <csp/csp.h>

windows_queue_t * windows_queue_create(int length, size_t item_size) {

    windows_queue_t * q = malloc(sizeof(windows_queue_t));

    if (q != NULL) {
        q->buffer = malloc(length * item_size);
        if (q->buffer != NULL) {
            q->size = length;
            q->item_size = (int)item_size;
            q->items = 0;
            q->in = 0;
            q->out = 0;
            InitializeCriticalSection(&q->mutex);
            InitializeConditionVariable(&q->cond_full);
            InitializeConditionVariable(&q->cond_empty);
        } else {
            free(q);
            q = NULL;
        }
    }

    return q;
}

void windows_queue_delete(windows_queue_t * q) {

    if (q == NULL)
        return;

    DeleteCriticalSection(&q->mutex);
    free(q->buffer);
    free(q);
}

static inline int wait_slot_available(windows_queue_t * queue, DWORD timeout_ms) {

    while (queue->items == queue->size) {
        if (!SleepConditionVariableCS(&queue->cond_full, &queue->mutex, timeout_ms)) {
            if (GetLastError() == ERROR_TIMEOUT) {
                return WINDOWS_QUEUE_FULL;
            }
            /* Other error, retry */
        }
    }

    return WINDOWS_QUEUE_OK;
}

int windows_queue_enqueue(windows_queue_t * queue, const void * value, uint32_t timeout) {

    int ret;
    DWORD timeout_ms;

    if (timeout == CSP_MAX_TIMEOUT) {
        timeout_ms = INFINITE;
    } else {
        timeout_ms = timeout;
    }

    EnterCriticalSection(&queue->mutex);

    ret = wait_slot_available(queue, timeout_ms);
    if (ret == WINDOWS_QUEUE_OK) {
        memcpy((char *)queue->buffer + (queue->in * queue->item_size), value, queue->item_size);
        queue->items++;
        queue->in = (queue->in + 1) % queue->size;
    }

    LeaveCriticalSection(&queue->mutex);

    if (ret == WINDOWS_QUEUE_OK) {
        WakeAllConditionVariable(&queue->cond_empty);
    }

    return ret;
}

static inline int wait_item_available(windows_queue_t * queue, DWORD timeout_ms) {

    while (queue->items == 0) {
        if (!SleepConditionVariableCS(&queue->cond_empty, &queue->mutex, timeout_ms)) {
            if (GetLastError() == ERROR_TIMEOUT) {
                return WINDOWS_QUEUE_EMPTY;
            }
            /* Other error, retry */
        }
    }

    return WINDOWS_QUEUE_OK;
}

int windows_queue_dequeue(windows_queue_t * queue, void * buf, uint32_t timeout) {

    int ret;
    DWORD timeout_ms;

    if (!queue) {
        csp_print("csp not initialized\n");
        return WINDOWS_QUEUE_ERROR;
    }

    if (timeout == CSP_MAX_TIMEOUT) {
        timeout_ms = INFINITE;
    } else {
        timeout_ms = timeout;
    }

    EnterCriticalSection(&queue->mutex);

    ret = wait_item_available(queue, timeout_ms);
    if (ret == WINDOWS_QUEUE_OK) {
        memcpy(buf, (char *)queue->buffer + (queue->out * queue->item_size), queue->item_size);
        queue->items--;
        queue->out = (queue->out + 1) % queue->size;
    }

    LeaveCriticalSection(&queue->mutex);

    if (ret == WINDOWS_QUEUE_OK) {
        WakeAllConditionVariable(&queue->cond_full);
    }

    return ret;
}

int windows_queue_items(windows_queue_t * queue) {

    EnterCriticalSection(&queue->mutex);
    int items = queue->items;
    LeaveCriticalSection(&queue->mutex);

    return items;
}

int windows_queue_free(windows_queue_t * queue) {

    EnterCriticalSection(&queue->mutex);
    int free_slots = queue->size - queue->items;
    LeaveCriticalSection(&queue->mutex);

    return free_slots;
}

void windows_queue_empty(windows_queue_t * queue) {

    EnterCriticalSection(&queue->mutex);
    queue->items = 0;
    queue->in = 0;
    queue->out = 0;
    LeaveCriticalSection(&queue->mutex);
}
