#include "../../csp_semaphore.h"

#include <csp/csp.h>
#include <csp/csp_debug.h>

#include <windows.h>

void csp_bin_sem_init(csp_bin_sem_t * sem) {
    /* Create an auto-reset event, initially signaled (unlocked) */
    *sem = CreateEvent(NULL, FALSE, TRUE, NULL);
}

int csp_bin_sem_wait(csp_bin_sem_t * sem, unsigned int timeout) {

    DWORD timeout_ms;
    DWORD ret;

    if (timeout == CSP_MAX_TIMEOUT) {
        timeout_ms = INFINITE;
    } else {
        timeout_ms = timeout;
    }

    ret = WaitForSingleObject(*sem, timeout_ms);

    if (ret == WAIT_OBJECT_0) {
        return CSP_SEMAPHORE_OK;
    }

    return CSP_SEMAPHORE_ERROR;
}

int csp_bin_sem_post(csp_bin_sem_t * sem) {

    if (SetEvent(*sem)) {
        return CSP_SEMAPHORE_OK;
    }

    return CSP_SEMAPHORE_ERROR;
}
