#include <csp/csp_hooks.h>

#include <windows.h>

uint32_t csp_memfree_hook(void) {
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    if (GlobalMemoryStatusEx(&status)) {
        /* Return available physical memory, capped to uint32_t max */
        if (status.ullAvailPhys > 0xFFFFFFFF) {
            return 0xFFFFFFFF;
        }
        return (uint32_t)status.ullAvailPhys;
    }
    return 0;
}

unsigned int csp_ps_hook(csp_packet_t * packet) {
    /* Process listing not implemented on Windows */
    (void)packet;
    return 0;
}

void csp_reboot_hook(void) {
    /* Reboot requires elevated privileges on Windows */
    /* Could use ExitWindowsEx(EWX_REBOOT, 0) with proper privileges */
}

void csp_shutdown_hook(void) {
    /* Shutdown requires elevated privileges on Windows */
    /* Could use ExitWindowsEx(EWX_SHUTDOWN, 0) with proper privileges */
}
