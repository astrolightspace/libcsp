
#include <csp/arch/csp_time.h>

#include <windows.h>

uint32_t csp_get_ms(void) {
    /* GetTickCount64 returns milliseconds since system start */
    return (uint32_t)(GetTickCount64() & 0xFFFFFFFF);
}

uint32_t csp_get_ms_isr(void) {
    return csp_get_ms();
}

uint32_t csp_get_s(void) {
    return (uint32_t)(GetTickCount64() / 1000);
}

uint32_t csp_get_s_isr(void) {
    return csp_get_s();
}
