#ifndef SPINLOCK_GUARD_H
#define SPINLOCK_GUARD_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// RAII C++ Wrapper for FreeRTOS Spinlocks
struct CriticalSectionGuard {
    portMUX_TYPE* mux;

    explicit CriticalSectionGuard(portMUX_TYPE& m) : mux(&m) {
        portENTER_CRITICAL(mux);
    }

    explicit CriticalSectionGuard(portMUX_TYPE* m) : mux(m) {
        if (mux) {
            portENTER_CRITICAL(mux);
        }
    }

    ~CriticalSectionGuard() {
        if (mux) {
            portEXIT_CRITICAL(mux);
        }
    }

    // Deleted copy and move constructors to prevent accidental double-unlocks
    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard(CriticalSectionGuard&&) = delete;
    CriticalSectionGuard& operator=(CriticalSectionGuard&&) = delete;
};

#endif