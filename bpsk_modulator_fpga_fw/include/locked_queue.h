#pragma once

#include <cstdint>
#include <cstddef>
#include "pico/critical_section.h"

template <typename T, size_t SIZE>
class LockedQueue {
private:
    static_assert(sizeof(T) % 4 == 0, "Queue element size must be a multiple of 4 bytes for optimized copying");
    T data[SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    mutable critical_section_t crit_sec;

public:
    LockedQueue() : head(0), tail(0) { critical_section_init(&crit_sec); }
    ~LockedQueue() { critical_section_deinit(&crit_sec); }

    __attribute__((always_inline)) inline bool try_add(const T *item) {
        critical_section_enter_blocking(&crit_sec);
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t next_head = (h + 1) % SIZE;
        if (next_head == t) {
            critical_section_exit(&crit_sec);
            return false;
        }
        uint32_t *dst = reinterpret_cast<uint32_t *>(&data[h]);
        const uint32_t *src = reinterpret_cast<const uint32_t *>(item);
        for (size_t i = 0; i < sizeof(T) / 4; i++)
            dst[i] = src[i];
        __dmb();
        head = next_head;
        critical_section_exit(&crit_sec);
        return true;
    }

    __attribute__((always_inline)) inline bool try_remove(T *item) {
        critical_section_enter_blocking(&crit_sec);
        uint16_t h = head;
        uint16_t t = tail;
        if (h == t) {
            critical_section_exit(&crit_sec);
            return false;
        }
        uint32_t *dst = reinterpret_cast<uint32_t *>(item);
        const uint32_t *src = reinterpret_cast<const uint32_t *>(&data[t]);
        for (size_t i = 0; i < sizeof(T) / 4; i++)
            dst[i] = src[i];
        __dmb();
        tail = (t + 1) % SIZE;
        critical_section_exit(&crit_sec);
        return true;
    }

    __attribute__((always_inline)) inline uint16_t get_level() const {
        critical_section_enter_blocking(&crit_sec);
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t lvl = (h >= t) ? (h - t) : (SIZE - t + h);
        critical_section_exit(&crit_sec);
        return lvl;
    }

    void clear() {
        critical_section_enter_blocking(&crit_sec);
        head = tail = 0;
        critical_section_exit(&crit_sec);
    }
};
