#ifndef SEQLOCK_HPP
#define SEQLOCK_HPP
#include <atomic>
#include <cstdint>

/* Port từ seqlock.h (STM32) sang C++ cho Jetson.
 * Cùng thuật toán hệt bản gốc: seq lẻ = đang ghi, seq chẵn = ổn định.
 * Khác biệt duy nhất: dùng std::atomic<uint32_t> + memory_order thay vì
 * volatile + __DMB() thủ công. */
struct seqlock_t {
    std::atomic<uint32_t> seq{0};
};

inline void seqlock_init(seqlock_t &lock) {
    lock.seq.store(0, std::memory_order_relaxed);
}

inline void seqlock_write_begin(seqlock_t &lock) {
    uint32_t s = lock.seq.load(std::memory_order_relaxed);
    lock.seq.store(s + 1, std::memory_order_release);
}

inline void seqlock_write_end(seqlock_t &lock) {
    uint32_t s = lock.seq.load(std::memory_order_relaxed);
    lock.seq.store(s + 1, std::memory_order_release);
}

inline uint32_t seqlock_read_begin(const seqlock_t &lock) {
    uint32_t s;
    do {
        s = lock.seq.load(std::memory_order_acquire);
    } while (s & 1u);
    return s;
}

inline bool seqlock_read_retry(const seqlock_t &lock, uint32_t start_seq) {
    return lock.seq.load(std::memory_order_acquire) != start_seq;
}

#endif