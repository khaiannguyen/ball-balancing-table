#ifndef SEQLOCK_H
#define SEQLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_compiler.h"   // __DMB()

typedef struct {
    volatile uint32_t seq;   // Odd = write in progress, even = stable
} seqlock_t;

static inline void seqlock_init(seqlock_t *lock) { lock->seq = 0; }

/* Start a write operation. */
static inline void seqlock_write_begin(seqlock_t *lock) {
    lock->seq++;      // Switch to odd sequence number
    __DMB();
}

/* Finish a write operation. */
static inline void seqlock_write_end(seqlock_t *lock) {
    __DMB();
    lock->seq++;      // Switch back to even sequence number
}

/* Start a read operation and return the initial sequence number. */
static inline uint32_t seqlock_read_begin(const seqlock_t *lock) {
    uint32_t s;
    do {
        s = lock->seq;
    } while (s & 1u);   // Wait while a write is in progress
    __DMB();
    return s;
}

/* Check whether the data changed during the read operation. */
static inline bool seqlock_read_retry(const seqlock_t *lock, uint32_t start_seq) {
    __DMB();
    return (lock->seq != start_seq);
}

#endif
