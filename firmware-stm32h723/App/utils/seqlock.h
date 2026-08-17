#ifndef SEQLOCK_H
#define SEQLOCK_H
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_compiler.h"   // cho __DMB()

typedef struct {
    volatile uint32_t seq;   // số lẻ = đang ghi, số chẵn = ổn định
} seqlock_t;

static inline void seqlock_init(seqlock_t *lock) { lock->seq = 0; }

// Gọi ở ĐẦU đoạn ghi (chỉ 1 writer duy nhất cho mỗi seqlock)
static inline void seqlock_write_begin(seqlock_t *lock) {
    lock->seq++;      // chuyển sang số lẻ
    __DMB();
}

// Gọi ở CUỐI đoạn ghi
static inline void seqlock_write_end(seqlock_t *lock) {
    __DMB();
    lock->seq++;      // chuyển về số chẵn
}

// Reader: gọi trước khi copy dữ liệu, trả về giá trị seq để so sánh lại
static inline uint32_t seqlock_read_begin(const seqlock_t *lock) {
    uint32_t s;
    do {
        s = lock->seq;
    } while (s & 1u);   // đợi nếu đang giữa lúc ghi (số lẻ)
    __DMB();
    return s;
}

// Reader: gọi sau khi copy xong, trả true nếu phải đọc lại (bị ghi đè giữa chừng)
static inline bool seqlock_read_retry(const seqlock_t *lock, uint32_t start_seq) {
    __DMB();
    return (lock->seq != start_seq);
}

#endif
