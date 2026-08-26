#ifndef SEQLOCK_HPP
#define SEQLOCK_HPP

#include <atomic>
#include <cstdint>

/*
 * Lightweight sequence lock for protecting small shared data structures.
 *
 * The sequence number is:
 *   - even: data is stable
 *   - odd:  a write operation is in progress
 *
 * Writers increment the sequence before and after modifying the protected
 * data. Readers capture the sequence before reading and retry if the value
 * changes during the read.
 *
 * std::atomic and C++ memory ordering provide the synchronization required
 * between Jetson threads.
 */
struct seqlock_t
{
    std::atomic<uint32_t> seq{ 0 };
};

/*
 * Initializes the sequence to the stable state.
 */
inline void seqlock_init(
    seqlock_t& lock)
{
    lock.seq.store(
        0,
        std::memory_order_relaxed
    );
}

/*
 * Marks the beginning of a write operation.
 *
 * The sequence becomes odd while the protected data is being modified.
 */
inline void seqlock_write_begin(
    seqlock_t& lock)
{
    uint32_t s =
        lock.seq.load(
            std::memory_order_relaxed
        );

    lock.seq.store(
        s + 1,
        std::memory_order_release
    );
}

/*
 * Marks the end of a write operation.
 *
 * The sequence becomes even again, indicating that the protected data
 * represents a complete update.
 */
inline void seqlock_write_end(
    seqlock_t& lock)
{
    uint32_t s =
        lock.seq.load(
            std::memory_order_relaxed
        );

    lock.seq.store(
        s + 1,
        std::memory_order_release
    );
}

/*
 * Captures the sequence number for a read operation.
 *
 * If a writer is active, the sequence is odd and the reader waits until
 * the writer completes.
 */
inline uint32_t seqlock_read_begin(
    const seqlock_t& lock)
{
    uint32_t s;

    do
    {
        s =
            lock.seq.load(
                std::memory_order_acquire
            );
    } while (s & 1u);

    return s;
}

/*
 * Checks whether the protected data changed while it was being read.
 *
 * The caller must retry the complete read operation when this returns true.
 */
inline bool seqlock_read_retry(
    const seqlock_t& lock,
    uint32_t start_seq)
{
    return lock.seq.load(
        std::memory_order_acquire
    ) != start_seq;
}

#endif