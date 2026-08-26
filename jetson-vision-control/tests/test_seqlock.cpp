#include "seqlock.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

struct payload_t
{
    seqlock_t lock;

    int32_t a;
    int32_t b;
};

static int g_failures = 0;

#define CHECK(cond, msg)             \
    do                               \
    {                                \
        if (!(cond))                 \
        {                            \
            std::printf(             \
                "[FAIL] %s (line %d)\n", \
                msg,                 \
                __LINE__             \
            );                       \
            g_failures++;            \
        }                            \
    } while (0)

static void test_single_thread()
{
    std::printf(
        "== Test 1: single-thread write/read ==\n"
    );

    payload_t p;

    seqlock_init(
        p.lock
    );

    CHECK(
        p.lock.seq.load() == 0,
        "Sequence number must start at 0 (even)"
    );

    seqlock_write_begin(
        p.lock
    );

    CHECK(
        (p.lock.seq.load() & 1u) == 1u,
        "Sequence number must be odd while writing"
    );

    p.a = 42;
    p.b = 84;

    seqlock_write_end(
        p.lock
    );

    CHECK(
        (p.lock.seq.load() & 1u) == 0u,
        "Sequence number must be even after the write completes"
    );

    uint32_t start =
        seqlock_read_begin(
            p.lock
        );

    int32_t a = p.a;
    int32_t b = p.b;

    bool retry =
        seqlock_read_retry(
            p.lock,
            start
        );

    CHECK(
        !retry,
        "Read must not require a retry when no write occurs concurrently"
    );

    CHECK(
        a == 42 && b == 84,
        "Read values must match the values written"
    );

    std::printf(
        "Test 1 done.\n\n"
    );
}

static void test_concurrent()
{
    std::printf(
        "== Test 2: concurrent writer + readers (1 second) ==\n"
    );

    payload_t p;

    seqlock_init(
        p.lock
    );

    p.a = 0;
    p.b = 0;

    std::atomic<bool> stop{
        false
    };

    std::atomic<uint64_t> reads_ok{
        0
    };

    std::atomic<uint64_t> reads_retried{
        0
    };

    std::atomic<uint64_t> corrupt_seen{
        0
    };

    std::thread writer(
        [&]()
        {
            int32_t v = 0;

            while (
                !stop.load(
                    std::memory_order_relaxed
                ))
            {
                v++;

                seqlock_write_begin(
                    p.lock
                );

                p.a = v;
                p.b = v * 2;

                seqlock_write_end(
                    p.lock
                );
            }
        }
    );

    auto reader_fn =
        [&]()
    {
        while (
            !stop.load(
                std::memory_order_relaxed
            ))
        {
            uint32_t start =
                seqlock_read_begin(
                    p.lock
                );

            int32_t a = p.a;
            int32_t b = p.b;

            if (
                seqlock_read_retry(
                    p.lock,
                    start
                ))
            {
                reads_retried.fetch_add(
                    1,
                    std::memory_order_relaxed
                );

                continue;
            }

            reads_ok.fetch_add(
                1,
                std::memory_order_relaxed
            );

            /*
             * The writer always publishes the invariant b = 2 * a.
             * A successful read must never observe a partially updated
             * payload.
             */
            if (b != a * 2)
            {
                corrupt_seen.fetch_add(
                    1,
                    std::memory_order_relaxed
                );
            }
        }
    };

    std::thread r1(
        reader_fn
    );

    std::thread r2(
        reader_fn
    );

    std::thread r3(
        reader_fn
    );

    /*
     * Run the writer and readers concurrently long enough to exercise
     * repeated sequence-number changes and read retries.
     */
    std::this_thread::sleep_for(
        std::chrono::seconds(1)
    );

    stop.store(
        true,
        std::memory_order_relaxed
    );

    writer.join();

    r1.join();
    r2.join();
    r3.join();

    std::printf(
        "reads_ok=%lu reads_retried=%lu corrupt_seen=%lu\n",
        (unsigned long)reads_ok.load(),
        (unsigned long)reads_retried.load(),
        (unsigned long)corrupt_seen.load()
    );

    CHECK(
        reads_ok.load() > 0,
        "At least one read must complete successfully"
    );

    CHECK(
        corrupt_seen.load() == 0,
        "A successful read must never observe torn data"
    );

    std::printf(
        "Test 2 done.\n\n"
    );
}

int main()
{
    test_single_thread();

    test_concurrent();

    if (g_failures == 0)
    {
        std::printf(
            "=== ALL TESTS PASSED ===\n"
        );

        return 0;
    }

    std::printf(
        "=== %d TEST(S) FAILED ===\n",
        g_failures
    );

    return 1;
}