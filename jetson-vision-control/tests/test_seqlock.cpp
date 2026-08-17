#include "seqlock.hpp"
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <chrono>

struct payload_t {
    seqlock_t lock;
    int32_t a;
    int32_t b;
};

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
        g_failures++; \
    } \
} while (0)

static void test_single_thread() {
    std::printf("== Test 1: single-thread write/read ==\n");
    payload_t p;
    seqlock_init(p.lock);
    CHECK(p.lock.seq.load() == 0, "seq phai bat dau tu 0 (chan)");

    seqlock_write_begin(p.lock);
    CHECK((p.lock.seq.load() & 1u) == 1u, "seq phai le trong luc ghi");
    p.a = 42; p.b = 84;
    seqlock_write_end(p.lock);
    CHECK((p.lock.seq.load() & 1u) == 0u, "seq phai chan sau khi ghi xong");

    uint32_t start = seqlock_read_begin(p.lock);
    int32_t a = p.a, b = p.b;
    bool retry = seqlock_read_retry(p.lock, start);
    CHECK(!retry, "khong duoc retry khi khong co ghi xen vao");
    CHECK(a == 42 && b == 84, "gia tri doc phai dung nhu da ghi");

    std::printf("Test 1 done.\n\n");
}

static void test_concurrent() {
    std::printf("== Test 2: concurrent writer + readers (chay 1 giay) ==\n");
    payload_t p;
    seqlock_init(p.lock);
    p.a = 0; p.b = 0;

    std::atomic<bool> stop{false};
    std::atomic<uint64_t> reads_ok{0};
    std::atomic<uint64_t> reads_retried{0};
    std::atomic<uint64_t> corrupt_seen{0};

    std::thread writer([&]() {
        int32_t v = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            v++;
            seqlock_write_begin(p.lock);
            p.a = v;
            p.b = v * 2;
            seqlock_write_end(p.lock);
        }
    });

    auto reader_fn = [&]() {
        while (!stop.load(std::memory_order_relaxed)) {
            uint32_t start = seqlock_read_begin(p.lock);
            int32_t a = p.a, b = p.b;
            if (seqlock_read_retry(p.lock, start)) {
                reads_retried.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            reads_ok.fetch_add(1, std::memory_order_relaxed);
            if (b != a * 2) {
                corrupt_seen.fetch_add(1, std::memory_order_relaxed);
            }
        }
    };

    std::thread r1(reader_fn), r2(reader_fn), r3(reader_fn);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    stop.store(true, std::memory_order_relaxed);

    writer.join(); r1.join(); r2.join(); r3.join();

    std::printf("reads_ok=%lu reads_retried=%lu corrupt_seen=%lu\n",
                (unsigned long)reads_ok.load(),
                (unsigned long)reads_retried.load(),
                (unsigned long)corrupt_seen.load());

    CHECK(reads_ok.load() > 0, "phai co it nhat vai lan doc thanh cong");
    CHECK(corrupt_seen.load() == 0, "KHONG duoc co du lieu bi xe");

    std::printf("Test 2 done.\n\n");
}

int main() {
    test_single_thread();
    test_concurrent();

    if (g_failures == 0) {
        std::printf("=== ALL TESTS PASSED ===\n");
        return 0;
    } else {
        std::printf("=== %d TEST(S) FAILED ===\n", g_failures);
        return 1;
    }
}