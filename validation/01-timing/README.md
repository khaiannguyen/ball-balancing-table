# 1 — Real-time scheduling & control-loop timing

Tổng hợp kết quả đo timing cho STM32H723 (control loop chính) và Jetson (`TaskControlLoop`). Chi tiết từng phần xem README riêng trong mỗi thư mục con.

## Tóm tắt nhanh

| Mục | Nền tảng | Bug tìm thấy | Kết quả chính |
|---|---|---|---|
| [1.1 — Jitter](./control-loop-jitter/README.md) | STM32H723 | ✅ `osDelay` (relative-delay) ở 3 vị trí — đã sửa thành `vTaskDelayUntil` | jitter std: 4.12 µs (before) → 8.67 µs (after), N=2048 |
| [1.2 — WCET](./control-loop-wcet/README.md) | STM32H723 | — (tái dùng hạ tầng 1.1) | WCET 7.88 µs (0.08% ngân sách 10ms), **chỉ đo được Home mode** |
| [1.3 — Jitter & WCET](./jetson-control-loop/README.md) | Jetson `TaskControlLoop` | ❌ Không có bug — đã dùng đúng `sleep_until` absolute-time | jitter std 17.59 µs, WCET mean 6.34 µs / max 31.01 µs (0.06%/0.31%) |

---

## 1.1 — STM32 Control Loop Jitter: Before vs After

![Jitter before vs after](./assets/chart1_jitter_before_after.png)

Sau khi sửa `osDelay` → `vTaskDelayUntil` (3 vị trí trong `task_control_loop.c`), std thực đo **tăng** từ 4.12 µs lên 8.67 µs trong cửa sổ đo ngắn (~20 giây, N=2048) — ngược với giả thuyết ban đầu. Cả hai giá trị đều rất nhỏ so với chu kỳ 10ms (dưới 0.1%). Đây là phát hiện hợp lệ, ghi trung thực — không ép số liệu. `vTaskDelayUntil` vẫn là lựa chọn đúng về lý thuyết để chống trôi tích luỹ dài hạn, một hiệu ứng phép đo ngắn này không đo được. Chi tiết: [`control-loop-jitter/README.md`](./control-loop-jitter/README.md).

---

## 1.2 — STM32 Control Loop WCET (Home mode)

![WCET STM32](./assets/chart2_wcet_stm32.png)

WCET đo được **7.88 µs**, chỉ chiếm **0.08%** ngân sách 10ms — dư địa thời gian rất lớn. ⚠️ **Giới hạn quan trọng:** chỉ đo được ở **Home mode** (mode mặc định lúc boot) — không chuyển được sang Balance/Position do thiết kế chương trình hiện tại, nên câu hỏi "mode nào tốn nhất" còn để ngỏ. Không được suy diễn số liệu Home mode sang các mode khác. Chi tiết: [`control-loop-wcet/README.md`](./control-loop-wcet/README.md).

---

## 1.3 — Jetson TaskControlLoop: Jitter & WCET

![Jetson jitter và WCET](./assets/chart3_jetson.png)

Xác nhận code Jetson **không có bug** — `sleep_until(next_wake)` với `next_wake += period` đã đúng chuẩn absolute-time từ đầu, giống `task_can_tx.cpp`. Period trung bình ≈ 10000 µs khớp thiết kế, std 17.59 µs. WCET rất nhỏ (mean 6.34 µs, max 31.01 µs). ⚠️ **Lưu ý:** mốc kết thúc WCET dùng `attitude_desired_write()` thay vì gửi CAN thật (việc gửi CAN nằm ở task riêng `TaskCanTx`, không đồng bộ với `TaskControlLoop`) — số liệu phản ánh đúng thời gian xử lý nội bộ, không bao gồm thời gian truyền CAN đầu-cuối. Chi tiết: [`jetson-control-loop/README.md`](./jetson-control-loop/README.md).

---

## So sánh tổng thể: WCET so với ngân sách 10ms

![So sánh WCET tổng thể](./assets/chart4_overview.png)

Cả 2 nền tảng đều có dư địa thời gian rất lớn so với ngân sách 10ms của control loop — không có nguy cơ vi phạm deadline ở các phép đo đã thực hiện. Điểm cần lưu ý là cả 2 phép đo WCET đều **chưa đầy đủ**: STM32 thiếu Balance/Position mode, Jetson thiếu thời gian truyền CAN thật sự qua `TaskCanTx`.

---

## Việc còn để ngỏ (chưa hoàn thành 100%)

- [ ] Đo WCET STM32 ở Balance mode và Position mode (hiện chỉ có Home mode).
- [ ] Đo WCET đầu-cuối thật sự của đường CAN trên Jetson, bao gồm cả `TaskCanTx` (hiện chỉ đo tới `attitude_desired_write()`).
- [ ] Quyết định giữ lại hay revert code instrumentation đã thêm vào `task_control_loop.cpp` (Jetson) — hiện đang sửa nhưng chưa commit.

## Cấu trúc thư mục

```
validation/01-timing/
├── README.md                      <- file này (tổng hợp)
├── assets/
│   ├── chart1_jitter_before_after.png
│   ├── chart2_wcet_stm32.png
│   ├── chart3_jetson.png
│   └── chart4_overview.png
├── control-loop-jitter/           (STM32, mục 1.1)
│   ├── README.md
│   ├── before/
│   └── after/
├── control-loop-wcet/             (STM32, mục 1.2)
│   ├── README.md
│   ├── wcet_home_raw.bin
│   └── wcet_home_raw_summary.csv
└── jetson-control-loop/           (Jetson, mục 1.3)
    ├── README.md
    ├── control_loop_period_ns.csv
    └── control_loop_wcet_ns.csv
```
