# Jetson `TaskControlLoop` jitter & WCET (mục 1.3)

## Tóm tắt

> Jetson TaskControlLoop: already uses absolute-time sleep_until (no bug found) — jitter 17.59 µs std, WCET 6.34 µs mean / 31.01 µs max (0.06% mean / 0.31% max of 10ms budget), N=2000 cycles — see `validation/01-timing/jetson-control-loop/`.

## Xác nhận từ đọc code (trước khi đo)

Đối chiếu `task_control_loop.cpp` với mẫu chuẩn của `task_can_tx.cpp`: vòng lặp chính dùng đúng `sleep_until(next_wake)` với `next_wake += period` — cơ chế absolute-time tự sửa lệch (self-correcting), không tích luỹ drift. **Không có bug cùng loại với STM32** (STM32 dùng `osDelay` tương đối, đã sửa ở mục 1.1). Vì vậy mục này chỉ đo xác nhận 1 lần, không có before/after.

## Phương pháp đo

- **Công cụ:** `clock_gettime(CLOCK_MONOTONIC)`, đơn vị nanosecond, quy đổi ra µs khi phân tích (`/1000`).
- **Period (jitter):** ghi ở đầu thân vòng lặp, khoảng cách giữa 2 lần vào loop liên tiếp.
- **WCET:** `t0` đầu thân vòng lặp, `t1` — xem lưu ý quan trọng bên dưới.
- **Buffer:** `std::vector<uint64_t>` đã `reserve(2000)` trước vòng lặp thời gian thực, dump ra CSV **một lần duy nhất** (dùng cờ `static bool`) sau khi đủ N=2000 mẫu, không I/O nào khác trong hot path (task chạy `SCHED_FIFO`).
- **Điều kiện đo:** chạy hệ thống thật đầy đủ (camera + CAN + PID + watchdog + video) trong ~25 giây, `can0` up, **không cần đặt bóng lên bàn** — camera nhìn khung hình trống (`detected=0` suốt quá trình), đúng như thiết kế: `TaskControlLoop` vẫn chạy đúng chu kỳ không phụ thuộc có bóng hay không.

## ⚠️ Lưu ý quan trọng — sai lệch so với đặc tả gốc cho mốc t1 của WCET

Đặc tả gốc yêu cầu `t1` đặt "ngay sau khi gửi xong CAN (`can_send`/tương đương)". Tuy nhiên khi đọc code thật, phát hiện: **`task_control_loop.cpp` không gọi trực tiếp hàm gửi CAN nào** — nó chỉ ghi giá trị PID output vào `system_state().attitude_desired` qua hàm `attitude_desired_write()`. Việc gửi CAN thật sự diễn ra ở **task riêng biệt `TaskCanTx`**, không đồng bộ (chạy độc lập, không nằm trong cùng chu kỳ của `TaskControlLoop`).

Vì vậy, mốc `t1` trong phép đo này dùng **`attitude_desired_write()`** — điểm hand-off CAN duy nhất tồn tại trong file `task_control_loop.cpp`. Điều này có nghĩa:

- Số liệu WCET dưới đây phản ánh đúng **thời gian thực thi nội bộ của `TaskControlLoop`** (đọc cảm biến → tính PID → ghi setpoint), **không bao gồm** thời gian truyền CAN vật lý thật sự (việc đó thuộc về `TaskCanTx`, cần đo riêng nếu muốn biết WCET đầu-cuối của toàn bộ đường truyền CAN).
- Đây là cách diễn giải hợp lý nhất trong giới hạn của file này, nhưng khác với chữ "can_send" trong đặc tả gốc — ghi rõ ở đây để người đọc sau không hiểu nhầm là đã đo trọn vẹn thời gian CAN.

## Kết quả (N = 2000 mẫu)

| | Period (jitter) | WCET |
|---|---|---|
| mean (µs) | 9999.946 | 6.336 |
| std (µs) | 17.591 | 1.144 |
| min (µs) | 9785.273 | 1.600 |
| max (µs) | 10216.203 | 31.009 |
| % của ngân sách 10ms | — | mean 0.063%, max 0.310% |

## Đánh giá

- **Period trung bình ≈ 10000 µs**, đúng khớp `CONTROL_LOOP_PERIOD_MS`. `std = 17.59 µs` (0.18% chu kỳ) — nhỏ, xác nhận cơ chế `sleep_until` hoạt động đúng như thiết kế, không có drift tích luỹ trong cửa sổ đo 2000 chu kỳ (~20 giây).
- **WCET rất nhỏ** so với ngân sách: trung bình chỉ chiếm 0.063%, tệ nhất (max) cũng chỉ 0.31% của 10ms — dư địa thời gian rất lớn cho phần xử lý nội bộ của `TaskControlLoop`. Không loại trừ khả năng WCET đầu-cuối thật sự (bao gồm cả `TaskCanTx` gửi CAN) cao hơn con số này — xem lưu ý ở trên.
- Không phát hiện bug jitter kiểu STM32 (`osDelay`) — kết quả này xác nhận đúng nhận định ban đầu trong file kế hoạch, không cần sửa code.

## Bằng chứng

```
validation/01-timing/jetson-control-loop/
├── README.md
├── control_loop_period_ns.csv
├── control_loop_wcet_ns.csv
└── control_loop_timing_summary.txt   (hoặc .md, tuỳ tên Claude Code đã lưu)
```

File CSV gốc và summary hiện đang ở `~/Downloads` trên Jetson — copy vào đúng thư mục trên trước khi commit.

## Ghi chú thay đổi code

`src/task_control_loop.cpp` đã được sửa để thêm instrumentation đo timing (2 buffer `std::vector`, dump CSV một lần khi đủ N=2000 mẫu) — thay đổi này **chưa commit**. Cân nhắc: giữ lại instrumentation này trong code (có thể bọc trong `#ifdef VALIDATION_...` giống style `VALIDATION_DEADLOCK_TEST_CONTROL` đã có ở STM32) để tái đo dễ dàng sau này, hoặc gỡ bỏ nếu chỉ cần đo 1 lần cho việc validation này.
