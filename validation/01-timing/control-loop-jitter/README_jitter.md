# Control-loop jitter — STM32H723 (mục 1.1)

## Tóm tắt

> Control loop jitter: 4.12 µs std (osDelay) → 8.67 µs std (vTaskDelayUntil), N=2048 cycles, Balance mode — vTaskDelayUntil did not reduce short-window jitter in this run (both well under 0.1% of the 10ms period); see `validation/01-timing/control-loop-jitter/`.

## Bối cảnh

`task_control_loop.c` dùng `osDelay(CONTROL_LOOP_PERIOD_MS)` ở 3 vị trí trong vòng lặp chính của ControlLoopTask, trong khi `task_watchdog.c` và `task_can_tx.c` đều dùng `vTaskDelayUntil()` (periodic, absolute-time, không tích luỹ trôi). Đây là task quan trọng nhất hệ thống (sinh lệnh servo trực tiếp), nên bug này được ưu tiên sửa và đo trước/sau.

## Phương pháp đo

- **Công cụ:** DWT cycle counter (Cortex-M7, CPU 400MHz → `time_us = cycles / 400`).
- **Vị trí ghi mẫu:** đầu thân vòng lặp `for(;;)` của ControlLoopTask, ghi vào ring buffer tĩnh `s_period_buf[2048]` (`uint32_t`, đơn vị: cycles giữa 2 lần vào vòng lặp liên tiếp).
- **Điều kiện đo:** Balance mode, có tải thật (servo + IMU + CAN hoạt động đồng thời), không đo lúc idle.
- **N mẫu:** 2048 (buffer đầy hoàn toàn — `s_idx == 2048` xác nhận trước khi dump, tránh dump dở buffer).
- **Cách lấy dữ liệu:** halt bằng debugger (CubeIDE) đúng lúc buffer đầy, export vùng nhớ `s_period_buf` qua Memory View → RAW Binary (`.bin`, 8192 bytes = 2048 × 4 bytes), sau đó parse bằng script Python (`analyze.py` / `analyze_after.py`) để tính `mean`/`std`/`min`/`max` theo µs và xuất CSV.
- **Không dùng UART/printf trong hot path** — `retarget.c` dùng `HAL_UART_Transmit(..., HAL_MAX_DELAY)` (blocking thật sự từng byte), có thể phá phép đo nếu lỡ log trong vòng lặp.

## Kết quả

| | before (`osDelay`) | after (`vTaskDelayUntil`) |
|---|---|---|
| N | 2048 | 2048 |
| mean (µs) | 9999.91 | 9999.81 |
| **std (µs)** | **4.12** | **8.67** |
| min (µs) | 9815.57 | 9608.17 |
| max (µs) | 10004.78 | 10004.69 |

File dữ liệu gốc: `before/before_raw.bin`, `before/before_summary.csv`, `after/after_raw.bin`, `after/after_summary.csv`.

## Đánh giá

`vTaskDelayUntil` **không cải thiện jitter** trong cửa sổ đo ngắn này (~20 giây, N=2048) — std thực đo **tăng** so với `osDelay` (8.67 µs vs 4.12 µs), ngược với giả thuyết ban đầu.

Đây là phát hiện hợp lệ, không phải lỗi đo: cả hai giá trị std đều rất nhỏ so với chu kỳ 10ms (0.04% và 0.087%), cho thấy hệ thống ổn định ở cả 2 phiên bản trong khoảng thời gian đo — khác biệt tuyệt đối chỉ vài µs. Chênh lệch có thể do điều kiện tải giữa 2 lần đo không hoàn toàn giống hệt nhau (đo ở 2 phiên debug riêng biệt, cách nhau về thời gian), không loại trừ khả năng nhiễu môi trường đo (cache write-back vào RAM_D1, xem ghi chú buffer bên dưới) đóng góp vào chênh lệch này nhiều hơn bản thân cơ chế delay.

**Về mặt lý thuyết, `vTaskDelayUntil` vẫn là lựa chọn đúng để giữ nguyên** — lợi ích chính của nó là chống **trôi tích luỹ dài hạn** (long-term drift), một hiệu ứng cần thời gian chạy dài hơn nhiều so với ~20 giây của phép đo này mới bộc lộ rõ qua std của period tức thời. Phép đo hiện tại không đủ để bác bỏ lợi ích lý thuyết đó, chỉ cho biết trong cửa sổ ngắn cả hai cách đều chấp nhận được.

## Ghi chú kỹ thuật về buffer đo

Buffer `s_period_buf` dùng mảng tĩnh bình thường (hướng (a) trong kế hoạch), không ép vào DTCM — vì linker script hiện tại của project (`STM32H723ZGTX_FLASH.ld`) không định nghĩa section `.dtcm` trong `SECTIONS{}`, nên attribute `section(".dtcm")` sẽ không thực sự đặt buffer vào DTCM. Buffer nằm trong `RAM_D1` (AXI SRAM), có thể dính cache write-back do `SCB_EnableDCache()` được gọi trong `main.c` — đây là nguồn nhiễu nhỏ nhưng chấp nhận được so với độ lớn jitter ms-scale đang đo.

## Ghi chú vận hành (rút kinh nghiệm khi đo)

- Phải xác nhận `s_idx == 2048` qua **Live Expressions** (không cần halt để xem — CubeIDE cập nhật giá trị này khi board đang chạy) **trước khi** halt để dump. Một số lần dump thử nghiệm cho ra N < 2048 (buffer chưa đầy) đã bị loại bỏ, không dùng trong bảng kết quả trên.
- Cần xoá các breakpoint còn sót lại (ví dụ breakpoint tình cờ nằm trong `tasks.c` của FreeRTOS) trước khi Resume, nếu không chương trình có thể tự dừng giữa chừng ở idle task thay vì chạy liên tục tới khi buffer đầy.
- Sau khi sửa code (`osDelay` → `vTaskDelayUntil`), cần build lại (không chỉ Debug/Resume lại binary cũ) trước khi đo after.

## Bằng chứng

```
validation/01-timing/control-loop-jitter/
├── README.md
├── before/
│   ├── before_raw.bin
│   └── before_summary.csv
└── after/
    ├── after_raw.bin
    └── after_summary.csv
```
