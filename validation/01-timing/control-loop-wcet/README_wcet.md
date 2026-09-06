# Control-loop WCET — STM32H723 (mục 1.2)

## Tóm tắt

> Control loop WCET: 7.88 µs (0.08% of 10ms budget), N=2048 cycles, Home mode (mode mặc định lúc boot — Balance/Position không đo được, xem giới hạn bên dưới) — see `validation/01-timing/control-loop-wcet/`.

## Phương pháp đo

Tái dùng nguyên hạ tầng DWT của mục 1.1, thêm buffer riêng `s_exec_buf[2048]` và cặp mốc:
- **t0:** đầu thân xử lý mỗi chu kỳ trong `for(;;)`, ngay sau đoạn đo period của 1.1.
- **t1:** ngay sau lệnh `servo_actuator_step()` — đặt ở cả 3 vị trí gọi hàm này trong code thật (nhánh `state != STATE_RUN`, nhánh `!setpoint_get`, và nhánh chính sau `switch(sp.mode)`), để không bỏ sót chu kỳ nào bất kể trạng thái hệ thống.

`time_us = (t1 - t0) / 400` (CPU 400MHz). N = 2048 mẫu, xác nhận `s_exec_idx == 2048` (buffer đầy hoàn toàn) trước khi halt và dump — cùng quy trình kiểm tra chặt chẽ như mục 1.1, sau khi rút kinh nghiệm từ các lần dump dở buffer ở mục đó.

## Kết quả

| | Home mode (mode mặc định lúc boot) |
|---|---|
| N | 2048 |
| mean (µs) | 2.85 |
| **WCET = max (µs)** | **7.88** (0.08% của ngân sách 10ms) |
| min (µs) | 2.705 |
| std (µs) | 0.365 |

File dữ liệu gốc: `wcet_home_raw.bin`, `wcet_home_raw_summary.csv`.

## Trả lời 3 câu hỏi của mục 1.2

**1. WCET chiếm bao nhiêu % ngân sách 10ms?**
7.88 µs / 10000 µs ≈ **0.08%** — cực kỳ dư dả, không có nguy cơ vi phạm deadline ở mode đã đo.

**2. Mode nào tốn nhất (Balance vs Position vs Home)?**
**Không trả lời được đầy đủ** — chỉ đo được **Home mode**. Xem mục Giới hạn bên dưới.

**3. Có `dt` dài bất thường không (multi-segment trajectory)?**
Không quan sát thấy — `max = 7.88µs` chỉ chênh lệch nhỏ so với `mean = 2.85µs` (gấp ~2.8 lần), và `std = 0.365µs` rất nhỏ so với mean. Không có mẫu nào đột biến cao bất thường trong tập N=2048 ở Home mode. Không loại trừ khả năng multi-segment trajectory consumption chỉ xảy ra ở Position/Balance mode (không đo được ở đây).

## Giới hạn của phép đo — quan trọng, đọc trước khi dùng số liệu này

Chỉ đo được **Home mode** (mode mặc định khi hệ thống mới boot, `OPMODE_HOME = 0`). Balance mode và Position mode **chưa đo được** do thiết kế chương trình hiện tại không cho phép chuyển mode trong điều kiện thao tác thực tế lúc đo. Vì vậy:

- Con số WCET 7.88µs / 0.08% **chỉ đại diện cho Home mode**, không thể suy ra hoặc giả định áp dụng cho Balance/Position.
- Home mode nhiều khả năng là mode **ít tốn tính toán nhất** (không có vòng điều khiển PID phản hồi cảm biến phức tạp như Balance, không có nội suy trajectory nhiều đoạn như Position) — nên WCET thật của Balance/Position **có thể cao hơn đáng kể** con số này. Không được dùng số liệu Home mode để kết luận hệ thống an toàn ở mọi mode.
- Câu hỏi "mode nào tốn nhất" của mục 1.2 **còn để ngỏ**, cần bổ sung đo khi có cách chuyển mode được (ví dụ qua lệnh CAN, nút bấm vật lý, hoặc sửa tạm code để ép mode trong lúc đo).

## Bằng chứng

```
validation/01-timing/control-loop-wcet/
├── README.md
├── wcet_home_raw.bin
└── wcet_home_raw_summary.csv
```
