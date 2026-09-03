# Soak test 30 phút xác nhận lại (VIỆC 2), 2026-09-03 23:19:21 → 23:49:21 (+07)

Chạy theo yêu cầu người dùng (VIỆC 2) sau các phát hiện ở
[`../regression-after-reboot/README.md`](../regression-after-reboot/README.md) mục 2.4, để xem hệ
thống đã quay lại được trạng thái sạch như soak 30 phút hôm qua
([`../125kbps-soak-30min/`](../125kbps-soak-30min/README.md), thực ra là mục 5.1 của
[`../README.md`](../README.md)) hay chưa.

Lệnh:
```
./scripts/run_soak.sh 1800 validation/03-can/soak-after-reconnect
python3 scripts/analyze_soak.py validation/03-can/soak-after-reconnect
```
`run_soak.sh` đã được cập nhật (xem `../regression-after-reboot/README.md` mục 1/5) để reset cứng
driver (`rmmod`/`modprobe mttcan`) trước khi đo — `run_info.txt` ghi `berr-counter tx=0 rx=9` lúc bắt
đầu (rx đã kịp tăng lên 9 trong ~1s giữa lúc lên interface và lúc ghi log, vì bus đang lỗi tích cực
ngay từ giây đầu — xem bên dưới), không phải `tx=0 rx=0` tuyệt đối, nhưng đây vẫn là số liệu thật từ
phần cứng, không phải carry-over từ phiên trước.

## Kết quả: TÁI HIỆN ĐẦY ĐỦ TRIỆU CHỨNG GỐC, tệ hơn nhiều so với hôm qua

| Chỉ số | Hôm qua (`125kbps-soak-30min`, README mục 5.1) | Hôm nay (`soak-after-reconnect`) |
|---|---|---|
| `TaskCanRx: FAULT` | **0 lần** | **1071 lần** (1071 `CLEARED` tương ứng) |
| `stm32_ok` | luôn = 1 | dao động liên tục 1↔0 — đúng y hệt triệu chứng gốc trong báo cáo ban đầu ("`can state ERROR-WARNING`... `stm32_ok` dao động liên tục") |
| Thời gian ổn định giữa 2 lần FAULT | N/A (không có FAULT) | avg **1.54s**, max 21.00s — tức trung bình chưa đầy 2 giây lại mất kết nối 1 lần |
| Thời gian mất kết nối mỗi lần | N/A | avg 0.15s, max 1.00s |
| `delta_drop` [CAN_TX] (STM32) | **0/363 mẫu khác 0** | **350/361 mẫu khác 0**, tổng 13905, trung bình 38.5/mẫu (5s) |
| `delta_busoff` [CAN_TX] (STM32) | **0/363** | **354/361 mẫu khác 0**, tổng **1563 lần bus-off** trong 1800s (~0.87 lần/giây) |
| Mẫu rời `ERROR-ACTIVE` (canstats) | **0/360** | **204/360 (56.7%)**, đa số ở `ERROR-PASSIVE` |
| `berr-counter tx` cuối kỳ | luôn 0 | **128** (đúng ngưỡng `ERROR-PASSIVE`) — khác hẳn lúc app tắt (tx luôn=0 vì Jetson không tự phát) vì giờ Jetson đang phát thật và bị NACK |
| candump.log (chỉ chiều STM32, KHÔNG phải bus load thật — xem đính chính dưới bảng) | 282.3 fr/s | 113.9 fr/s |
| Jetson TX (netdevice `tx_packets` delta, canstats.log) | 289.8 fr/s | **173.0 fr/s** — giảm ~40%, xác nhận chiều Jetson->STM32 cũng bị ảnh hưởng, không chỉ STM32->Jetson |
| **Bus load thực (2 chiều: candump + Jetson TX)** | **572.1 fr/s ≈ 61.79%** (khớp thiết kế ~62%) | **286.9 fr/s ≈ 30.98%** — chưa bằng nửa |
| Phân bố ID vs thiết kế | khớp gần hoàn hảo 100/100/20×4 Hz | **0x100=52.3Hz(52%) 0x102=23.2Hz(23%) 0x1FF=10.9Hz(55%) 0x101=10.0Hz(50%) 0x103=10.0Hz(50%) 0x104=7.5Hz(38%)** — mọi ID đều mất ít nhất 45%, `0x102` mất nặng nhất (77%) |
| Gap candump >100ms | **0** | **1354** |
| `[BOOT]` (STM32 reset) | 0 | 0 — **không có MCU reboot nào**, nghĩa là toàn bộ 1071 lần FAULT là do mất frame/timeout thật trên bus, không phải do STM32 tự khởi động lại |

**Đính chính bus load (2026-09-04)**: bản đầu của bảng này chỉ ghi số liệu `candump.log`
(113.9 fr/s ≈ 12.30% hôm nay, 282.3 fr/s ≈ 30.48% hôm qua) và gọi nhầm đó là "bus load thực" — sai,
vì `candump` không bao giờ thấy frame do chính Jetson phát (`CanTransport::open()` tắt
`CAN_RAW_LOOPBACK` trên socket gửi, nên kernel không loopback frame đó về bất kỳ socket local nào).
`scripts/analyze_soak.py` đã được sửa để cộng thêm `tx_packets` (netdevice, đọc từ `canstats.log`,
không phụ thuộc `CAN_RAW_LOOPBACK`) — bảng trên đã cập nhật số đúng. Các số liệu khác (FAULT,
busoff, berr-counter, gap, phân bố ID) không bị ảnh hưởng bởi lỗi này.

## Kết luận

**Xác nhận dứt điểm giả thuyết ở `../regression-after-reboot/README.md`**: hệ thống **chưa** quay lại
trạng thái sạch như hôm qua. Ngay khi có tải thật (app chạy, cả 2 chiều cùng phát), lỗi vật lý gián
đoạn phát hiện được ở mục 2.3/2.4 của tài liệu đó bộc phát thành đúng triệu chứng gốc mà người dùng
báo cáo ban đầu — `stm32_ok` dao động 1↔0 liên tục — chỉ trong vòng **15 giây đầu tiên** sau khi app
khởi động (`busoff=1009` chỉ sau vài giây, xem log gốc `app.log`/`stm32_uart.log`), và duy trì suốt
1071 lần trong 30 phút, không tự hết.

**Không có `[BOOT]` nào** loại trừ được khả năng đây là do STM32 tự reset liên tục (khác với sự kiện
reboot đơn lẻ chưa rõ nguyên nhân ở soak 900s đầu tiên, mục 5 của `../README.md`) — đây là mất
frame/timeout thuần tuý trên bus, khớp với bằng chứng vật lý (đầu nối/tiếp xúc chập chờn) đã nêu.

**5 fix trước đó ở `../README.md` mục 3 (bit timing, AutoRetransmission, rate-limit bus-off, bitrate
125k, chia tần) vẫn hoạt động đúng như thiết kế** — không có bằng chứng nào cho thấy chúng bị vô hiệu
hoá hay hỏng; `busoff` vẫn được rate-limit (không tăng ~100 lần/giây như trước fix #4, chỉ ~0.87
lần/giây — vẫn là cải thiện lớn so với baseline gốc, chỉ là không đủ để che giấu một lỗi vật lý đang
hoạt động tích cực). Vấn đề nằm hoàn toàn ở tầng vật lý bên ngoài phạm vi các fix phần mềm này.

## Bước tiếp theo cần phần cứng, không phải phần mềm

Bằng chứng qua 3 phép đo (dmesg trước khi module nạp, dao động berr-counter khi app tắt, và soak 30
phút này khi app chạy) đều nhất quán chỉ về **một điểm tiếp xúc vật lý chập chờn** (đầu nối CAN,
connector JST/terminal block, hoặc mối hàn) — không phải cấu hình, không phải firmware, không phải
driver Jetson. Việc cắm lại bằng tay của người dùng trước phiên này chỉ tạm thời cải thiện (khoảng
nghỉ ~14 phút thấy ở dmesg), không phải fix dứt điểm. Khuyến nghị kiểm tra vật lý kỹ hơn: rung nhẹ
từng đầu nối CAN_H/CAN_L/GND dọc theo dây trong lúc app đang chạy và console `candump`/berr-counter
đang mở, để khoanh vùng chính xác điểm lỏng (đầu Jetson, đầu STM32, hay giữa dây).

## File

| File | Nội dung |
|---|---|
| `run_info.txt` | Trạng thái can0 lúc bắt đầu/kết thúc |
| `candump.log` | Toàn bộ frame CAN 1800s (9.3MB, không nén — dưới ngưỡng 10MB trong `.gitignore`) |
| `canstats.log` | `ip -details -s link show can0` mỗi 5s |
| `app.log` | Log ứng dụng Jetson (bao gồm 1071 dòng `TaskCanRx: FAULT`) |
| `stm32_uart.log` | UART STM32 (`[CAN_TX] attempt/drop/busoff/TEC/REC...` mỗi ~5s) |
| `analysis.txt` | Output đầy đủ của `scripts/analyze_soak.py` (danh sách timestamp FAULT đã lược bớt để đỡ dài — xem `app.log` nếu cần đầy đủ) |
