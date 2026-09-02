# Điều tra CAN bus STM32H723 <-> Jetson: từ "stm32_ok rơi về 0 sau 20-45s" đến 45 phút không tái hiện

Thư mục này tổng hợp toàn bộ quá trình điều tra, khoanh vùng và các fix đã áp dụng cho sự cố mất kết
nối CAN định kỳ giữa STM32H723 (node điều khiển) và Jetson (node vision/control). Mỗi thư mục con
là một lần đo cụ thể, giữ nguyên log gốc làm bằng chứng.

**Mức độ tin cậy hiện tại**: sau các fix ở mục 3, hai lần soak test liên tiếp (15 phút + 30 phút =
45 phút, mục 5 và 5.1) không tái hiện lại triệu chứng gốc (`stm32_ok` dao động liên tục 1↔0). Đây là
bằng chứng "không tái hiện được trong 45 phút đo", **không phải bằng chứng "đã hết lỗi vĩnh viễn"** —
45 phút chỉ giới hạn được tần suất lỗi còn lại xuống dưới cỡ ~1 lần/45 phút nếu có, chưa loại trừ được
lỗi hiếm hơn hoặc phụ thuộc điều kiện chưa tái hiện trong phòng thí nghiệm (nhiệt độ, rung động,
servo hoạt động nặng, v.v.). Một sự kiện STM32 tự reboot **chưa rõ nguyên nhân** đã xảy ra ở lần soak
đầu (mục 5) và không lặp lại ở lần soak sau (mục 5.1) — xem mục 6 (Giới hạn đã biết) trước khi coi vấn
đề này là đã đóng.

```
busoff-investigation/   giai đoạn đầu — phát hiện bug thứ tự socket + Jetson controller bị kẹt
signal-integrity/       đo baseline 500kbps sau khi bật hiển thị error frame
125kbps-measurement/    đo 60s ngay sau khi hạ bitrate — vẫn còn REC=127 và mất frame
125kbps-soak/           soak test 900s (15 phút) sau khi chia tần cả 2 phía — 1 lần reboot chưa rõ nguyên nhân, còn lại sạch
125kbps-soak-30min/     soak test 1800s (30 phút) lặp lại — 0 bất thường, không tái hiện reboot
```

## 1. Triệu chứng ban đầu

Hệ thống chạy được 20-45 giây rồi `stm32_ok` (đọc từ `stm32_state_is_ok()`, `src/system_state.cpp`)
rơi về 0 vĩnh viễn, không tự phục hồi, dữ liệu telemetry đóng băng ở giá trị cuối. Firmware STM32
lúc đó chưa có log chẩn đoán nào ngoài quan sát định tính.

## 2. Quá trình khoanh vùng (kể cả giả thuyết bị bác bỏ)

### 2.1. Bug thứ tự khởi tạo socket (xác nhận, đã sửa)

Log ứng dụng báo `setsockopt(CAN_RAW_ERR_FILTER) - Bad file descriptor` (do người dùng báo cáo khi
mở nhiệm vụ điều tra) — `CanTransport::open()` gọi `setsockopt(fd_, ...)` **trước** khi `fd_` được
gán bởi `socket()`. Sửa lại đúng thứ tự (`src/can_transport.cpp`). Đây là điều kiện tiên quyết để
error frame reporting hoạt động — nếu không sửa, mọi bằng chứng error-frame ở các bước sau đều
không thể thu thập được.

### 2.2. Thêm hiển thị error frame (công cụ chẩn đoán)

Sau khi sửa 2.1, error frame lần đầu vào được tới socket ứng dụng, nhưng
`frame.id = raw.can_id & CAN_SFF_MASK` cắt mất `CAN_ERR_FLAG` nên bị xử lý nhầm thành frame dữ liệu
rồi rơi vào `default: break;`. Thêm `is_error`/`err_class` vào `can_frame_t` (`include/can_protocol.h`),
decode trong `CanTransport::receive()`, và log `[CAN_ERR] class=... tec=... rec=...` trong
`src/task_can_rx.cpp` (không dùng `continue` để không phá watchdog alive-mark). Đây là công cụ chẩn
đoán then chốt dẫn tới bằng chứng ở mục 2.4.

### 2.3. Jetson mttcan controller bị "kẹt" ở tầng driver (phát hiện phụ, tự khắc phục bằng reset)

Trong lần đo đầu (`busoff-investigation/`), toàn bộ counter `ip -s link show can0` (RX/TX packets,
error-pass, bus-off) đứng yên tuyệt đối suốt 62 giây dù app đang chạy, `candump` không ghi được
frame nào, `dmesg` không có sự kiện CAN mới nào trong đúng khung giờ đo. Lặp lại độc lập ở
`signal-integrity/` (baseline 500kbps) cho kết quả giống hệt. Sau khi `sudo ip link set can0 down/up`,
controller hoạt động trở lại bình thường (xác nhận ở `125kbps-measurement/`). **Kết luận: đây là một
lỗi driver/controller riêng, không liên quan tới nguyên nhân gốc, nhưng đã làm nhiễu 2 lần đo đầu —
mọi số liệu "0 frame" trong `busoff-investigation/candump.log` và `signal-integrity/candump_500k.log`
phải được đọc với lưu ý này, không phải bằng chứng "STM32 ngừng phát".**

### 2.4. Bằng chứng khoanh vùng chiều lỗi: `[CAN_ERR] class=0x00000004 tec=0 rec=127`

Sau khi reset interface, `app_125k.log` liên tục ghi được:
```
[CAN_ERR] class=0x00000004 tec=0 rec=127 data=00 10 00 00 00 00 00 7F
```
`class=0x04` = `CAN_ERR_CTRL` với byte cờ tương ứng `CAN_ERR_CRTL_RX_PASSIVE`; `data[6]=TEC=0`,
`data[7]=REC=127`. **Jetson gửi sạch (TEC=0) nhưng lỗi khi nhận (REC=127)** — khoanh vùng lỗi vào
chiều tín hiệu STM32 -> Jetson, không phải chiều ngược lại.

### 2.5. Giả thuyết "bus kẹt dominant" — BỊ BÁC BỎ

Đo trực tiếp chênh lệch điện áp CANH/CANL bằng đồng hồ: **không lệch 2V** ở trạng thái nghỉ, loại
trừ khả năng một node giữ bus ở mức dominant liên tục. (Phép đo phần cứng do người dùng thực hiện,
không có file log trong repo này.)

### 2.6. Giả thuyết "lỗi signal integrity thuần vật lý do R3=10k gây corrupt bus" — BỊ BÁC BỎ Ở DẠNG ĐƠN GIẢN

Test `cangen` tạo tải cao ở 125kbps trong khi app tắt (chỉ có traffic tổng hợp, không có traffic
thật/logic firmware) cho kết quả **bus sạch hoàn toàn** — nghĩa là bản thân tầng vật lý (transceiver
SN65HVD230 + R3=10k slope-control) ở 125kbps không tự sinh lỗi khi tải thuần túy tăng lên. (Test do
người dùng thực hiện, không có file log trong repo này — không suy diễn số liệu.) Điều này cho thấy
sự bất ổn quan sát được ở 500kbps chủ yếu đến từ **tương tác giữa traffic thật/lỗi firmware và giới
hạn slew-rate của slope-control**, không phải bus vật lý "hỏng" một cách tuyệt đối — nhưng R3=10k vẫn
là yếu tố giới hạn tốc độ tối đa an toàn, nên quyết định hạ bitrate (mục 4) vẫn được giữ như một biên
an toàn, không đảo ngược.

## 3. Bảng 5 fix — số liệu trước/sau (chỉ trích từ log thật)

| # | Fix | File | Trước | Sau |
|---|-----|------|-------|-----|
| 1 | Thứ tự `socket()` trước `setsockopt(CAN_RAW_ERR_FILTER)` | `src/can_transport.cpp` | `setsockopt(...) - Bad file descriptor` (báo cáo ban đầu) | Error frame reporting hoạt động, tiền đề cho fix #2 |
| 2 | Decode + log error frame (`is_error`/`err_class`, `[CAN_ERR]`) | `include/can_protocol.h`, `src/can_transport.cpp`, `src/task_can_rx.cpp` | Error frame bị hiểu nhầm thành data frame, rơi `default: break;` | `[CAN_ERR] class=0x00000004 tec=0 rec=127` — bằng chứng khoanh vùng chiều lỗi (mục 2.4) |
| 3 | `AutoRetransmission` STM32 `ENABLE` -> `DISABLE` | firmware STM32 | `attempt=18505 drop=18495` (~99.9% drop), `free=0` liên tục (Nhiệm vụ A, trước khi app chạy) | `drop=58` đứng yên, `free=2` không về 0 (Step 0 / baseline sau fix) |
| 4 | Rate-limit bus-off recovery STM32 (1000ms) | firmware STM32 | `busoff` tăng ~100 lần/giây (busoff-investigation, 43458->49458 trong 59s) | `busoff` tăng ~0.6-1 lần/giây (125kbps-measurement) -> **0 lần/giây ổn định** ở soak test (busoff đứng yên 3->4->5, chỉ tăng ở 2 sự kiện đơn lẻ trong 900s) |
| 5 | Hạ bitrate 500k->125k (sample-point 0.875 khớp 2 phía) + chia tần telemetry cả 2 phía | `src/task_can_tx.cpp` (Jetson), firmware STM32 | REC pinned ở 127 (500kbps, xuyên suốt); ở lần đo 125kbps đầu (chưa soak) 0x102/0x103/0x104 **không xuất hiện một lần nào** trong 59s, `S1=S2=S3=0` | Soak 900s: `berr-counter tx 0 rx 0` ở 176/179 mẫu (97.8%), `S1=1632 S2=1549 S3=1580` (giá trị thật), phân bố ID khớp thiết kế 100/100/20/20/20/20 Hz |

## 4. Quyết định thiết kế: 125kbps + slope control giữ nguyên

Schematic module CAN (SN65HVD230 Breakout V2.0) có chân RS nối GND qua R3=10k (chế độ
slope-control, giảm dV/dt để chống nhiễu EMI từ 3 servo chạy gần dây CAN). Thay vì nối tắt R3
(rủi ro tăng nhiễu bức xạ ảnh hưởng IMU/servo), quyết định **giữ nguyên phần cứng, hạ bitrate xuống
125kbps** (đủ thời gian cho slew-rate giới hạn ổn định trong mỗi bit) và **chia tần các frame
telemetry không tối quan trọng** để giữ bus load trong ngân sách mới:

- `CAN_ID_BALL_POS` (0x200), `CAN_ID_ATTITUDE_DESIRED` (0x204): giữ 100Hz (vòng điều khiển thật)
- `CAN_ID_BALL_VEL` (0x201): 50Hz
- `CAN_ID_BALL_STATE` (0x202), `CAN_ID_HEARTBEAT_RX` (0x2FF): 20Hz (50ms, vẫn dưới ngưỡng timeout 500ms
  của `stm32_state_is_ok()`, `src/system_state.cpp:105`)
- Firmware STM32 áp dụng chia tần tương tự cho 0x101/0x103/0x104/0x1FF (20Hz), giữ 0x100/0x102 ở 100Hz

Sample point 2 phía khớp **chính xác 0.875**: Jetson `tq=40ns prop-seg=87 phase-seg1=87 phase-seg2=25`
(tổng 200 tq, `(1+87+87)/200=0.875`), STM32 `TimeSeg1=13 TimeSeg2=2` (tổng 16 tq, `14/16=0.875`).
(Ở 500kbps, do ràng buộc chia hết của xung nhịp 50MHz trên mttcan, sample point chỉ đạt được 0.870 —
không thể khớp chính xác 0.875 ở tốc độ đó.)

## 5. Soak test đầu tiên — 900 giây (15 phút), `125kbps-soak/`

Điều kiện: 2026-09-02 20:58:27 -> 21:13:27 (+07), 125kbps, sample-point 0.875 hai phía, firmware
STM32 có AutoRetransmission=DISABLE + rate-limit bus-off recovery 1000ms + chia tần telemetry, Jetson
`task_can_tx.cpp` đã chia tần theo mục 4. Tại thời điểm đo này repo chưa có commit nào (đã commit sau
đó — xem lịch sử git, bắt đầu từ "Initial commit: STM32H723 firmware + Jetson vision control").

| Chỉ số | Kết quả | Ghi chú |
|---|---|---|
| Số lần `TaskCanRx: FAULT` trong 900s | **2 lần** | Cả 2 đều trùng khớp 1 sự kiện **STM32 tự reboot thật** ở t+374s (log UART có đầy đủ chuỗi boot: `FDCAN kernel clock`, `StartTaskCanTx STARTED (125 kbit/s bus budget)`, `MPU6500 init: OK`; biến đếm `attempt` reset 1908347->1244). Không phải lỗi bus lặp lại. |
| `berr-counter tx/rx` khác 0 | 3/179 mẫu (1.7%) | Cả 3 đều nằm trong cửa sổ ±20s quanh sự kiện reboot hoặc các busoff đơn lẻ tương ứng (t+136s, t+359s — khớp đúng dòng `busoff=4`, `busoff=5` trong `stm32_uart_soak.log`) |
| Trạng thái rời `ERROR-ACTIVE` | 2/181 mẫu (1.1%), `ERROR-PASSIVE` | Cùng cửa sổ reboot |
| `delta_drop` [CAN_TX] (loại trừ cửa sổ reboot) | `44 -> 50` trong ~520s còn lại (+6, gắn với 2 busoff đơn lẻ) | Không phải 0 tuyệt đối nhưng cực nhỏ, không phải hiện tượng lặp lại hệ thống |
| `busoff` [CAN_TX] (loại trừ reboot) | `3 -> 7` trong ~520s (2 sự kiện đơn lẻ) | So với ~100/giây trước fix #4 — cải thiện >99.9% |
| TX dropped (netdevice, `ip -s link`) | `16 -> 16`, **delta=0 tuyệt đối** | |
| Bus load thực | candump (chỉ chiều STM32) = 279.0 fr/s; cộng TX packets Jetson (+302 fr/s, không hiện trong candump do `CAN_RAW_LOOPBACK` đã tắt ở `can_transport.cpp`) = **~581 fr/s × 1080µs ≈ 62.7%** | Khớp gần đúng thiết kế lý thuyết ~62%, dưới ngưỡng 70% |
| Phân bố ID (900s) | `0x100=90544(~100.6Hz) 0x102=88356(~98.2Hz) 0x1FF=18115 0x103=18114 0x104=18068 0x101=17957(~20.0Hz mỗi ID)` | Khớp gần như hoàn hảo thiết kế 100/100/20/20/20/20 Hz |
| Frame Jetson (0x200 v.v.) trong candump | **Không thấy** | Do `CAN_RAW_LOOPBACK` bị tắt trong `CanTransport::open()` (fix trước đó, dùng chung code cho cả socket RX và TX) — không phải bằng chứng Jetson ngừng phát. Xác nhận gián tiếp qua netdevice TX packets (+272000/900s ≈302 fr/s, khớp thiết kế ~290Hz) |
| S1/S2/S3 | `S1=1632 S2=1549 S3=1580` xuyên suốt phần lớn thời gian | Giá trị thật, không còn 0 |

**Pháp y quanh thời điểm reboot** (t+375s, tức 1788357883, tính từ `START_soak=1788357507.72`):
gap duy nhất >1s trong toàn bộ 900s của `candump_soak.log` là 2.10s, xảy ra ở candump-relative
t=394.19s->396.29s (khớp đúng lần FAULT thứ 2, không phải lúc reboot) — nghĩa là bản thân quá trình
reboot của STM32 im lặng trên bus CAN **dưới 1 giây**, không đủ để lọt qua ngưỡng phát hiện gap 1s.
Ngay tại t+375s, `app_soak.log` ghi thêm `[CAN_ERR] class=0x00000004 tec=128 rec=0` (chiều ngược với
bằng chứng REC=127 ở mục 2.4 — lần này TEC phía Jetson tăng, hợp lý vì Jetson vẫn phát trong lúc
STM32 tạm ngừng ACK để reboot) và dòng `TaskWatchdog: CANH BAO ... missing_mask=0x01 (Camera)` —
nhưng cảnh báo camera này **xuất hiện liên tục 899 lần trong suốt cả 900 giây, bắt đầu từ giây thứ 2**
(không phải riêng quanh lúc reboot), nên đây là một vấn đề nền tồn tại sẵn, không tương quan với sự
kiện STM32 reboot — không nên coi là manh mối liên quan.

**Phát hiện phụ đáng chú ý**: UART print `[CAN_TX]` của STM32 "đứng hình" bit-for-bit suốt 374 giây
đầu soak test (in lặp lại y hệt giá trị `attempt=1803347 drop=26908 busoff=4069` 74 lần liên tiếp,
mỗi 5s), trong khi bằng chứng độc lập phía Jetson (candump: 105064 frame thật với nội dung data đổi
liên tục, netdevice RX/TX tăng đều ~280-320 fr/s) xác nhận **bus CAN hoạt động bình thường cùng lúc**.
**Nguyên nhân UART đứng hình chưa được xác định.** Retarget UART của firmware dùng
`HAL_UART_Transmit` blocking với `HAL_MAX_DELAY`, không dùng DMA — giả thuyết "lỗi DMA" trong bản
nháp trước của tài liệu này không khớp với code và đã bị loại bỏ. Cần điều tra thêm để phân biệt giữa
(a) code tính `attempt`/`drop`/`busoff` trong firmware ngừng cập nhật (dù task CAN TX/RX vẫn chạy —
việc bus vẫn có frame thật chứng minh ít nhất phần gửi/nhận CAN không bị treo), và (b) phía Jetson
mất dữ liệu khi đọc `/dev/ttyACM0` (đường đọc UART bị lỗi/bỏ sót byte trong đúng cửa sổ đó dù các
đoạn khác của cùng file log vẫn đọc đúng).

Phép thử "giá trị `attempt` có nhảy vọt sau 374 giây hay không" không áp dụng được trực tiếp ở đây,
vì **một sự kiện MCU reboot thật xảy ra ngay khi UART hết đứng hình** (t+375s: chuỗi boot đầy đủ
`FDCAN kernel clock = 80000000 Hz` ... `StartTaskCanTx STARTED (125 kbit/s bus budget)` ...
`MPU6500 init: OK`, biến `attempt` reset `1908347 -> 1244`) — bất kỳ giá trị nào trước 374 giây đều
bị xóa bởi reset này nên không thể dùng để suy ra "có nhảy vọt hay không". Bằng chứng gián tiếp mạnh
nhất vẫn là candump/netdevice: bus CAN không hề im lặng trong suốt 374 giây UART đứng hình, nên nếu
đúng là code TX/RX bị treo thì đó là treo *một phần* (chỉ phần counter/print), không phải treo toàn
bộ firmware.

## 5.1. Soak test thứ hai — 1800 giây (30 phút), `125kbps-soak-30min/`

`candump.log` gốc (24MB) được nén `gzip -9` thành `candump.log.gz` (3.08MB) trước khi commit — giải
nén bằng `gunzip -k candump.log.gz` nếu cần xem từng frame; các số liệu tổng hợp (tổng 508074 frame,
1800.1s, phân bố theo ID) đã trích sẵn trong bảng dưới đây nên không bắt buộc phải giải nén.

Chạy bằng `scripts/run_soak.sh 1800 validation/03-can/125kbps-soak-30min`, phân tích bằng
`scripts/analyze_soak.py` (script tái sử dụng, viết sau lần soak 900s đầu — xem
`scripts/run_soak.sh`/`scripts/analyze_soak.py`). `can0` được reset sạch qua
`systemctl restart can0.service` ngay trước khi đo (`ERROR-ACTIVE, berr tx=0 rx=0` xác nhận trong
`run_info.txt`). Firmware STM32 tại thời điểm này đã có thêm log `[BOOT]` (đọc `RCC->RSR` lúc khởi
động, đề xuất ở mục pháp y sự kiện reboot của lần soak trước) — mục đích chính của lần chạy này là chờ
xem sự kiện reboot ở t+374s của lần soak 900s trước có lặp lại không, và nếu có thì bắt được nguyên
nhân qua `[BOOT]`.

**Kết quả: 22:03:47 → 22:33:47 (+07), 30 phút liên tục, không có bất kỳ bất thường nào:**

| Chỉ số | Kết quả |
|---|---|
| `TaskCanRx: FAULT` | **0 lần** (`stm32_ok=1` ở toàn bộ 8980 dòng `[main]`, kể cả dòng đầu tiên ngay sau khi start) |
| `[BOOT]` (STM32 reset) | **0 lần** — không có sự kiện reboot lặp lại như lần soak trước |
| `delta_drop` / `delta_busoff` [CAN_TX] | **0/363 mẫu khác 0** — đứng yên tuyệt đối suốt 30 phút |
| Mẫu rời `ERROR-ACTIVE` (canstats) | **0/360** |
| `berr-counter` | `tx` luôn = 0; `rx` chạm 2 và 8 đúng 2 lần (nhiễu bit đơn lẻ, tự hết trong 1 chu kỳ 5s, không leo thang tới ngưỡng warning 96) |
| Gap candump >100ms | **0** |
| Bus load thực (chỉ chiều STM32, candump) | 508074 frame / 1800.1s = 282.3 fr/s ≈ **30.48%** |
| Phân bố ID | `0x100=181714(100.9Hz) 0x102=181110(100.6Hz) 0x103=36345 0x1FF=36344 0x104=36330 0x101=36231(~20.1-20.2Hz mỗi ID)` — khớp thiết kế |
| S1/S2/S3 | `S1=1632 S2=1549 S3=1580` không đổi suốt 30 phút |

Đây là bằng chứng mạnh cho thấy sự kiện reboot ở lần soak 900s trước **là hiếm/ngẫu nhiên, không phải
lỗi lặp lại theo chu kỳ** — 30 phút liên tục sau đó không tái hiện. Do không có `[BOOT]` nào xảy ra,
công cụ chẩn đoán `RCC->RSR` mới thêm **chưa có dịp được kiểm chứng thực tế** — vẫn cần chạy tiếp tới
khi bắt được một lần reboot thật để xác nhận log `[BOOT]` hoạt động đúng và đọc được nguyên nhân.

## 5.2. Kiểm chứng có chủ đích: reset thủ công + code `[BOOT]`

**Vòng 1** (trước khi sửa vị trí code): reset thủ công bằng nút bấm cho thấy reset xảy ra thật
(`attempt` giảm `1599517 -> 1246`, đầy đủ chuỗi boot quen thuộc) nhưng KHÔNG có dòng `[BOOT]` nào —
kết luận lúc đó: code đọc `RCC->RSR` đặt trước `MX_USART3_UART_Init()` nên `printf` chưa có gì để ghi
ra. Đã đề xuất fix: lưu `RCC->RSR` vào biến global ngay sau `HAL_Init()`, gọi
`__HAL_RCC_CLEAR_RESET_FLAGS()`, in biến đó sau khi UART đã init.

**Vòng 2** (sau khi nạp fix trên) — kiểm chứng có chủ đích bằng **2 phép thử độc lập** để phân biệt
"đọc đúng thanh ghi" với "in ra một giá trị cố định":

- **(a) Bấm nút reset (kỳ vọng `PIN=1`)**: `[BOOT]` **đã in ra** (vị trí code nay đã đúng — xác nhận
  reset thật qua `attempt` giảm `39046 -> 1246` cùng đầy đủ chuỗi boot), nhưng nội dung là:
  ```
  [BOOT] RSR=0x00000000 IWDG=0 WWDG=0 SFT=0 BOR=0 POR=0 PIN=0 LPWR=0
  ```
  **Tất cả các cờ đều = 0, kể cả `PIN`** — không khớp kỳ vọng `PIN=1`.
- **(b) Rút nguồn cắm lại (kỳ vọng `POR=1`)**: thử 3 lần, không bắt được giá trị `RSR` nào — giới hạn
  kỹ thuật của việc bắt UART qua USB-VCP (STLink-V3 mất ~1s để enumerate lại sau khi mất điện,
  trong khi firmware in `[BOOT]` chỉ vài chục ms sau khi UART sẵn sàng; `dmesg` xác nhận
  `New USB device found` đúng lúc rút/cắm). Ngay cả với vòng lặp tự mở lại thiết bị ngay khi tái xuất
  hiện, dòng `[BOOT]` và vài dòng đầu vẫn bị mất/hỏng ký tự (`stty raw` chưa kịp áp dụng lên thiết bị
  mới trước khi các byte đầu tiên tới). Đây là giới hạn phần cứng/thời gian của phương pháp bắt log,
  **không phải bằng chứng về giá trị `POR`** — quyết định không tiếp tục thử để tiết kiệm thời gian,
  vì phép thử (a) đã đủ mạnh để kết luận (xem bên dưới).

**Kết luận**: kiểm chứng **KHÔNG đạt** ở vòng 2. Cơ chế in `[BOOT]` (vị trí code) đã đúng, nhưng giá
trị `RSR` đọc được luôn là `0x00000000` bất kể loại reset — đúng dạng "in ra giá trị cố định" mà phép
kiểm chứng 2-phép-thử này được thiết kế để phát hiện. Nguyên nhân nhiều khả năng: thứ tự
`g_boot_rsr = RCC->RSR;` và `__HAL_RCC_CLEAR_RESET_FLAGS();` bị đảo (xóa trước khi lưu), hoặc có một
lệnh `__HAL_RCC_CLEAR_RESET_FLAGS()` khác chạy trước đoạn code mới ở đâu đó trong chuỗi khởi tạo
(vd. trong code CubeMX sinh sẵn). Cần xem lại đúng đoạn code đã nạp để xác định nguyên nhân cụ thể —
chưa có đủ thông tin để khẳng định chắc chắn hơn.

**Trạng thái công cụ `[BOOT]` tính đến thời điểm này: vị trí in đã đúng, giá trị đọc vẫn sai — chưa
dùng được để chẩn đoán nguyên nhân reboot thật.**

## 6. Giới hạn đã biết

- Nguyên nhân STM32 tự reboot ở t+374s trong soak test 900s đầu tiên **vẫn chưa được xác định** —
  không tái hiện trong 30 phút chạy tiếp theo (mục 5.1), nên khả năng cao là sự kiện hiếm/ngẫu nhiên,
  nhưng chưa đủ dữ liệu để loại trừ hẳn một nguyên nhân định kỳ hiếm gặp (vd. brown-out khi servo giật
  dòng đỉnh). **Log `[BOOT]`/`RCC->RSR` in ra đúng vị trí nhưng đọc SAI giá trị** — kiểm chứng chủ
  động vòng 2 bằng reset nút bấm (mục 5.2) cho kết quả `RSR=0x00000000` (mọi cờ đều 0) dù `PIN` reset
  chắc chắn đã xảy ra. Cần xem lại thứ tự lưu/xóa cờ trong firmware trước khi công cụ này dùng được
  cho lần reboot ngẫu nhiên tiếp theo; phép thử (b) rút nguồn chưa kiểm chứng được do giới hạn kỹ
  thuật bắt UART qua USB-VCP (xem mục 5.2).
- UART print bị đứng hình 374 giây (chỉ ở lần soak đầu) là một lỗi riêng biệt (mục 5, phát hiện phụ),
  nguyên nhân gốc **vẫn chưa xác định** — không tái diễn ở lần soak 30 phút nên chưa có thêm dữ liệu để
  phân biệt giữa 2 giả thuyết (a)/(b) đã nêu.
- Giả thuyết "bus kẹt dominant" và "cangen test bác bỏ signal-integrity thuần túy" (mục 2.5, 2.6) dựa
  trên phép đo phần cứng/test do người dùng thực hiện ngoài phiên làm việc này — không có file log
  trong repo để trích dẫn số liệu cụ thể.
- Tổng cộng đã có 2 lần soak (900s + 1800s = 45 phút bằng chứng liên tục, không tính khoảng nghỉ giữa
  2 lần), chưa phải bằng chứng vận hành liên tục nhiều giờ/ngày hoặc qua nhiều chu kỳ bật/tắt nguồn.

## 7. Khuyến nghị bước tiếp theo

1. ~~Chạy soak test dài hơn~~ **Đã làm** (mục 5.1, 30 phút, 0 bất thường). Nếu cần độ tin cậy cao hơn
   nữa cho vận hành thực tế, cân nhắc chạy qua đêm (≥8 giờ) hoặc lặp lại soak 30 phút qua vài lần
   bật/tắt nguồn khác nhau — sự kiện reboot ở lần đầu vẫn chưa giải thích được nên chưa nên coi là
   "đã đóng".
2. Điều tra riêng lỗi UART print bị đứng hình ở lần soak đầu (mục 5) — nguyên nhân chưa xác định, cần
   xem lại code counter/print trong firmware (không phải DMA — retarget dùng `HAL_UART_Transmit`
   blocking). Không liên quan tới CAN nhưng ảnh hưởng khả năng chẩn đoán tương lai.
3. Cân nhắc thêm test `cangen` bên trong repo (có log) để việc bác bỏ giả thuyết signal-integrity ở
   mục 2.6 có bằng chứng tái lập được, thay vì chỉ dựa vào báo cáo ngoài phiên.
4. ~~Commit working tree vào git~~ **Đã làm** — xem lịch sử commit trong repo (bắt đầu từ
   "Initial commit: STM32H723 firmware + Jetson vision control").
