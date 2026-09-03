# Điều tra CAN bus STM32H723 <-> Jetson: 3 bug phần mềm đã sửa, nguyên nhân gốc là phần cứng — CHƯA GIẢI QUYẾT

Thư mục này tổng hợp toàn bộ quá trình điều tra, khoanh vùng và các fix đã áp dụng cho sự cố mất kết
nối CAN định kỳ giữa STM32H723 (node điều khiển) và Jetson (node vision/control). Mỗi thư mục con
là một lần đo cụ thể, giữ nguyên log gốc làm bằng chứng.

## TRẠNG THÁI HIỆN TẠI (2026-09-04): CHƯA GIẢI QUYẾT — đang chờ khắc phục phần cứng

**Đọc mục này trước khi đọc bất kỳ phần nào khác của tài liệu.** Bản trước của phần mở đầu này (viết
2026-09-02, trước khi có `regression-after-reboot/` và `soak-after-reconnect/`) mô tả câu chuyện theo
hướng "5 fix phần mềm -> 45 phút soak sạch -> vấn đề gần như đã đóng". Cách kể đó **gây hiểu lầm** và
đã bị chứng minh sai bởi chính dữ liệu thu được ngày 2026-09-03/04. Câu chuyện thật là:

1. **Nguyên nhân gốc là một điểm tiếp xúc vật lý chập chờn** trên đường dây CAN (đầu nối/mối hàn —
   xem bằng chứng dứt điểm ở [`regression-after-reboot/README.md`](regression-after-reboot/README.md)
   mục 2.3/2.4: lỗi xảy ra cả khi module kernel `can`/`can_raw` còn chưa nạp — không một dòng code hay
   socket userspace nào có thể gây ra nó — và tái diễn thành từng cụm xen kẽ khoảng nghỉ, mẫu hình
   kinh điển của tiếp xúc điện chập chờn, không phải lỗi cấu hình).
2. Lỗi tiếp xúc này tạo ra triệu chứng (`stm32_ok` dao động, REC/TEC tăng, mất frame) **trông giống**
   nhiều giả thuyết phần mềm khác nhau, nên quá trình điều tra ban đầu (mục 1-4 bên dưới) đã lần lượt
   đuổi theo và **tìm ra 3 bug phần mềm CÓ THẬT**, mỗi bug đều được chứng minh bằng số liệu trước/sau
   cụ thể (bảng ở mục 3): (a) thứ tự `socket()`/`setsockopt(CAN_RAW_ERR_FILTER)` sai, (b)
   `AutoRetransmission` STM32 để `ENABLE` gây bùng nổ retry, (c) vòng lặp bus-off recovery của STM32
   không rate-limit. Cả 3 fix này **vẫn đúng, vẫn có giá trị thật** — chúng sửa lỗi thật, không phải
   giả tưởng, và làm hệ thống chịu lỗi tốt hơn hẳn (busoff giảm từ ~100 lần/giây xuống <1 lần/giây khi
   có tải thật — xem `soak-after-reconnect/README.md`).
3. **Nhưng cả 3 fix đó không phải là lời giải cho vấn đề gốc.** Soak test 15+30 phút sạch (mục 5, 5.1
   dưới đây) từng được coi là bằng chứng "đã hết lỗi" — **sai**: đó chỉ là 45 phút tình cờ rơi vào một
   khoảng nghỉ giữa các cụm lỗi vật lý. Khi tắt máy qua đêm rồi bật lại (2026-09-03), lỗi tái phát ở
   DẠNG KHÁC (hai chiều, không phải một chiều như lỗi gốc — xem `regression-after-reboot/`), và soak
   30 phút lặp lại ngay sau đó (`soak-after-reconnect/`) **tái hiện đầy đủ triệu chứng gốc**: 1071 lần
   `TaskCanRx: FAULT`, `stm32_ok` dao động 1↔0 liên tục ngay giây thứ 15, bus load thật rơi từ ~62%
   xuống còn ~31%.
4. **Việc cắm lại đầu nối bằng tay của người dùng (trong lúc điều tra `regression-after-reboot/`) chỉ
   tạm thời cải thiện, không phải fix dứt điểm** — dmesg cho thấy một khoảng nghỉ ~14 phút sau đó, rồi
   lỗi lại tiếp diễn.

**Kết luận**: vấn đề **CHƯA được khắc phục**. Bước tiếp theo cần thiết là **kiểm tra/sửa phần cứng**
(đầu nối CAN_H/CAN_L/GND, mối hàn, transceiver) — không phải thêm fix phần mềm hay soak test dài hơn
(xem mục "Khuyến nghị bước tiếp theo" cuối tài liệu, đã cập nhật theo hướng này). Phần còn lại của
tài liệu này (mục 1-7 bên dưới) giữ nguyên làm hồ sơ điều tra gốc — có giá trị lịch sử và các fix vẫn
đúng — nhưng **đừng đọc kết luận "45 phút sạch" ở mục 5.1 như bằng chứng vấn đề đã đóng**.

```
busoff-investigation/   giai đoạn đầu — phát hiện bug thứ tự socket + Jetson controller bị kẹt
signal-integrity/       đo baseline 500kbps sau khi bật hiển thị error frame
125kbps-measurement/    đo 60s ngay sau khi hạ bitrate — vẫn còn REC=127 và mất frame
125kbps-soak/           soak test 900s (15 phút) sau khi chia tần cả 2 phía — 1 lần reboot chưa rõ nguyên nhân, còn lại sạch
125kbps-soak-30min/     soak test 1800s (30 phút) lặp lại — 0 bất thường, không tái hiện reboot
regression-after-reboot/ 2026-09-03 — lỗi tái phát sau tắt/mở máy qua đêm, DẠNG KHÁC (hai chiều,
                         tx≈rx) — dmesg + kiểm tra lại theo thời gian thực xác định đây là lỗi TIẾP
                         XÚC VẬT LÝ GIÁN ĐOẠN (đầu nối/dây), theo cụm, CHƯA hết — không liên quan tới
                         5 fix ở mục 3 bên dưới
soak-after-reconnect/   2026-09-03 — soak 1800s lặp lại sau phát hiện trên: TÁI HIỆN ĐẦY ĐỦ triệu
                         chứng gốc (1071 lần FAULT, busoff 1563 lần, bus load rơi còn 12.3%) — xác
                         nhận vấn đề vật lý vẫn đang hoạt động, chưa được khắc phục
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
`scripts/run_soak.sh`/`scripts/analyze_soak.py`). `can0` đọc được `ERROR-ACTIVE, berr tx=0 rx=0`
ngay trước khi đo (xác nhận trong `run_info.txt`) sau một `systemctl restart can0.service`.

**Đính chính (2026-09-03, xem [`regression-after-reboot/`](regression-after-reboot/README.md))**:
câu trên dễ gây hiểu lầm rằng bản thân lệnh `systemctl restart can0.service` "làm sạch" berr-counter
— **không đúng**. Đã kiểm chứng: lệnh này (và cả `ip link down/up` thủ công) không hề reset
berr-counter phần cứng, nó chỉ giữ nguyên giá trị cũ. `tx=0 rx=0` đọc được ở đây chỉ vì counter vốn
đã ở 0 sẵn tại đúng thời điểm đó (bus tình cờ đang sạch), không phải do lệnh restart gây ra. Cách
reset thật sự (đã kiểm chứng) là nạp lại module driver: `sudo rmmod mttcan && sudo modprobe mttcan`
— `scripts/run_soak.sh` đã được cập nhật để làm việc này trước mỗi lần đo, và `scripts/analyze_soak.py`
nay đọc berr-counter xuất phát từ `run_info.txt` và tính delta thay vì giả định bắt đầu từ 0. Kết
luận "0 bất thường" của lần soak này (mục dưới) không bị ảnh hưởng bởi đính chính này — số liệu
`berr-counter` đo được xuyên suốt 1800s vẫn đúng, chỉ có mô tả *lý do* nó bắt đầu ở 0 là sai.

Firmware STM32 tại thời điểm này đã có thêm log `[BOOT]` (đọc `RCC->RSR` lúc khởi
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
| Bus load thực | candump (chỉ chiều STM32) = 282.3 fr/s; cộng TX packets Jetson (netdevice, +289.8 fr/s — không hiện trong candump vì `CanTransport::open()` tắt `CAN_RAW_LOOPBACK` trên chính socket gửi, nên frame Jetson tự phát không được kernel loopback về BẤT KỲ socket local nào, kể cả `candump`, xem đính chính 2026-09-04 bên dưới bảng) = **~572.1 fr/s × 1080µs ≈ 61.79%** |
| Phân bố ID | `0x100=181714(100.9Hz) 0x102=181110(100.6Hz) 0x103=36345 0x1FF=36344 0x104=36330 0x101=36231(~20.1-20.2Hz mỗi ID)` — khớp thiết kế |
| S1/S2/S3 | `S1=1632 S2=1549 S3=1580` không đổi suốt 30 phút |

**Đính chính (2026-09-04)**: hàng "Bus load thực" ở trên ban đầu chỉ ghi `282.3 fr/s ≈ 30.48%` — con số
này CHỈ đếm được chiều STM32->Jetson (candump không thấy frame Jetson tự phát, lý do nêu trong ô bên
trên), nên thực chất là bus load MỘT NỬA, không phải toàn bus. Đã sửa lại thành `~572.1 fr/s ≈ 61.79%`
(khớp thiết kế lý thuyết ~62%, giống cách tính đã làm đúng ở mục 5 cho lần soak 900s). `scripts/analyze_soak.py`
đã được sửa để tự tính cả 2 chiều (đọc thêm `tx_packets` từ `canstats.log`) thay vì chỉ đếm dòng
`candump.log` — không có bảng/kết luận nào khác trong tài liệu này bị ảnh hưởng bởi lỗi này (mọi số
liệu khác — FAULT, berr-counter, gap, ID distribution — không liên quan tới cách tính bus load).

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

- **(Cập nhật 2026-09-04, quan trọng nhất)** Toàn bộ đánh giá "0 bất thường"/"đã hết lỗi" ở mục 5 và
  5.1 dưới đây **không còn đúng như một kết luận cuối cùng** — xem "TRẠNG THÁI HIỆN TẠI" ở đầu tài
  liệu và [`regression-after-reboot/README.md`](regression-after-reboot/README.md) +
  [`soak-after-reconnect/README.md`](soak-after-reconnect/README.md). Nguyên nhân gốc là phần cứng
  (tiếp xúc chập chờn), lỗi hoạt động theo cụm ngắt quãng, và 45 phút soak sạch ngày 2026-09-02 chỉ là
  một khoảng nghỉ giữa các cụm — KHÔNG phải bằng chứng đã hết lỗi. Vấn đề hiện **CHƯA GIẢI QUYẾT**.
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

**(Cập nhật 2026-09-04)** Với bằng chứng dứt điểm rằng nguyên nhân gốc là phần cứng (mục "TRẠNG THÁI
HIỆN TẠI" ở đầu tài liệu), khuyến nghị #1 dưới đây (chạy thêm soak test phần mềm) **không còn là ưu
tiên** — đã có đủ bằng chứng, thêm soak nữa chỉ đo lại cùng một lỗi đã biết, không giúp sửa nó. Ưu
tiên bây giờ là phần cứng:

0. **(Mới, ưu tiên cao nhất)** Kiểm tra vật lý kỹ đường dây CAN: rung nhẹ từng đầu nối CAN_H/CAN_L/GND
   dọc theo dây trong lúc app đang chạy và `candump`/berr-counter đang mở, để khoanh vùng chính xác
   điểm lỏng (đầu Jetson, đầu STM32, hay giữa dây — xem khuyến nghị chi tiết ở
   `soak-after-reconnect/README.md`). Sau khi sửa/thay đầu nối, lặp lại soak 30 phút
   (`scripts/run_soak.sh 1800 <thư mục mới>`) để xác nhận — lần này nếu sạch thật, cần thêm ít nhất
   một chu kỳ tắt/mở máy qua đêm nữa trước khi tin, vì bài học từ chính tài liệu này (mục 5.1 tưởng
   sạch, hoá ra không) là 1 lần soak sạch không đủ để kết luận đã hết.
1. ~~Chạy soak test dài hơn~~ Đã làm 2 lần (mục 5.1 hôm 2026-09-02, 30 phút "sạch"; và
   `soak-after-reconnect/` hôm 2026-09-04, 30 phút "tái hiện đầy đủ lỗi") — **kết quả trái ngược nhau
   chứng minh chính bài học của mục này**: soak test phần mềm không phân biệt được "đã hết lỗi" với
   "đang ở khoảng nghỉ giữa các cụm lỗi phần cứng". Không nên chạy thêm soak thuần phần mềm nữa cho
   tới khi phần cứng được sửa.
2. Điều tra riêng lỗi UART print bị đứng hình ở lần soak đầu (mục 5) — nguyên nhân chưa xác định, cần
   xem lại code counter/print trong firmware (không phải DMA — retarget dùng `HAL_UART_Transmit`
   blocking). Không liên quan tới CAN nhưng ảnh hưởng khả năng chẩn đoán tương lai.
3. Cân nhắc thêm test `cangen` bên trong repo (có log) để việc bác bỏ giả thuyết signal-integrity ở
   mục 2.6 có bằng chứng tái lập được, thay vì chỉ dựa vào báo cáo ngoài phiên.
4. ~~Commit working tree vào git~~ **Đã làm** — xem lịch sử commit trong repo (bắt đầu từ
   "Initial commit: STM32H723 firmware + Jetson vision control").
