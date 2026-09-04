# Tái phát sau reboot qua đêm: lỗi hai chiều (ERROR-WARNING tx≈rx), khác hẳn triệu chứng gốc

Ngày đo: 2026-09-03. Bối cảnh: hôm qua (2026-09-02) hệ thống chạy 45 phút liên tục không lỗi
(`berr 0/0`, xem [`../README.md`](../README.md) mục 5.1). Sau khi tắt máy qua đêm và bật lại
sáng/tối nay, lỗi CAN trở lại — nhưng **không phải cùng dạng lỗi cũ**.

## 0. Triệu chứng mới, khác triệu chứng gốc

| | Lỗi gốc (trước khi hạ bitrate, mục 2.4 README chính) | Lỗi hôm nay |
|---|---|---|
| Chiều lỗi | Một chiều tuyệt đối: 500k `TEC=0 REC=127`; 125k `TEC=255 REC=0` | **Hai chiều gần bằng nhau**: `tx 106 rx 103` (báo cáo ban đầu của người dùng) |
| Ý nghĩa | Một node phát sạch, một node nhận lỗi -> khoanh vùng được vào 1 chiều tín hiệu | tx≈rx cân bằng -> theo lý thuyết CAN, kiểu lỗi này thường gắn với **tầng vật lý** (cả 2 chiều cùng thấy bit lỗi trên cùng đường dây), không phải lỗi cấu hình một phía |
| Bit timing | (đã là nghi phạm ở giai đoạn đầu, đã sửa) | Đã kiểm tra lại hôm nay: `(1+87+87)/200=0.875`, `200 tq × 40ns = 125kbps`, `brp 2, clock 50MHz -> 40ns/tq` — **khớp, không phải nguyên nhân** |
| Dữ liệu | Mất hẳn, `stm32_ok` rơi về 0 vĩnh viễn | Vẫn chảy một phần: `roll_imu` còn biến thiên (0.56-0.63) nên 0x100 vẫn tới, nhưng đủ mất frame để `stm32_ok` dao động 1↔0 liên tục |

Người dùng đã tự kiểm tra và cắm lại kết nối vật lý bằng tay trước khi phiên điều tra này bắt đầu.

## 1. Nhiệm vụ 1 — Cô lập controller Jetson bằng loopback

Chi tiết đầy đủ + caveat: [`task1_loopback_notes.txt`](task1_loopback_notes.txt),
frame log: [`task1_loopback_candump.log`](task1_loopback_candump.log).

```
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000 loopback on
sudo ip link set can0 up
candump can0 &  ->  cansend can0 123#DEADBEEF  ->  sleep 1; kill
```

**Kết quả: frame `123#DEADBEEF` quay lại đúng** (2 dòng trong log — echo TX + loopback RX) ->
**controller Jetson (silicon + driver mttcan) khoẻ**, lỗi không nằm ở chính con chip CAN trên Jetson.

**Caveat quan trọng phát hiện được**: `loopback on` trên driver `mttcan` này **không ngắt kết nối
khỏi bus vật lý** — nó chỉ thêm echo cục bộ của frame tự phát vào hàng đợi RX. Trong lúc bật
loopback, counter RX packets (`ip -s link show can0`) vẫn tăng vì frame thật từ STM32 vẫn tới —
xác nhận dây vẫn nối trong suốt bài test này. Nên phép thử này chỉ chứng minh được đường dữ liệu
nội bộ TX->RX của controller hoạt động đúng, **không cô lập hoàn toàn được khỏi bus vật lý** (muốn
cô lập thật sự cần rút dây CAN khỏi Jetson, chưa làm trong phiên này).

**Phát hiện phụ (không nằm trong nhiệm vụ gốc nhưng liên quan)**: `sudo systemctl restart
can0.service` (và cả `ip link down/up` thủ công) **không reset berr-counter (TEC/REC) phần cứng**
— counter chỉ giữ nguyên giá trị cũ. Dòng "ERROR-ACTIVE, berr tx=0 rx=0" từng thấy ngay sau restart
trong `run_info.txt` của các lần soak trước không phải do bản thân lệnh restart gây ra, mà vì lúc
đó counter vốn đã ở 0 sẵn. Gửi 3 frame thật lên bus (`cansend can0 7DF#...`) làm `tx` giảm
`108->105` (mỗi lần phát thành công trừ 1 theo chuẩn CAN) — xác nhận đường TX vật lý đã hoạt động
lại bình thường vào lúc đó (xem mục 2 để biết vì sao: cơn bão lỗi thật đã dừng ~10-15 phút trước khi
phiên này bắt đầu).

## 2. Nhiệm vụ 2 — Baseline với app TẮT + pháp y `dmesg`

Lệnh chạy đúng như yêu cầu:
```
sudo systemctl restart can0.service
ip -details -s link show can0        # -> link_before.txt
timeout 30 candump -tz can0 > baseline_today.log
ip -details -s link show can0        # -> link_after.txt
```
Phân tích đầy đủ: [`task2_baseline_analysis.txt`](task2_baseline_analysis.txt). Xác nhận app
(`balance_ball_main`) không chạy trong suốt phiên này (`pgrep` rỗng).

### 2.1. Kết quả số liệu 30 giây

| Chỉ số | Kết quả |
|---|---|
| Tổng frame | 6425 trong 30s (candump) |
| Phân bố ID | `0x100=2625(87.5Hz) 0x102=1767(58.9Hz) 0x103=533(17.8Hz) 0x1FF=528(17.6Hz) 0x101=505(16.8Hz) 0x104=467(15.6Hz)` |
| Error frame (`ERRORFRAME`) | **0** |
| `berr-counter` trước/sau | `tx 105 rx 0` -> `tx 105 rx 0` — **không đổi** trong 30s (RX sạch tuyệt đối; TX đứng yên vì app tắt nên Jetson không phát gì để counter tăng/giảm — không phải bằng chứng "TX sạch", chỉ là "TX không hoạt động") |
| `error-warn` / `error-pass` / `bus-off` (netlink, tích luỹ từ lúc driver load) | `67 / 508 / 0` -> `67 / 508 / 0` — **không tăng thêm trong 30s này** |
| RX dropped (netdevice) | `115651 -> 115651`, delta = 0 |

**Bus SẠCH theo tiêu chí berr-counter/error-frame khi app tắt** — khớp nhánh "quay lại nhánh cũ,
lỗi phụ thuộc tải" theo tiêu chí đề ra ban đầu.

### 2.2. NHƯNG: phát hiện mới không nằm trong tiêu chí gốc — khoảng lặng bus tới 691ms

Dù berr-counter và error-frame hoàn toàn sạch, phân tích khoảng cách giữa các frame liên tiếp
(toàn bus, không phân biệt ID) phát hiện **8 khoảng lặng >100ms trong 30 giây**, tối đa 691ms,
xảy ra thành 2 cụm liên tiếp gần nhau (~t=9.9-14.3s và ~t=25.9-29.6s), ảnh hưởng đồng thời cả 6 ID
(không phải một ID lẻ tẻ bị trễ — toàn bus im lặng cùng lúc):

```
t= 9.896s -> 10.360s   gap=463.9ms
t=11.863s -> 12.339s   gap=476.3ms
t=12.862s -> 13.328s   gap=466.4ms
t=14.149s -> 14.318s   gap=168.8ms
t=25.926s -> 26.618s   gap=691.2ms
t=27.299s -> 27.607s   gap=308.0ms
t=28.152s -> 28.596s   gap=444.2ms
t=28.961s -> 29.585s   gap=624.5ms
```

**Đây là khác biệt rõ so với soak test 30 phút hôm qua** (`../125kbps-soak-30min/`), nơi
"Gap candump >100ms: **0**" trong toàn bộ 1800 giây. Hôm nay chỉ 30 giây đã có 8 lần. Vì không có
frame lỗi/bit-error đi kèm (không ai đang phát trong lúc im lặng nên không có gì để bị lỗi bit),
kiểu "im lặng sạch" này **không** làm berr-counter tăng — nó không được phát hiện bởi tiêu chí
gốc của Nhiệm vụ 2 (chỉ nhìn berr-counter + error-frame). Giả thuyết hợp lý nhất: đứt tiếp xúc
gián đoạn ngắn (đầu nối lỏng) — bus tạm thời hở mạch vài trăm ms rồi tự nối lại, không đủ để tạo
bit lỗi vì không có ai đang truyền đúng lúc đó. **Chưa xác định được nguyên nhân chắc chắn — cần
capture dài hơn 30s để xem tần suất/tính chu kỳ của các khoảng lặng này.**

### 2.3. Pháp y `dmesg` — bằng chứng mạnh nhất trong phiên này, xảy ra TRƯỚC khi bắt đầu điều tra

Toàn bộ log: [`dmesg_full.log`](dmesg_full.log), tóm tắt: [`task2_dmesg_timeline.txt`](task2_dmesg_timeline.txt).
Máy khởi động lúc `2026-09-03 22:37:09` (`uptime -s`).

| Thời điểm | Sự kiện |
|---|---|
| 22:37:18 | Driver `mttcan` đăng ký (`net can0: mttcan device registered`) |
| 22:37:21 | `can0.service` cấu hình interface lúc boot (`Bitrate set`) |
| **22:40:10 -> 22:48:37** | **67 lần `entered error warning state`** — bắt đầu chỉ ~3 phút sau boot |
| 22:48:41 | `NET: Registered PF_CAN protocol family` — **module CAN core/raw mới được nạp lần đầu** (nghĩa là **chưa hề có socket CAN nào của userspace — kể cả `candump`/app — tồn tại được trước thời điểm này**) |
| **22:40:35 -> 22:54:47** | **508 lần `entered error passive state`**, chia thành nhiều cụm (đỉnh điểm 160 lần trong phút 22:48, 142 lần trong phút 22:49) |
| 22:54:47 | **Lần `error passive` cuối cùng ghi nhận được** |
| 22:54:50 trở đi | Chuỗi `Bitrate set` do phiên điều tra này thực hiện (Nhiệm vụ 1, 2) — không có `error passive`/`error warning` mới nào xuất hiện sau mốc này |

**Ý nghĩa quan trọng nhất**: cơn bão lỗi (508 lần error-passive, 67 lần error-warning) xảy ra
**từ 22:40 đến 22:54:47 — TRƯỚC KHI phiên điều tra này (và mọi lệnh `candump`/`cansend`/app) bắt
đầu chạy**. Không chỉ vậy, **trong suốt 22:40:10 đến 22:48:41, module `can`/`can_raw` của kernel
còn CHƯA ĐƯỢC NẠP** — tức là về mặt kỹ thuật, không một socket CAN userspace nào (app, candump,
hay bất kỳ tiến trình nào) có thể tồn tại để gây ra các lỗi này. Đây là **bằng chứng kernel-level
độc lập** rằng phần lớn cơn bão lỗi hôm nay là lỗi thuần phần cứng/tầng vật lý (controller tự sinh
lỗi khi arbitrate lên một bus có vấn đề điện), hoàn toàn không phụ thuộc ứng dụng hay cấu hình
socket — khớp với suy luận ban đầu của người dùng ("lỗi hai chiều cân bằng là dấu hiệu lỗi tầng
vật lý").

Từ 22:54:47 đến khoảng 23:01 (~6 phút) không có thêm lỗi passive/warning nào — đây là lúc bản báo
cáo này ban đầu (nhầm) kết luận "cơn bão lỗi đã hết hẳn". Mục 2.4 dưới đây, đo lại ~15 phút sau, cho
thấy kết luận đó **sai** — lỗi chỉ đang trong một cụm im lặng tạm thời, không phải đã hết.

### 2.4. Kiểm tra lại theo thời gian thực lúc 23:15-23:17 — lỗi VẪN ĐANG XẢY RA, không phải "đã hết"

Trong lúc thực hiện VIỆC 1 (tìm cách reset thật berr-counter), đã thử `sudo rmmod mttcan && sudo
modprobe mttcan` (nạp lại module driver — xem mục 1 phần "Phát hiện phụ" đã cập nhật). Ngay sau khi
lên lại, `tx=0 rx=0` thật sự (ifindex mới, mọi counter về 0 — xác nhận đây mới là cách reset thật, còn
`systemctl restart`/`ip link down-up` thì không). Nhưng chỉ **~1 giây sau đó**, `rx` đã bắt đầu tăng,
và trong 40 giây theo dõi tiếp theo (lấy mẫu 2s/lần, xem đầy đủ ở
[`postreload_berr_trace.log`](postreload_berr_trace.log)), `rx` dao động liên tục trong khoảng
**73-127** (127 = sát ngay dưới ngưỡng ERROR-PASSIVE 128), phần lớn thời gian ở trạng thái
`ERROR-WARNING`, trong khi `tx` đứng yên ở 0 (app vẫn tắt, không có gì để phát). `dmesg` xác nhận
lỗi `error passive` mới nhất tính đến lúc viết dòng này là **23:16:52** — tức là đang xảy ra ngay
trong lúc viết báo cáo, không phải chuyện của 20 phút trước.

Candump chạy song song 40 giây đó ([`postreload_candump.log`](postreload_candump.log)) cho kết quả
tệ hơn hẳn baseline "sạch" ở mục 2.1:

| Chỉ số | Baseline mục 2.1 (22:57, 30s) | Kiểm tra lại mục 2.4 (23:16, 40s) |
|---|---|---|
| Tổng frame | 6425 (214 fr/s) | 1832 (**44.7 fr/s** — giảm ~5 lần) |
| `0x102` | 1767 frame (58.9Hz) | **52 frame (1.3Hz)** — gần như biến mất |
| `0x100` | 2625 frame (87.5Hz) | 1068 frame (26.8Hz) |
| Gap >100ms | 8 lần / 30s | **40 lần / 40s** — gần như liên tục |

**Kết luận sửa lại**: cơn bão lỗi từ mục 2.3 (22:40-22:54:47) **không phải là kết thúc dứt điểm** —
đó chỉ là một khoảng nghỉ giữa các đợt. Bus tiếp tục dao động giữa "sạch vài phút" và "lỗi tích cực"
theo từng đợt (giống hệt kiểu phân bố theo phút đã thấy ở mục 2.3: 22:40 nhẹ, im lặng tới 22:46, bùng
lên 22:46-22:49, nhẹ lại, bùng lên 22:51...). Baseline sạch ở mục 2.1 (22:57:22-22:57:52) chỉ là một
trong những khoảng nghỉ đó, không phải bằng chứng "đã ổn định". Kiểu dao động cụm-rồi-nghỉ này **khớp
kinh điển với tiếp xúc điện chập chờn** (đầu nối/mối hàn lỏng phản ứng với rung động/nhiệt độ/vị trí
dây) hơn là với bất kỳ giả thuyết phần mềm nào — càng củng cố thêm bằng chứng ở mục 2.3 rằng đây là
lỗi vật lý, chỉ có điều **chưa hết**, còn đang tiếp diễn ngắt quãng tại thời điểm viết báo cáo này.

## 3. Kết luận

- **Lỗi vật lý gián đoạn (đầu nối/tiếp xúc chập chờn), KHÔNG phải lỗi cấu hình phần mềm, và CHƯA
  kết thúc** — đây là kết luận cuối cùng sau khi kiểm tra lại (mục 2.4), thay cho kết luận ban đầu
  "đã hết" ở mục 2.3 (chỉ đúng tại thời điểm viết, sai khi kiểm tra lại 15 phút sau). Bằng chứng:
  (a) cơn bão lỗi ban đầu (508+67 sự kiện) xảy ra khi module CAN core còn chưa nạp — không thể do
  app/socket gây ra (mục 2.3); (b) lỗi tái diễn thành từng cụm xen kẽ với khoảng nghỉ vài phút, kể cả
  sau khi reset cứng driver (mục 2.4) — mẫu hình kinh điển của tiếp xúc điện chập chờn; (c) khi lỗi
  đang hoạt động, cả tần suất frame lẫn số khoảng lặng >100ms đều xấu đi rõ rệt so với baseline sạch.
  **Baseline "sạch" đo được ở mục 2.1 không phải bằng chứng vấn đề đã hết — chỉ là một khoảng nghỉ
  ngẫu nhiên giữa các cụm lỗi.**
- **Nhiệm vụ 1** xác nhận controller Jetson khoẻ (loopback OK, TX vật lý thật cũng phát thành công
  ở một thời điểm khoảng nghỉ) — loại trừ được khả năng lỗi nằm ở chính chip/driver CAN Jetson; lỗi
  nằm ở đường truyền vật lý phía ngoài controller (dây/đầu nối/transceiver).
- **[`../soak-after-reconnect/`](../soak-after-reconnect/README.md) (VIỆC 2, soak 30 phút với app
  chạy, chạy ngay sau mục này) xác nhận dứt điểm**: 1071 lần `TaskCanRx: FAULT`, `stm32_ok` dao động
  1↔0 liên tục ngay trong 15 giây đầu, bus load rơi từ ~30% (hôm qua) xuống còn 12.3%, 1354 gap
  >100ms trong 1800s (so với 0 hôm qua). Đây chính là triệu chứng gốc tái hiện đầy đủ dưới tải thật —
  không còn nghi ngờ gì nữa, lỗi vật lý gián đoạn vẫn đang hoạt động, chưa hề "đã hết".
- Khoảng lặng bus tới 691ms thấy ở mục 2.2 **không phải hiện tượng riêng lẻ** — đã được xác nhận là
  cùng một nguyên nhân với 1354 gap >100ms thấy trong soak 30 phút ở `../soak-after-reconnect/`
  (VIỆC 2), không cần theo dõi thêm như một vấn đề tách biệt nữa.
- **Không mâu thuẫn với 5 fix đã áp dụng ở `../README.md`** (bit timing, AutoRetransmission,
  rate-limit bus-off, bitrate 125k, chia tần) — các fix đó vẫn đúng và không phải nguyên nhân của
  đợt lỗi này (bit timing đã re-verify ở đầu tài liệu này). Đây là một lớp vấn đề khác: **lỗi vật lý
  gián đoạn qua đêm (đầu nối/tiếp xúc), không phải lỗi cấu hình phần mềm**.

## 4. Nhiệm vụ 3 — bỏ qua theo quyết định của người dùng

Người dùng đã xem bằng chứng `dmesg` ở mục 2.3 và quyết định **bỏ qua Nhiệm vụ 3** (hạ bitrate
50kbps chỉ phía Jetson) — bằng chứng cơn bão lỗi xảy ra khi module `can`/`can_raw` còn chưa nạp được
đánh giá là đủ dứt điểm để loại trừ mọi giả thuyết phần mềm, nên phép thử bitrate không cần thiết
nữa. (Mục 2.4, đo sau quyết định này, càng củng cố thêm kết luận vật lý — không làm thay đổi quyết
định bỏ qua Nhiệm vụ 3.)

## 5. Việc tiếp theo đã thực hiện trong cùng phiên (theo yêu cầu người dùng)

- **Sửa 2 giả định sai trong tài liệu** (đã sửa ở mục 1 và 2.4 phía trên, cùng
  `../README.md` mục 5.1 và `scripts/run_soak.sh`): `systemctl restart can0.service` không reset
  berr-counter; `loopback on` không cô lập khỏi bus vật lý.
- **Tìm được cách reset berr-counter thật**: `sudo rmmod mttcan && sudo modprobe mttcan` (nạp lại
  driver) — đã kiểm chứng cho `tx=0 rx=0` và mọi counter netlink khác về 0 thật sự, khác với
  `systemctl restart`/`ip link down-up`. `scripts/run_soak.sh` đã cập nhật để dùng cách này trước mỗi
  lần đo; `scripts/analyze_soak.py` đã cập nhật để đọc berr-counter xuất phát từ `run_info.txt` và in
  delta thay vì giả định bắt đầu từ 0 (phòng trường hợp reset thất bại hoặc dùng log cũ).
- Soak test 30 phút xác nhận lại (`../soak-after-reconnect/`) — xem README ở đó để biết kết quả so
  sánh trực tiếp với `../125kbps-soak-30min/` hôm qua.
- **Đối chiếu `sjw`** (2026-09-04, theo yêu cầu người dùng): so sánh `run_info.txt` của
  `../125kbps-soak-30min/` (hôm qua) với `link_before.txt`/`link_after.txt` ở đây và
  `../soak-after-reconnect/run_info.txt` (hôm nay) — cả 3 đều đọc **`sjw 12`** giống hệt nhau. `sjw`
  không phải biến số thay đổi giữa 2 lần đo (driver `mttcan` tự tính `sjw` từ bitrate+sample-point yêu
  cầu qua `ip link set ... type can bitrate 125000 sample-point 0.875`, không phải giá trị người dùng
  tự đặt tay, nên giữ nguyên khi lặp lại đúng lệnh cấu hình) — loại trừ được đây là biến số gây khác
  biệt giữa hôm qua/hôm nay.
- **Sửa lỗi tính bus load trong `scripts/analyze_soak.py`** (2026-09-04, theo yêu cầu người dùng):
  script cũ chỉ đếm frame trong `candump.log`, vốn không bao giờ chứa frame do Jetson tự phát (vì
  `CanTransport::open()` tắt `CAN_RAW_LOOPBACK` trên chính socket gửi — xác nhận: không có `0x200`
  series nào trong `candump.log` của cả 2 lần soak, dù Jetson chắc chắn có phát vì STM32 vẫn ACK được
  frame Jetson trong lúc bus còn sạch), nên báo bus load thấp hơn ~2 lần so với thực tế (30.48% thay vì
  61.79% thật cho soak hôm qua). Đã sửa: script nay đọc thêm `tx_packets` (netdevice, không phụ thuộc
  `CAN_RAW_LOOPBACK`) từ `canstats.log` và cộng vào. Số liệu đã sửa lại trong `../README.md` mục 5.1
  và `../soak-after-reconnect/README.md`.

## 6. Danh sách file bằng chứng

| File | Nội dung |
|---|---|
| `task1_loopback_notes.txt` | Ghi chú đầy đủ Nhiệm vụ 1, gồm caveat loopback không cô lập vật lý và phát hiện phụ về berr-counter không tự reset |
| `task1_loopback_candump.log` | Log candump trong lúc loopback (2 dòng, frame quay lại đúng) |
| `link_before.txt` / `link_after.txt` | `ip -details -s link show can0` trước/sau 30s baseline |
| `baseline_today.log` | Raw candump 30s, app tắt (mục 2.1) |
| `task2_baseline_analysis.txt` | Phân tích đầy đủ: phân bố ID, gap, berr-counter, so sánh trước/sau |
| `dmesg_full.log` | Toàn bộ dmesg từ lúc boot, bằng chứng chính cho kết luận mục 2.3 |
| `task2_dmesg_timeline.txt` | Tóm tắt dmesg theo phút + các dòng CAN không phải lỗi (module load, reconfig) |
| `task_live_recheck_notes.txt` | Ghi chú kiểm tra lại theo thời gian thực lúc 23:15-23:17 (mục 2.4) |
| `postreload_berr_trace.log` | Trace berr-counter mỗi 2s trong 40s ngay sau khi reset cứng driver (mục 2.4) |
| `postreload_candump.log` | Candump song song 40s cùng lúc — cho thấy frame rate giảm ~5 lần, 40 gap >100ms |
