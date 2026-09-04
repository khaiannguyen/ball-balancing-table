# Wiggle test — định vị điểm tiếp xúc chập chờn trên dây CAN

Bối cảnh: đã đổi module transceiver SN65HVD230 phía Jetson, triệu chứng
KHÔNG đổi — vẫn `TEC=0` (hoặc thấp), `REC` leo cao (từng thấy 117-127),
`TX packets=0` khi không có app chạy nhưng vẫn nhận frame lỗi liên tục từ
STM32. Điều này loại trừ chính con chip transceiver phía Jetson là nguyên
nhân (xem lịch sử điều tra ở [`../README.md`](../README.md),
[`../regression-after-reboot/README.md`](../regression-after-reboot/README.md)).

Mục tiêu bài test này: tương quan **thời điểm thao tác tay** (lắc/ấn từng
điểm tiếp xúc vật lý) với **thời điểm berr-counter tăng đột biến**, để định
vị chính xác điểm chập chờn — hoặc loại trừ hoàn toàn khả năng lỗi tiếp xúc.

## Công cụ: `scripts/wiggle_monitor.sh`

Cố ý **không dùng candump, không chạy app** — chỉ đọc
`ip -details -s link show can0` mỗi 0.5s, để cô lập tối đa khỏi phần mềm và
chỉ còn lại tầng vật lý/driver.

Reset `can0` bằng `modprobe -r mttcan && modprobe mttcan` (reload driver
thật sự) — **không** dùng `ip link down/up` hay `systemctl restart
can0.service`, vì đã xác nhận trong
[`../regression-after-reboot/README.md`](../regression-after-reboot/README.md)
(và ghi lại trong `scripts/run_soak.sh`) rằng hai cách đó **không** reset
được berr-counter (TEC/REC) hay các counter error-warn/error-pass/bus-off
trên driver `mttcan` này.

Script tự động in "banner" hướng dẫn từng bước lên màn hình đúng lúc VÀ ghi
banner đó (kèm epoch timestamp) vào `wiggle_raw.log` — nhờ vậy mốc thời
gian đối chiếu ở bước phân tích lấy trực tiếp từ log, không phụ thuộc vào
việc người thao tác hô to/nhớ giờ chính xác.

### Cách chạy

Chạy trực tiếp trong terminal của bạn (không qua agent) để nhìn được output
real-time trong lúc tay đang thao tác:

```
sudo scripts/wiggle_monitor.sh
```

Mặc định: outdir `wiggle_run_<timestamp>/`, thời lượng 180s (3 phút).
Có thể chỉ định khác: `sudo scripts/wiggle_monitor.sh <outdir> <duration_s>`.

### Lịch trình thao tác (tự động hiện trên màn hình, ~20s/bước)

| Bước | Thời điểm (giây) | Thao tác |
|---|---|---|
| a | 0-20 | **Baseline** — không chạm gì |
| b | 20-40 | Lắc nhẹ đầu connector CAN phía **Jetson** (module mới) |
| c | 40-60 | Lắc nhẹ đầu connector CAN phía **STM32** |
| d | 60-80 | Lắc/uốn nhẹ **dọc theo dây CANH/CANL** giữa hai board |
| e | 80-100 | Ấn nhẹ **từng chân header** module Jetson (không tháo ra) |
| f | 100-120 | Ấn nhẹ **từng chân header** module STM32 |
| g | 120-140 | Kiểm tra điểm nối **GND chung** giữa hai board (lắc nhẹ) |
| h | 140-180 | **Buông tay hoàn toàn** — quan sát lỗi có còn tăng không khi không chạm |

Bước h (buông tay, không có trong yêu cầu gốc nhưng thêm vào vì nó là phép
đối chứng bắt buộc: nếu lỗi vẫn tăng đều trong 40s cuối này dù không ai
chạm, đó là bằng chứng trực tiếp rằng nguyên nhân KHÔNG phải tiếp xúc cơ khí).

### Output mỗi lần chạy

- `run_info.txt` — trạng thái `can0` ngay sau reset (phải là
  `ERROR-ACTIVE, berr tx=0 rx=0` nếu reset thành công) + lịch trình bước.
- `wiggle_raw.log` — dump thô đầy đủ `ip -details -s link show can0` mỗi
  0.5s kèm epoch timestamp, và banner mỗi lần chuyển bước.
- `wiggle_berr.csv` — bản rút gọn dạng CSV mỗi mẫu: bước, epoch,
  elapsed_s, can_state, berr_tx, berr_rx, rx_packets, rx_errors,
  rx_dropped, tx_packets, restarts, bus_errors, arbit_lost, error_warn,
  error_pass, bus_off — dùng để tính delta giữa các bước nhanh hơn là parse
  log thô.

## Kết quả

Đã chạy 1 lần: [`wiggle_run_20260904_072240/`](wiggle_run_20260904_072240/) (2026-09-04 07:22,
180s). `run_info.txt` xác nhận reset sạch trước khi bắt đầu (`ERROR-ACTIVE, berr tx=0 rx=0`, đúng
125kbps/sample-point 0.875 như `can_up.sh`).

### Số lần `entered error passive` (cột `error_pass`, tích luỹ) theo từng bước, 20s/bước

| Bước | Thao tác | Δ error_pass trong bước | Quy đổi /20s |
|---|---|---|---|
| a | Baseline, không chạm | 0 | 0 |
| b | Lắc connector Jetson | 21 | 21 |
| c | Lắc connector STM32 | 50 | 50 |
| d | Lắc/uốn dọc dây CANH/CANL | **113** | **113** (cao nhất) |
| e | Ấn chân header Jetson | 93 | 93 |
| f | Ấn chân header STM32 | 47 | 47 |
| g | Lắc điểm nối GND chung | 78 | 78 |
| h | **Buông tay hoàn toàn** (40s) | 171 | ~85 |

Trong bước h, tốc độ tăng còn **tăng dần theo thời gian** dù không ai chạm gì: `+34` (140-150s),
`+36` (150-161s), `+46` (161-172s, quy đổi/10s), `+65` (172-180s, quy đổi/10s) — nhanh hơn ở cuối
bước hơn là chậm dần/dừng lại.

### Diễn giải — **KẾT LUẬN NGƯỢC LẠI với báo cáo `regression-after-reboot/`**

Bước h được thiết kế làm phép đối chứng bắt buộc (xem mục thiết kế bài test ở trên): *"nếu lỗi vẫn
tăng đều trong 40s cuối này dù không ai chạm, đó là bằng chứng trực tiếp rằng nguyên nhân KHÔNG phải
tiếp xúc cơ khí."* Đó chính xác là những gì xảy ra — không chỉ tăng đều, mà còn **tăng tốc** trong
lúc hoàn toàn không chạm vào bất kỳ điểm tiếp xúc nào.

So sánh giữa các bước "chạm" cũng không cho thấy tương quan nhất quán: bước d (uốn dây) cho tốc độ
cao nhất (113/20s), nhưng bước f (ấn chân STM32) lại thấp hơn cả baseline-tăng-tốc ở bước h
(47/20s so với ~85/20s trung bình của h, và h kết thúc ở ~130/20s quy đổi). Nếu lỗi thực sự do một
điểm tiếp xúc chập chờn phản ứng với thao tác tay, ta sẽ kỳ vọng thấy đột biến rõ rệt đúng lúc chạm
vào MỘT bước cụ thể rồi tốc độ giảm hẳn khi buông tay (bước h) — không phải một xu hướng tăng dần
xuyên suốt toàn bộ 180s bất kể có chạm hay không.

**Diễn giải lại**: bằng chứng "cơn bão lỗi xảy ra trước khi module CAN nạp" ở
[`../regression-after-reboot/README.md`](../regression-after-reboot/README.md) mục 2.3 vẫn đúng —
đó là bằng chứng độc lập kernel-level, không phụ thuộc phép đo này. Nhưng kết luận "nguyên nhân là
tiếp xúc cơ khí chập chờn phản ứng với rung động/chạm" (dựa trên bằng chứng gián tiếp: lỗi xảy ra
theo cụm xen kẽ khoảng nghỉ) **chưa được xác nhận trực tiếp**, và bài wiggle test này — công cụ được
thiết kế RIÊNG để xác nhận trực tiếp giả thuyết đó — **không** cho thấy tương quan chạm-tay/tốc độ
lỗi. Dữ liệu phù hợp hơn với một nguồn lỗi **liên tục, không phụ thuộc tiếp xúc cơ học**, có xu hướng
XẤU DẦN theo thời gian trong phiên đo (gợi ý trôi nhiệt/trôi xung nhịp, hoặc một vấn đề tầng vật lý
thường trực như phản xạ tín hiệu do thiếu/lệch điện trở termination — xem khuyến nghị bên dưới —
hơn là một mối hàn/đầu nối lỏng lẻo ngắt-nối theo rung động).

Lưu ý quan trọng về giới hạn phép đo này: `tx` giữ nguyên 0 suốt 180s (app không chạy, Jetson không
phát gì) — đây là lỗi CHIỀU NHẬN THUẦN TUÝ (STM32 → Jetson) trong điều kiện rảnh. Log
`app.log` của `soak-after-reconnect/` (lúc app đang chạy, Jetson có phát) lại cho thấy chính TEC
(chiều Jetson phát) leo lên 96-135 cùng lúc — tức là **cả hai chiều đều từng lỗi tuỳ thời điểm**, một
đặc điểm khớp với vấn đề tầng vật lý ảnh hưởng cả bus (termination/phản xạ/nhiễu chung), không khớp
với một lỗi cấu hình/logic chỉ ảnh hưởng một chiều cố định.

### Khuyến nghị bước tiếp theo (chưa làm được trong phiên này, cần đo bằng dụng cụ)

1. **Kiểm tra điện trở termination 120Ω ở cả hai đầu bus** (đo bằng đồng hồ khi bus đã tắt nguồn,
   giữa CANH-CANL: nên đọc ~60Ω nếu có đúng 2 điện trở 120Ω song song ở hai đầu; đọc ~120Ω nghĩa là
   thiếu 1 đầu, đọc hở/rất cao nghĩa là thiếu cả hai). **Chưa từng được kiểm tra trong toàn bộ quá
   trình điều tra tới nay** — README chính (`../README.md`) không có mục nào về termination.
2. **Soi dạng sóng CANH/CANL bằng oscilloscope** trong lúc bus có tải thật — tìm dấu hiệu phản xạ
   (ringing) ở cạnh bit, đặc biệt nếu thiếu termination ở mục 1. Đối chiếu biên độ chênh lệch
   CANH-CANL lúc dominant (nên ~2V, đã đo bằng đồng hồ ở mục 2.5 README chính nhưng đó là DC lúc
   nghỉ, chưa đo AC/transient lúc có tải).
3. **Kiểm tra dây CAN có phải cặp xoắn đôi (twisted pair) thật hay không**, và chiều dài/cách đi dây
   so với dây động lực servo (PWM) — nếu CANH/CANL đi chung máng với dây servo hoặc không xoắn, ghép
   nhiễu điện dung/cảm ứng từ PWM là một nguồn lỗi liên tục hợp lý (không cần chạm tay để kích hoạt).
4. **Đo lại có tương quan nhiệt độ hay không**: chạy wiggle test kiểu này (không chạm, chỉ log) trong
   30-60 phút liên tục ngay sau khi bật máy, xem tốc độ error_pass có tiếp tục tăng tốc theo thời gian
   (ủng hộ giả thuyết trôi nhiệt/xung nhịp) hay ổn định lại (ủng hộ giả thuyết khác).
5. **Kiểm tra cách ly GND giữa driver servo và mạch CAN** — đo điện áp GND-GND giữa STM32 và Jetson
   khi servo đang hoạt động (không chỉ lúc rảnh như bài đo mục 2.5 README chính) để phát hiện chênh
   lệch common-mode do dòng điện servo gây ra.

**CẬP NHẬT 2026-09-04, ĐÃ GIẢI QUYẾT — 5 mục trên không cần làm nữa**: nguyên nhân gốc thật là SJW
phía Jetson quá hẹp (12 tq/6% so với STM32 2 tq/12.5%, xác nhận qua SWD), không phải bất kỳ vấn đề
tầng vật lý nào ở trên. Đặt `sjw 16` (`scripts/can_up.sh`/`can0.service`) làm lỗi biến mất hoàn toàn
qua soak 300s xác minh (`../sjw-fix-verification/`). Xem `../README.md` mục "TRẠNG THÁI HIỆN TẠI" để
biết chuỗi loại trừ đầy đủ. Giữ nguyên các mục 1-5 ở trên làm hồ sơ lịch sử — đó là hướng suy luận hợp
lý tại thời điểm viết (ngay sau khi wiggle test loại trừ tiếp xúc vật lý, trước khi có dữ liệu SWD).
