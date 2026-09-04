# CẢNH BÁO: source trong thư mục này KHÔNG khớp firmware đang chạy trên chip thật

Viết 2026-09-04, sau khi điều tra lỗi CAN bus lặp lại giữa STM32H723 và Jetson
(toàn bộ quá trình: `../docs/` và `validation/03-can/` phía repo `balance_ball`).

## Sự thật đã xác nhận bằng SWD (đọc thanh ghi sống qua ST-Link, không build/flash)

Chip STM32H723 đang chạy thật trên board Nucleo-H723ZG (đã xác nhận qua ổ MSC
`NODE_H723ZG` của board) có:

```
FDCAN1->NBTP: NominalPrescaler=40, NominalTimeSeg1=13, NominalTimeSeg2=2  -> 125000 bps, sample-point 0.875
FDCAN1->CCCR: DAR=1 -> AutoRetransmission = DISABLE
```

**Source trong `Core/Src/main.c` ở thư mục này (và trên GitHub
`khaiannguyen/ball-balancing-table`, branch `main`, mọi commit/branch/stash đã
kiểm tra) lại ghi:**

```c
hfdcan1.Init.NominalPrescaler = 10;   // -> 500000 bps nếu build/flash từ đây, KHÔNG PHẢI 125000
hfdcan1.Init.AutoRetransmission = ENABLE;   // KHÁC DAR=1 (DISABLE) đang chạy thật
```

Với `NominalPrescaler=10`, `NominalTimeSeg1=13`, `NominalTimeSeg2=2` và
FDCAN kernel clock 80MHz (đã đo qua SWD, không giả định): bitrate =
80e6/(10×16) = **500000 bps** — lệch 4 lần so với Jetson (`can0` chạy
125000 bps, xem `../jetson-vision-control/scripts/can_up.sh`).

## Vì sao điều này nguy hiểm

Nếu ai đó build + flash STM32H723 từ chính thư mục này (hoặc từ bản GitHub
đã push, hiện đang giống hệt), board sẽ chạy sai bitrate và tái tạo lại
đúng loại lỗi CAN đã mất nhiều ngày điều tra (xem `validation/03-can/README.md`
phía `balance_ball` để biết toàn bộ diễn biến — nguyên nhân thật của lỗi hóa ra
là SJW phía Jetson quá hẹp, KHÔNG phải bitrate, nhưng nếu flash nhầm bản này thì
sẽ có thêm một lỗi bitrate thật chồng lên).

## Bản nào mới đúng?

Bản đang chạy thật trên chip (`NominalPrescaler=40`, `AutoRetransmission=DISABLE`)
**không tìm thấy ở bất kỳ đâu trên máy Jetson này** — không phải file, không
phải git object (đã kiểm tra toàn bộ commit/branch/stash/reflog), không phải
build artifact (`.elf`/`.hex`). Nó gần như chắc chắn chỉ tồn tại trên máy đã
dùng STM32CubeIDE để build + nạp lần cuối. **Trước khi build/flash lại từ thư
mục này, hãy tìm và đối chiếu với project trên máy đó trước.**

## Việc cần làm (chưa làm ở đây — chỉ ghi chú, không tự sửa `main.c`)

1. Tìm project STM32CubeIDE thật (máy đã flash lần cuối), lấy đúng
   `NominalPrescaler`/`AutoRetransmission` từ đó.
2. Cập nhật `Core/Src/main.c` ở đây cho khớp, hoặc — tốt hơn — build lại đúng
   từ máy CubeIDE đó, đọc lại qua SWD một lần nữa để xác nhận khớp, rồi mới
   đồng bộ source vào git.
3. Commit + push, xoá file cảnh báo này sau khi source đã khớp.
