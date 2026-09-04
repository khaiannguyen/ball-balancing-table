# Xác minh fix SJW=16 — soak 300s sau khi persist fix

Ngày đo: 2026-09-04, ngay sau khi persist `sjw 16` vào `scripts/can0.service`
(live: `/etc/systemd/system/can0.service`) + `scripts/can_up.sh`. Mục đích: xác
nhận BẰNG SỐ rằng nguyên nhân gốc thật sự là SJW (không phải cảm giác/một lần
thử ngắn), với thời lượng dài hơn hẳn baseline lỗi cũ để không bị "lọt" hiện
tượng lỗi tăng dần theo thời gian đã thấy trong wiggle test.

## Baseline lỗi CŨ (trước fix) — tham chiếu

Từ [`../physical-fault-localization/README.md`](../physical-fault-localization/README.md),
bước h (buông tay hoàn toàn, 40s, KHÔNG chạm gì) của wiggle test:

| Cửa sổ | error_pass tăng thêm |
|---|---|
| 140-150s | +34 |
| 150-161s | +36 |
| 161-172s | +46 (quy đổi/10s) |
| 172-180s | +65 (quy đổi/10s) |

Tốc độ lỗi **tăng dần** ngay cả khi không ai chạm vào bus — đây chính là baseline
"xấu" cần so sánh.

## Đo lại SAU khi persist sjw=16

Phương pháp: reset sạch (`modprobe -r mttcan && modprobe mttcan`, xác nhận
`berr tx=0 rx=0` trước khi đo — same methodology `wiggle_monitor.sh`/
`run_soak.sh`), `systemctl restart can0.service` (dùng đúng unit đã persist
fix), sau đó lấy mẫu `ip -details -statistics link show can0` mỗi 0.5s trong
300s liên tục — **dài hơn baseline cũ (180s wiggle test / 40s cửa sổ tham
chiếu)**, không chạm gì, không chạy app (methodology giống hệt wiggle test,
chỉ khác là không có thao tác tay vì không cần nữa).

Log thô: [`soak_berr.csv`](soak_berr.csv) (555 dòng, 299.7s).

| Cửa sổ (30 cửa sổ × 10s) | error_pass tăng thêm |
|---|---|
| 0-300s (toàn bộ 30 cửa sổ) | **0 ở TẤT CẢ 30 cửa sổ** |

Toàn bộ 555 mẫu: `can_state` luôn là `ERROR-ACTIVE` (không một lần nào rơi vào
`ERROR-WARNING`/`ERROR-PASSIVE`/`BUS-OFF`), `berr-counter tx=0 rx=0` không đổi
suốt 300s, `error-warn`/`error-pass`/`bus-off` cộng dồn đều = 0 không tăng.

## Kết luận

| | Baseline cũ (40s, không chạm) | Sau fix sjw=16 (300s, không chạm) |
|---|---|---|
| error_pass/10s | 34, 36, 46, 65 (tăng dần) | **0, 0, 0, ..., 0 (30/30 cửa sổ)** |
| can_state | ERROR-WARNING/PASSIVE dao động | ERROR-ACTIVE cố định |
| berr tx/rx | rx dao động 100-127 | **0/0 không đổi** |

Lỗi biến mất hoàn toàn, không phải giảm bớt — 300 giây liên tục (gấp ~7.5 lần
cửa sổ 40s tạo ra baseline 34-65 lỗi/10s) không có một sự kiện lỗi nào. Xác
nhận SJW là nguyên nhân gốc, không phải trùng hợp của một lần thử ngắn.
