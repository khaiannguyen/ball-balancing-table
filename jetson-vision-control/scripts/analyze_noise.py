import csv, statistics, sys

path = sys.argv[1] if len(sys.argv) > 1 else "/home/khaian/balance_ball/scripts/data.csv"

xs, ys = [], []
with open(path) as f:
    r = csv.DictReader(f)
    for row in r:
        if int(row["detected"]) == 1:
            xs.append(float(row["Ballx"]))
            ys.append(float(row["Bally"]))

if len(xs) < 5:
    print("Khong du du lieu detected=1 de phan tich - kiem tra lai file CSV")
    sys.exit(1)

print(f"So mau: {len(xs)}")
print(f"Ballx: mean={statistics.mean(xs):.2f}  stdev={statistics.stdev(xs):.2f}  "
      f"min={min(xs):.1f}  max={max(xs):.1f}  range={max(xs)-min(xs):.1f}")
print(f"Bally: mean={statistics.mean(ys):.2f}  stdev={statistics.stdev(ys):.2f}  "
      f"min={min(ys):.1f}  max={max(ys):.1f}  range={max(ys)-min(ys):.1f}")

# Goi y alpha dua tren stdev: neu stdev lon (>3-5mm), can loc manh hon (alpha nho)
sx, sy = statistics.stdev(xs), statistics.stdev(ys)
suggested = 0.5 if max(sx,sy) < 2 else (0.35 if max(sx,sy) < 5 else 0.2)
print(f"\nGoi y alpha khoi diem: {suggested} (dua tren stdev cao nhat = {max(sx,sy):.2f}mm)")