import struct
import statistics
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "wcet_raw.bin"

with open(filename, "rb") as f:
    data = f.read()

n_words = len(data) // 4
cycles = struct.unpack(f"<{n_words}I", data)
us = [c / 400.0 for c in cycles if c != 0]

budget_us = 10000.0
mean_us = statistics.mean(us)
max_us = max(us)

print("File:", filename)
print("N valid patterns:", len(us))
print("mean (us):", mean_us, f"({mean_us/budget_us*100:.2f}% of 10ms budget)")
print("max  (us):", max_us,  f"({max_us/budget_us*100:.2f}% of 10ms budget)  <-- WCET")
print("min  (us):", min(us))
print("std  (us):", statistics.pstdev(us))

out_csv = filename.replace(".bin", "_summary.csv")
with open(out_csv, "w") as f:
    f.write("sample_index,exec_us\n")
    for i, v in enumerate(us):
        f.write(f"{i},{v:.3f}\n")
print("Written", out_csv)