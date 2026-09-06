import struct
import statistics

with open("before_raw.bin", "rb") as f: 
    data = f.read()

print("File size:", len(data), "bytes")

n_words = len(data) // 4
cycles = struct.unpack(f"<{n_words}I", data)

# remove element = 0 (t_prev==0 in the first sample so the first period is not recorded)
us = [c / 400.0 for c in cycles if c != 0]

print("N valid patterns:", len(us))
print("mean (us):", statistics.mean(us))
print("std (us):", statistics.pstdev(us))
print("min (us):", min(us))
print("max (us):", max(us))

with open("before_summary.csv", "w") as f: 
    f.write("sample_index,period_us\n") 
    for i, v in enumerate(us): 
        f.write(f"{i},{v:.3f}\n")

print("Written before_summary.csv")