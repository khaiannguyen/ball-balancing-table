"""
Analyze ball detection noise from a CSV dataset.

Only samples with detected=1 are included in the analysis.
The script reports the mean, standard deviation, range, and a suggested
starting value for the ball-position filter alpha.
"""

import csv
import statistics
import sys


path = (
    sys.argv[1]
    if len(sys.argv) > 1
    else "/home/khaian/balance_ball/scripts/data.csv"
)

xs = []
ys = []


with open(path) as f:
    reader = csv.DictReader(f)

    for row in reader:
        if int(row["detected"]) == 1:
            xs.append(float(row["Ballx"]))
            ys.append(float(row["Bally"]))


if len(xs) < 5:
    print(
        "Not enough detected=1 samples for analysis. "
        "Check the CSV file and detection results."
    )

    sys.exit(1)


print(
    f"Sample count: {len(xs)}"
)

print(
    f"Ballx: mean={statistics.mean(xs):.2f} "
    f"stdev={statistics.stdev(xs):.2f} "
    f"min={min(xs):.1f} "
    f"max={max(xs):.1f} "
    f"range={max(xs) - min(xs):.1f}"
)

print(
    f"Bally: mean={statistics.mean(ys):.2f} "
    f"stdev={statistics.stdev(ys):.2f} "
    f"min={min(ys):.1f} "
    f"max={max(ys):.1f} "
    f"range={max(ys) - min(ys):.1f}"
)


# A higher standard deviation indicates stronger measurement noise and
# therefore requires stronger filtering, which corresponds to a lower alpha.
sx = statistics.stdev(xs)
sy = statistics.stdev(ys)

max_stddev = max(sx, sy)

suggested = (
    0.5
    if max_stddev < 2
    else (
        0.35
        if max_stddev < 5
        else 0.2
    )
)


print(
    f"\nSuggested starting alpha: {suggested} "
    f"(based on maximum standard deviation = {max_stddev:.2f} mm)"
)