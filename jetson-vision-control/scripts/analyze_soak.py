#!/usr/bin/env python3
"""scripts/analyze_soak.py - analyze one CAN soak-test capture directory.

Reads candump.log / canstats.log / app.log / stm32_uart.log (as written by
scripts/run_soak.sh) from a soak directory and prints a report: FAULT
count/duration, [CAN_TX] delta stats (deltas between consecutive samples,
NOT absolute counter values - the STM32 counters are cumulative since
boot), berr-counter/state timeline, realized bus load, per-ID frame
distribution vs the designed rate table, candump gaps >100ms, and any
[BOOT] (STM32 MCU reset) events found in the UART log.

Usage: scripts/analyze_soak.py <soak_dir>
"""
import argparse
import re
import statistics
import sys
from pathlib import Path

# Designed frame rates (Hz) - matches validation/03-can/README.md section 4.
DESIGN_HZ = {
    "100": 100, "102": 100,
    "101": 20, "103": 20, "104": 20, "1FF": 20,
    "200": 100, "204": 100,
    "201": 50,
    "202": 20, "2FF": 20,
}

BUS_LOAD_SEC_PER_FRAME = 1080e-6  # seconds/frame at 125kbps, per validation/03-can/README.md


def read_lines(path):
    if not path.exists():
        return []
    return path.read_text(errors="replace").splitlines()


# ---------- app.log: FAULT/CLEARED ----------

FAULT_RE = re.compile(r"^(\d+) .*TaskCanRx: FAULT -")
CLEARED_RE = re.compile(r"^(\d+) .*TaskCanRx: FAULT CLEARED")


def analyze_app_log(lines):
    events = []
    for l in lines:
        m = FAULT_RE.match(l)
        if m:
            events.append(("FAULT", int(m.group(1))))
            continue
        m = CLEARED_RE.match(l)
        if m:
            events.append(("CLEARED", int(m.group(1))))

    stable_durations = []
    down_durations = []
    prev_cleared = None
    last_fault = None
    for kind, t in events:
        if kind == "FAULT":
            if prev_cleared is not None:
                stable_durations.append(t - prev_cleared)
            last_fault = t
        else:  # CLEARED
            if last_fault is not None:
                down_durations.append(t - last_fault)
            prev_cleared = t

    fault_count = sum(1 for k, _ in events if k == "FAULT")
    cleared_count = sum(1 for k, _ in events if k == "CLEARED")

    return {
        "fault_count": fault_count,
        "cleared_count": cleared_count,
        "stable_avg": statistics.mean(stable_durations) if stable_durations else None,
        "stable_max": max(stable_durations) if stable_durations else None,
        "down_avg": statistics.mean(down_durations) if down_durations else None,
        "down_max": max(down_durations) if down_durations else None,
        "fault_timestamps": [t for k, t in events if k == "FAULT"],
    }


# ---------- stm32_uart.log: [CAN_TX] deltas + [BOOT] ----------

CANTX_RE = re.compile(
    r"^(\d+) \[CAN_TX\] attempt=(\d+) drop=(\d+) burst=(\d+) busoff=(\d+) "
    r"TEC=(\d+) REC=(\d+) passive=(\d+) warn=(\d+) LEC=(\d+) free=(\d+)"
)
BOOT_RE = re.compile(r"^(\d+) .*\[BOOT\](.*)")
RSR_HINT_RE = re.compile(r"RSR", re.IGNORECASE)


def analyze_uart_log(lines):
    samples = []
    for l in lines:
        m = CANTX_RE.match(l)
        if m:
            samples.append({
                "t": int(m.group(1)), "attempt": int(m.group(2)), "drop": int(m.group(3)),
                "burst": int(m.group(4)), "busoff": int(m.group(5)),
                "TEC": int(m.group(6)), "REC": int(m.group(7)),
                "passive": int(m.group(8)), "warn": int(m.group(9)),
                "LEC": int(m.group(10)), "free": int(m.group(11)),
            })

    boots = []
    other_rsr_lines = []
    for l in lines:
        m = BOOT_RE.match(l)
        if m:
            boots.append((int(m.group(1)), m.group(2).strip()))
        elif RSR_HINT_RE.search(l):
            other_rsr_lines.append(l)

    # deltas between consecutive [CAN_TX] samples, EXCLUDING any pair that
    # straddles a counter reset (delta < 0 => a reboot happened between them,
    # not a real drop in traffic - counted separately, not folded into stats)
    deltas = []
    counter_resets_detected = 0
    for a, b in zip(samples, samples[1:]):
        d_attempt = b["attempt"] - a["attempt"]
        d_drop = b["drop"] - a["drop"]
        d_busoff = b["busoff"] - a["busoff"]
        if d_attempt < 0 or d_drop < 0 or d_busoff < 0:
            counter_resets_detected += 1
            continue
        deltas.append({"t": b["t"], "d_attempt": d_attempt, "d_drop": d_drop, "d_busoff": d_busoff})

    return {
        "samples": samples, "boots": boots, "other_rsr_lines": other_rsr_lines,
        "deltas": deltas, "counter_resets_detected": counter_resets_detected,
    }


# ---------- canstats.log: berr-counter / state timeline ----------

STATE_RE = re.compile(r"can state (\S+) \(berr-counter tx (\d+) rx (\d+)\)")
TS_RE = re.compile(r"^(\d+\.\d+)$")


def analyze_canstats(lines):
    samples = []
    ts = None
    for l in lines:
        m = TS_RE.match(l.strip())
        if m:
            ts = float(m.group(1))
            continue
        m = STATE_RE.search(l)
        if m and ts is not None:
            samples.append({"t": ts, "state": m.group(1), "berr_tx": int(m.group(2)), "berr_rx": int(m.group(3))})
            ts = None

    non_active = [s for s in samples if s["state"] != "ERROR-ACTIVE"]
    return {"samples": samples, "non_active": non_active}


# ---------- candump.log: ID distribution, gaps, bus load ----------

CANDUMP_RE = re.compile(r"^\s*\((\d+\.\d+)\)\s+\S+\s+(\S+)\s+\[")


def analyze_candump(lines):
    frames = []
    for l in lines:
        m = CANDUMP_RE.match(l)
        if m:
            frames.append((float(m.group(1)), m.group(2)))

    id_counts = {}
    for _, cid in frames:
        id_counts[cid] = id_counts.get(cid, 0) + 1

    gaps = []
    for (t0, _), (t1, _) in zip(frames, frames[1:]):
        d = t1 - t0
        if d > 0.1:
            gaps.append((t0, t1, d))

    duration = (frames[-1][0] - frames[0][0]) if frames else 0.0
    total = len(frames)
    rate = total / duration if duration > 0 else 0.0
    bus_load_pct = rate * BUS_LOAD_SEC_PER_FRAME * 100

    return {
        "total_frames": total, "duration": duration, "rate": rate,
        "bus_load_pct": bus_load_pct, "id_counts": id_counts, "gaps": gaps,
    }


def fmt_s(x):
    return f"{x:.2f}s" if x is not None else "n/a"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("soak_dir", type=Path, help="thu muc chua candump.log/canstats.log/app.log/stm32_uart.log")
    args = ap.parse_args()

    d = args.soak_dir
    app_lines = read_lines(d / "app.log")
    uart_lines = read_lines(d / "stm32_uart.log")
    canstats_lines = read_lines(d / "canstats.log")
    candump_lines = read_lines(d / "candump.log")

    if not (app_lines or uart_lines or canstats_lines or candump_lines):
        print(f"analyze_soak.py: khong doc duoc file log nao trong {d}", file=sys.stderr)
        sys.exit(1)

    app = analyze_app_log(app_lines)
    uart = analyze_uart_log(uart_lines)
    canstats = analyze_canstats(canstats_lines)
    candump = analyze_candump(candump_lines)

    print(f"=== Soak analysis: {d} ===\n")

    print("-- stm32_ok (app.log) --")
    print(f"FAULT count: {app['fault_count']}  CLEARED count: {app['cleared_count']}")
    print(f"Stable duration: avg={fmt_s(app['stable_avg'])} max={fmt_s(app['stable_max'])}")
    print(f"Down duration:   avg={fmt_s(app['down_avg'])} max={fmt_s(app['down_max'])}")
    if app["fault_timestamps"]:
        print(f"FAULT timestamps (epoch): {app['fault_timestamps']}")
    print()

    print("-- [CAN_TX] deltas (stm32_uart.log), counter resets excluded --")
    if uart["deltas"]:
        d_attempt = [x["d_attempt"] for x in uart["deltas"]]
        d_drop = [x["d_drop"] for x in uart["deltas"]]
        d_busoff = [x["d_busoff"] for x in uart["deltas"]]
        print(f"delta_attempt: sum={sum(d_attempt)} avg={statistics.mean(d_attempt):.1f} max={max(d_attempt)}")
        print(f"delta_drop:    sum={sum(d_drop)} avg={statistics.mean(d_drop):.2f} max={max(d_drop)}")
        print(f"delta_busoff:  sum={sum(d_busoff)} avg={statistics.mean(d_busoff):.3f} max={max(d_busoff)}")
        nonzero_drop = sum(1 for x in d_drop if x > 0)
        nonzero_busoff = sum(1 for x in d_busoff if x > 0)
        print(f"mau co delta_drop>0: {nonzero_drop}/{len(d_drop)}   delta_busoff>0: {nonzero_busoff}/{len(d_busoff)}")
    else:
        print("(khong co du 2 dong [CAN_TX] hop le lien tiep de tinh delta)")
    if uart["counter_resets_detected"]:
        print(f"** {uart['counter_resets_detected']} lan counter GIAM giua 2 dong lien tiep (loai khoi thong ke "
              f"delta o tren vi day la dau hieu reboot, khong phai giam traffic that) - xem [BOOT] ben duoi **")
    print()

    print("-- [BOOT] events (STM32 UART - MCU reset giua chung) --")
    if uart["boots"]:
        for t, info in uart["boots"]:
            print(f"  t={t}: {info if info else '(dong [BOOT] khong co chi tiet RSR)'}")
    else:
        print("(khong phat hien dong [BOOT] nao trong log nay)")
    if uart["other_rsr_lines"]:
        print(f"  Luu y: co {len(uart['other_rsr_lines'])} dong chua chu 'RSR' nhung KHONG khop dinh dang [BOOT] "
              f"da mong doi - kiem tra lai tag/dinh dang firmware dang dung:")
        for l in uart["other_rsr_lines"][:5]:
            print(f"    {l}")
    print()

    print("-- berr-counter / trang thai (canstats.log) --")
    total_samples = len(canstats["samples"])
    print(f"Tong so mau: {total_samples}")
    print(f"Mau khac ERROR-ACTIVE: {len(canstats['non_active'])}/{total_samples}")
    for s in canstats["non_active"][:20]:
        print(f"  t={s['t']:.1f} state={s['state']} berr_tx={s['berr_tx']} berr_rx={s['berr_rx']}")
    if len(canstats["non_active"]) > 20:
        print(f"  ... va {len(canstats['non_active']) - 20} mau khac")
    print()

    print("-- Bus load (candump.log) --")
    print(f"Tong frame: {candump['total_frames']}  thoi luong: {candump['duration']:.1f}s  "
          f"rate: {candump['rate']:.1f} fr/s")
    print(f"Bus load (chi chieu hien trong candump - KHONG gom frame Jetson tu phat, "
          f"vi CAN_RAW_LOOPBACK da tat trong CanTransport::open()): {candump['bus_load_pct']:.2f}%")
    print()

    print("-- Phan bo frame theo ID (candump.log) vs thiet ke --")
    for cid, count in sorted(candump["id_counts"].items(), key=lambda kv: -kv[1]):
        hz = count / candump["duration"] if candump["duration"] > 0 else 0
        design = DESIGN_HZ.get(cid)
        design_s = f"{design}Hz" if design is not None else "?"
        print(f"  0x{cid}: {count} frame ({hz:.1f}Hz thuc te, thiet ke {design_s})")
    print()

    print("-- Gap trong candump (>100ms) --")
    if candump["gaps"]:
        print(f"So gap: {len(candump['gaps'])}")
        worst = sorted(candump["gaps"], key=lambda g: -g[2])[:10]
        for t0, t1, dd in worst:
            print(f"  gap={dd:.3f}s  candump-relative t={t0:.3f} -> {t1:.3f}")
        if len(candump["gaps"]) > 10:
            print(f"  ... va {len(candump['gaps']) - 10} gap khac (da sap xep, chi in 10 gap lon nhat)")
    else:
        print("(khong co gap nao >100ms)")


if __name__ == "__main__":
    main()
