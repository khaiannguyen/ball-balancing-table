# Wiggle test — locating the intermittent contact point on the CAN wire

Context: the SN65HVD230 transceiver module on the Jetson side was replaced, the symptom
did NOT change — still `TEC=0` (or low), `REC` climbing high (seen up to 117-127),
`TX packets=0` when the app is not running but still receiving continuous error frames
from STM32. This rules out the Jetson-side transceiver chip itself as the cause (see
the investigation history in [`../README.md`](../README.md),
[`../regression-after-reboot/README.md`](../regression-after-reboot/README.md)).

Goal of this test: correlate **the moment of hand manipulation** (wiggling/pressing
each physical contact point) with **the moment berr-counter spikes**, to pinpoint the
exact intermittent point — or rule out the possibility of a contact fault entirely.

## Tool: `scripts/wiggle_monitor.sh`

Deliberately **does not use candump, does not run the app** — only reads
`ip -details -s link show can0` every 0.5s, to isolate as much as possible from
software and leave only the physical/driver layer.

Reset `can0` with `modprobe -r mttcan && modprobe mttcan` (a real driver reload) —
**not** `ip link down/up` or `systemctl restart can0.service`, since it was confirmed
in [`../regression-after-reboot/README.md`](../regression-after-reboot/README.md)
(and recorded in `scripts/run_soak.sh`) that those two methods do **not** reset the
berr-counter (TEC/REC) or the error-warn/error-pass/bus-off counters on this `mttcan`
driver.

The script automatically prints a step-by-step instruction "banner" to the screen at
the right moment AND writes that banner (with an epoch timestamp) to `wiggle_raw.log`
— so the timing reference used at the analysis step comes directly from the log, not
dependent on the operator calling out loud/remembering the exact time.

### How to run

Run directly in your own terminal (not through the agent) so you can see real-time
output while your hand is doing the manipulation:

```
sudo scripts/wiggle_monitor.sh
```

Default: outdir `wiggle_run_<timestamp>/`, duration 180s (3 minutes). Can be specified
otherwise: `sudo scripts/wiggle_monitor.sh <outdir> <duration_s>`.

### Manipulation schedule (automatically shown on screen, ~20s/step)

| Step | Time (seconds) | Action |
|---|---|---|
| a | 0-20 | **Baseline** — touch nothing |
| b | 20-40 | Gently wiggle the CAN connector end on the **Jetson** side (new module) |
| c | 40-60 | Gently wiggle the CAN connector end on the **STM32** side |
| d | 60-80 | Gently wiggle/flex **along the CANH/CANL wire** between the two boards |
| e | 80-100 | Gently press **each header pin** of the Jetson module (without removing it) |
| f | 100-120 | Gently press **each header pin** of the STM32 module |
| g | 120-140 | Check the **common GND** connection point between the two boards (gently wiggle) |
| h | 140-180 | **Hands off completely** — observe whether the error keeps increasing with no touching |

Step h (hands off, not in the original request but added because it is a mandatory
counter-test: if the error still increases steadily during these final 40s even though
nobody is touching anything, that is direct evidence that the cause is NOT mechanical
contact).

### Output of each run

- `run_info.txt` — the `can0` state right after reset (must be
  `ERROR-ACTIVE, berr tx=0 rx=0` if the reset succeeded) + the step schedule.
- `wiggle_raw.log` — the full raw dump of `ip -details -s link show can0` every 0.5s
  with an epoch timestamp, and a banner at every step transition.
- `wiggle_berr.csv` — a shortened CSV form per sample: step, epoch, elapsed_s,
  can_state, berr_tx, berr_rx, rx_packets, rx_errors, rx_dropped, tx_packets, restarts,
  bus_errors, arbit_lost, error_warn, error_pass, bus_off — used to compute the delta
  between steps faster than parsing the raw log.

## Result

Run 1 time: [`wiggle_run_20260904_072240/`](wiggle_run_20260904_072240/) (2026-09-04
07:22, 180s). `run_info.txt` confirms a clean reset before starting
(`ERROR-ACTIVE, berr tx=0 rx=0`, correct 125kbps/sample-point 0.875 as in `can_up.sh`).

### Number of `entered error passive` events (the `error_pass` column, cumulative) per step, 20s/step

| Step | Action | Δ error_pass in the step | Converted to /20s |
|---|---|---|---|
| a | Baseline, no touch | 0 | 0 |
| b | Wiggle Jetson connector | 21 | 21 |
| c | Wiggle STM32 connector | 50 | 50 |
| d | Wiggle/flex along the CANH/CANL wire | **113** | **113** (highest) |
| e | Press Jetson header pins | 93 | 93 |
| f | Press STM32 header pins | 47 | 47 |
| g | Wiggle the common GND connection point | 78 | 78 |
| h | **Hands off completely** (40s) | 171 | ~85 |

During step h, the rate of increase kept **accelerating over time** even though nobody
was touching anything: `+34` (140-150s), `+36` (150-161s), `+46` (161-172s,
converted/10s), `+65` (172-180s, converted/10s) — faster near the end of the step
rather than slowing/stopping.

### Interpretation — **CONCLUSION OPPOSITE to the `regression-after-reboot/` report**

Step h was designed as a mandatory counter-test (see the test-design section above):
*"if the error still increases steadily during these final 40s even though nobody is
touching anything, that is direct evidence that the cause is NOT mechanical contact."*
That is exactly what happened — not only a steady increase, but even **acceleration**
while not touching any contact point at all.

Comparing the "touching" steps also shows no consistent correlation: step d (flexing
the wire) gives the highest rate (113/20s), but step f (pressing STM32 pins) is even
lower than the accelerating baseline in step h (47/20s vs. the ~85/20s average of h,
and h ends at ~130/20s converted). If the error really were caused by an intermittent
contact point reacting to hand manipulation, we would expect to see a clear spike
exactly when touching ONE specific step, then the rate to drop sharply once hands are
off (step h) — not a trend of steady increase throughout the entire 180s regardless of
touching.

**Reinterpretation**: the "error storm occurred before the CAN module loaded" evidence
in [`../regression-after-reboot/README.md`](../regression-after-reboot/README.md)
section 2.3 is still correct — that is independent kernel-level evidence, not
dependent on this measurement. But the conclusion that "the cause is an intermittent
mechanical contact reacting to vibration/touch" (based on indirect evidence: errors
occurring in bursts interspersed with quiet periods) **has not been confirmed
directly**, and this wiggle test — the tool designed SPECIFICALLY to directly confirm
that hypothesis — does **not** show a hand-touch/error-rate correlation. The data fits
better with a **continuous** error source, **not dependent on mechanical contact**,
that tends to get WORSE over time within the measurement session (suggesting thermal
drift/clock drift, or a persistent physical-layer issue such as signal reflection from
missing/mismatched termination resistance — see the recommendations below — rather
than a loose solder joint/connector that connects-disconnects with vibration).

An important note on the limitation of this measurement: `tx` stays at 0 throughout
the 180s (the app is not running, Jetson is not transmitting anything) — this is a
PURE RECEIVE-DIRECTION (STM32 → Jetson) fault under idle conditions. The `app.log` log
from `soak-after-reconnect/` (with the app running, Jetson transmitting) instead shows
TEC itself (the Jetson-transmit direction) climbing to 96-135 at the same time —
meaning **both directions have errored at various times**, a characteristic that
matches a physical-layer issue affecting the whole bus (termination/reflection/common
noise), not matching a configuration/logic fault that only affects one fixed
direction.

### Recommended next steps (not done in this session, requires instrumented measurement)

1. **Check the 120Ω termination resistance at both ends of the bus** (measure with a
   multimeter while the bus is powered off, between CANH-CANL: should read ~60Ω if
   there are exactly 2× 120Ω resistors in parallel at the two ends; reading ~120Ω
   means 1 end is missing, reading open/very high means both are missing). **Never
   checked in the entire investigation to date** — the main README (`../README.md`)
   has no section on termination.
2. **Observe the CANH/CANL waveform with an oscilloscope** while the bus has a real
   load — look for signs of reflection (ringing) at bit edges, especially if
   termination is missing per item 1. Cross-check the CANH-CANL voltage differential
   at dominant (should be ~2V, already measured with a multimeter in section 2.5 of
   the main README but that was DC at idle, AC/transient under load not yet measured).
3. **Check whether the CAN wire is actually a real twisted pair**, and its
   length/routing relative to the servo power (PWM) wires — if CANH/CANL run in the
   same channel as the servo wires or are not twisted, capacitive/inductive coupling
   from the PWM is a plausible continuous error source (does not need hand-touching to
   trigger).
4. **Re-measure for a temperature correlation**: run this kind of wiggle test (no
   touching, just logging) continuously for 30-60 minutes right after power-on, and
   see whether the error_pass rate keeps accelerating over time (supporting the
   thermal-drift/clock-drift hypothesis) or settles down (supporting a different
   hypothesis).
5. **Check the GND isolation between the servo driver and the CAN circuit** — measure
   the GND-GND voltage between STM32 and Jetson while the servos are actively running
   (not just at idle as in the section 2.5 measurement in the main README) to detect
   common-mode offset caused by servo current.

**UPDATE 2026-09-04, ROOT-CAUSE ATTRIBUTION SUPERSEDED — see
[`../README.md`](../README.md) for the current, final root cause**: this note
originally attributed the fix to the Jetson-side SJW being too narrow (12 tq/6% vs.
STM32's 2 tq/12.5%, confirmed via SWD), with a 300s soak run at `sjw 16`
(`scripts/can_up.sh`/`can0.service`) cited as confirmation
(`../sjw-fix-verification/`). That attribution is superseded: later isolation testing
(holding `sjw=16` fixed across both trials) found that `sjw=16` alone did not stop the
fault while the CAN_H/CAN_L wire pair was still untwisted, and that twisting the wire
pair was the change that actually eliminated it. The current root cause is the
untwisted CAN_H/CAN_L wire pair (cross-coupled noise/EMI) — item 3 above, checking
whether the wire is a real twisted pair, turned out to be the one that mattered.
`sjw=16` is kept as an additional safety margin; its own individual contribution has
not been verified in isolation. Items 1, 2, 4 and 5 above are kept as historical
record.
