#!/bin/bash
# scripts/run_soak.sh - reusable CAN soak-test capture: candump + canstats
# (every 5s) + app.log + STM32 UART, all timestamped to a common wall-clock
# so validation/03-can/README.md-style analysis can correlate them.
#
# Usage: scripts/run_soak.sh <duration_seconds> <output_dir> [uart_dev] [uart_baud]
#
# Resets can0 to a known-clean state (systemctl restart can0.service) BEFORE
# capturing, and records the resulting `ip -details link show can0` at the
# top of run_info.txt -- several earlier measurements in this investigation
# gave confusing results because can0 was already ERROR-PASSIVE from a prior
# session when the capture started.
set -u

DURATION="${1:?Usage: run_soak.sh <duration_seconds> <output_dir> [uart_dev] [uart_baud]}"
OUTDIR="${2:?Usage: run_soak.sh <duration_seconds> <output_dir> [uart_dev] [uart_baud]}"
UART_DEV="${3:-/dev/ttyACM0}"
UART_BAUD="${4:-115200}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ ! -x "./build/balance_ball_main" ]; then
    echo "run_soak.sh: ./build/balance_ball_main khong ton tai hoac khong chay duoc - build truoc." >&2
    exit 1
fi

mkdir -p "$OUTDIR"

CANSTATS_PID=""

cleanup() {
    echo "--- cleanup: stopping background processes ---"
    sudo pkill -f "candump -tz can0" 2>/dev/null
    sudo pkill -f "cat $UART_DEV" 2>/dev/null
    sudo pkill -INT -f "build/balance_ball_main" 2>/dev/null
    sleep 1
    sudo pkill -f "build/balance_ball_main" 2>/dev/null
    sudo pkill -f "script -qc" 2>/dev/null
    [ -n "$CANSTATS_PID" ] && kill "$CANSTATS_PID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM ERR

echo "=== resetting can0 to a clean state (systemctl restart can0.service) ==="
sudo systemctl restart can0.service
sleep 1

{
  echo "SOAK_START=$(date +%s.%N) ($(date '+%Y-%m-%d %H:%M:%S %Z'))"
  echo "duration=${DURATION}s outdir=$OUTDIR uart=$UART_DEV@$UART_BAUD"
  echo "can0 state at start (after can0.service restart):"
  ip -details link show can0
} | tee "$OUTDIR/run_info.txt"

sudo stty -F "$UART_DEV" "$UART_BAUD" raw -echo -echoe -echok

sudo candump -tz can0 > "$OUTDIR/candump.log" &

# mawk's fflush() does not reliably flush stdout on this system when stdout
# is a pipe/file (verified during the busoff-investigation session) - use
# `-W interactive` instead, which forces unbuffered line-at-a-time output.
( while true; do date +%s.%N; ip -details -s link show can0; sleep 5; done ) > "$OUTDIR/canstats.log" &
CANSTATS_PID=$!

sudo cat "$UART_DEV" | awk -W interactive '{print systime()" "$0}' > "$OUTDIR/stm32_uart.log" &

# `script` allocates a pty for balance_ball_main's stdout so it is
# line-buffered instead of fully-buffered-until-exit (stdbuf's LD_PRELOAD
# trick does not survive the sudo exec boundary reliably on this system).
sudo script -qc "./build/balance_ball_main" /dev/null 2>&1 | awk -W interactive '{print systime()" "$0}' > "$OUTDIR/app.log" &

echo "Running for ${DURATION}s..."
sleep "$DURATION"

echo "SOAK_END=$(date +%s.%N) ($(date '+%Y-%m-%d %H:%M:%S %Z'))" | tee -a "$OUTDIR/run_info.txt"
