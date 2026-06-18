#!/usr/bin/env bash
#
# Capture raw USB traffic for a Switch 2 controller with usbmon, while running
# the init handshake. Produces a text capture you can grep/inspect, plus the
# decoded hidraw stream from sw2_capture.py.
#
# Usage:  sudo ./run_usbmon.sh 0x2069 [seconds]
#
set -euo pipefail
PID="${1:?usage: sudo ./run_usbmon.sh <pid hex> [seconds]}"
SECS="${2:-30}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TS="$(date +%s)"

if [[ "$(id -u)" -ne 0 ]]; then
	echo "Run as root (sudo)." >&2
	exit 1
fi

# Mount usbmon if needed.
if [[ ! -d /sys/kernel/debug/usb/usbmon ]]; then
	modprobe usbmon
	mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
fi

# Find the bus number for the device so we capture the right usbmon node.
BUS=""
for d in /sys/bus/usb/devices/*; do
	[[ -f "$d/idVendor" && -f "$d/idProduct" ]] || continue
	if [[ "$(cat "$d/idVendor")" == "057e" &&
	      "$(printf '0x%s' "$(cat "$d/idProduct")")" == "$PID" ]]; then
		BUS="$(cat "$d/busnum")"
		break
	fi
done
[[ -n "$BUS" ]] || { echo "device 057e:$PID not found" >&2; exit 1; }
BUS=$((10#$BUS))

MONFILE="$HERE/usbmon-bus${BUS}-${PID}-${TS}.txt"
echo "Capturing usbmon bus ${BUS} -> $MONFILE for ${SECS}s"
timeout "${SECS}" cat "/sys/kernel/debug/usb/usbmon/${BUS}u" > "$MONFILE" &
MONPID=$!
sleep 0.3

# Kick off the init + hidraw decode (also writes its own capture-*.log).
echo "Running init + hidraw decode. Press buttons now..."
timeout "${SECS}" python3 "$HERE/sw2_capture.py" --pid "$PID" || true

wait "$MONPID" 2>/dev/null || true
echo "Done. Files:"
ls -la "$HERE"/usbmon-*"${TS}".txt "$HERE"/capture-*"$(printf '%04x' "$PID")"-*.log 2>/dev/null || true
