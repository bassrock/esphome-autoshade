#!/bin/sh
# Builds autoshade_dumb.ino for the board's ATmega328P (Optiboot, Uno FQBN) and
# drops the hex where the ESPHome component bundles it into the ESP image.
#
# Needs the Arduino AVR core plus the AccelStepper and Adafruit RGB LCD Shield
# libraries. Uses arduino-cli from PATH, or the one inside Arduino IDE 2.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
out="$here/../components/autoshade_link/firmware"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT

cli="$(command -v arduino-cli || true)"
[ -n "$cli" ] || cli="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"

"$cli" compile --fqbn arduino:avr:uno --output-dir "$build" "$here/autoshade_dumb"
mkdir -p "$out"
cp "$build/autoshade_dumb.ino.hex" "$out/autoshade_dumb.hex"
echo "wrote $out/autoshade_dumb.hex"
