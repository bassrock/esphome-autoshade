# esphome-autoshade

ESPHome control for the **AUTOSHADE** 6-channel roller-shade controller
(board `100108 Rev C`), over the board's own USB port — with no modification to
the board.

## What this is

The AUTOSHADE board turns out to be three Adafruit Motor Shield v2 clones plus
an Adafruit RGB LCD Shield clone on one PCB, driving six 4-wire bipolar
steppers through twelve DRV8871 H-bridges.

This project replaces its stock firmware with a **policy-free** AVR sketch and
moves every decision to ESPHome:

```
ESP32-S3  --USB host, CDC-ACM 115200-->  ATmega16U2  --UART-->  ATmega328P
                                                                    |
                                                          I2C 400 kHz bus
                                                   MCP23017 0x20 (LCD + buttons)
                                                   PCA9685 0x7C / 0x7D / 0x7E
```

| Runs on the AVR | Runs in ESPHome |
| --- | --- |
| Step pulse train | Travel distance, speed, acceleration |
| Step counters, saved to EEPROM at rest | Travel, what a position *means* |
| Raw I²C to the PCA9685s | What each button does |
| Raw button bits | What the LCD says |
| Raw ADC counts | Schedule, homing, hold policy |

The AVR knows nothing about shades, feet, "up", or "down". Its protocol has no
vocabulary for any of it.

## Layout

```
components/autoshade_link/   the ESPHome external component
avr/autoshade_dumb/          firmware for the board's ATmega328P
avr/build.sh                 builds it into components/autoshade_link/firmware/
example/autoshade.yaml       a complete working config
example/awning-shades.yaml   the live config, with setup and update buttons
```

## Using it

```yaml
external_components:
  - source: github://YOUR-USERNAME/esphome-autoshade@main
    components: [autoshade_link]
```

Then a hub and one cover per shade:

```yaml
usb_host:

usb_uart:
  - type: cdc_acm
    vid: 0
    pid: 0
    channels:
      - id: avr_uart
        baud_rate: 115200
        buffer_size: 1024

autoshade_link:
  id: avr
  uart_id: avr_uart
  update_interval: 500ms

cover:
  - platform: autoshade_link
    id: shade_1
    name: Shade 1
    motor: 1              # as silkscreened: SHADE 1 .. SHADE 6
    travel_steps: 9360    # ~130 steps per inch, so 6 ft
    hold: brake
    home_overrun: 150
```

See `example/autoshade.yaml` for the full thing, including the panel buttons,
the LCD text, the sunset schedule and the supply-voltage sensors.

### `autoshade_link` (hub)

| Option | Default | |
| --- | --- | --- |
| `uart_id` | required | any UART — `usb_uart` channel, or a plain `uart:` |
| `update_interval` | `500ms` | status poll rate |
| `supply_voltage` | — | sensor; 12 V rail, `scale` defaults to `0.01238` V/count |
| `battery_voltage` | — | sensor; 9 V backup, `scale` defaults to `0.009872` |
| `usb_channel` | — | the `usb_uart` channel id; needed for board firmware updates |
| `avr_firmware` | bundled | Intel HEX to flash; defaults to `firmware/autoshade_dumb.hex` |
| `motor_resistance` | `0.92` | ohms per winding, for the PWM drive modes |
| `motor_inductance` | `2.68` | mH per winding |
| `motor_back_emf` | `0.318` | V per rad/s |

### Updating the board firmware from the ESP

`avr/build.sh` compiles `autoshade_dumb.ino` (FQBN `arduino:avr:uno`; the
board has Optiboot) into `components/autoshade_link/firmware/autoshade_dumb.hex`,
which gets compiled into the ESP image. `id(avr)->start_avr_update()` then
pulses DTR to reset the '328P into Optiboot and writes and verifies the image
over the same USB link (STK500v1), all without a laptop. It is refused while a
shade moves. `board_version()`, `bundled_version()` and `avr_update_status()`
feed text sensors. A failed update never touches the bootloader; retry, or
flash from Arduino IDE.

### Drive modes (board firmware 3.4+)

The PCA9685s are PWM chips, so a DRV8871 input can be pulsed rather than
only switched. `id(avr)->set_drive(div, pwm)` picks 1, 2, 4 or 8 microsteps
per full step, with on/off coils (1 and 2 only) or sine PWM.
`set_drive_current(amps)` sets the PWM peak current. The DRV8871s cap current
near 1.6 A whatever you ask for. The board works out the duty from the winding
model (`motor_resistance`, `motor_inductance`, `motor_back_emf` on the hub;
defaults are the 23HS22-2804S), the present speed and the measured 12 V.
PWM runs at ~1.5 kHz, the PCA9685's limit. At 1/8 the I²C bus caps speed
near 97 full steps/s.

### `cover` platform

| Option | Default | |
| --- | --- | --- |
| `motor` | required | 1–6 |
| `travel_steps` | required | full drop; 2124 steps per foot |
| `max_speed` | `90` | full steps/s; the board half-steps. 150+ grinds |
| `acceleration` | `100` | full steps/s² |
| `hold` | `brake` | `brake` shorts the windings at rest: no current, resists back-drive |
| `home_overrun` | `150` | steps driven into the top hard stop when homing |

### `number` platform — shade length

```yaml
number:
  - platform: autoshade_link
    cover_id: shade_1
    name: Shade 1 Length     # inches, editable in HA
```

Options: `steps_per_foot` (default `2124`, which is ~1.35× too high for these
shades; measured 1510–1670, so set it), `max_length` (inches, default `240`).
A length set here, or by `set_bottom_here()`, is saved on the ESP and overrides
`travel_steps` from then on.

### Setting top and bottom

The cover exposes `jog(steps)` (negative = up, ignores limits), `set_top_here()`,
`set_bottom_here()`, `mark_position(pos)` (1.0 = up, 0.0 = down; tells the ESP
and board where the shade is without moving it) and `start_home()`. The link
has `set_all_speed()` / `set_all_accel()` for sliders. The example wires them to a
**Setup Shade** select, a **Jog Distance** number and **Jog Up / Jog Down /
Set Top Here / Set Bottom Here / Home Selected Shade** buttons. Set Top keeps
the bottom physically where it was, so either end can be redone on its own.

### `binary_sensor` platform

`button:` is one of `select`, `right`, `down`, `up`, `left`.

## Wire protocol

ASCII, one command per line, 115200 8N1. Motors are 1–6. Positions, speeds
and accelerations are full steps (200/rev); from firmware 3.3 the board drives
half steps internally, which stopped the stalls and grinding.

| Command | Meaning |
| --- | --- |
| `V` | identify; also sent unprompted at boot |
| `Q` | poll → `Q P=… M=… K=… U=… A=… B=… E=… F=…` |
| `T n target sps acc endmask` | run to an absolute step count, then apply `endmask` |
| `S n` | decelerating stop (`n=0` = all) |
| `X n pos` | set the step counter without moving |
| `C n mask` | raw coil mask — `0x00` coast, `0x0F` brake |
| `L row text` | literal text to LCD row 0 or 1 |
| `G v` | backlight bits, R=1 G=2 B=4 |
| `R addr len` | raw EEPROM read, up to 16 bytes, hex |

`E=1` means the `P=` counters are real (restored from EEPROM or set by `X`);
`E=0` means a blank board. `F=1` means the last move was cancelled because the
12 V motor supply dropped out.

`K=` is a sticky button mask, ORed at 20 Hz and cleared on each read, so a tap
between polls is never dropped.

## Notes

- **The board is the source of truth for position** (firmware 3.2+). It
  writes counters, coil phase and hold mode to EEPROM each time a motor comes
  to rest — two A/B slots with CRC, so a power cut mid-write can't corrupt
  both. On connect the ESP reads positions rather than pushing them. It pushes
  its own copy only when the board has nothing saved, runs older firmware, or
  reset part-way through a move (its last poll is then closer than the
  board's pre-move save). The ESP asserts DTR on connect, which resets the
  '328P; with EEPROM that costs nothing.
- **12 V interlock.** If the motor supply drops mid-move the board cancels
  motion within 20 ms, so it never counts steps the motor didn't take.
- **USB host needs an ESP32-S2, S3 or P4.** The C3 cannot do it: its USB block
  is a fixed-function Serial/JTAG device. A C3 can still be used over a plain
  UART to the '328P's RX/TX pins.
- **One motor moves at a time.** Not an ESP limitation — the 400 kHz I²C bus
  tops out near 797 steps/s across all channels.
- **LCD writes are held while a motor moves.** Each LCD line blocks the
  board's I²C for ~35 ms, which stalls a running stepper, so `send_lcd()` and
  `send_backlight()` wait until nothing is moving and 1 s has passed since the
  last move command.
- Coil phase is tracked separately from the step counter, so `X` and re-homing
  do not make the rotor jump.

## Status

First-draft code. The protocol and the architecture are settled; the component
has had limited real-world use. Issues and fixes welcome.

## License

None yet — a public repo without a license is all-rights-reserved by default.
Add MIT or similar if you want other people to be able to use this.
