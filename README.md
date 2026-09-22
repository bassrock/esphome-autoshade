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
| Step counters | Position, persisted across reboots |
| Raw I²C to the PCA9685s | What each button does |
| Raw button bits | What the LCD says |
| Raw ADC counts | Schedule, homing, hold policy |

The AVR knows nothing about shades, feet, "up", or "down". Its protocol has no
vocabulary for any of it.

## Layout

```
components/autoshade_link/   the ESPHome external component
avr/autoshade_dumb.ino       firmware for the board's ATmega328P
example/autoshade.yaml       a complete working config
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
    travel_steps: 14868   # 2124 steps per foot, so 7 ft
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

### `cover` platform

| Option | Default | |
| --- | --- | --- |
| `motor` | required | 1–6 |
| `travel_steps` | required | full drop; 2124 steps per foot |
| `max_speed` | `400` | steps/s — the board's I²C bus tops out near 797 total |
| `acceleration` | `400` | steps/s² |
| `hold` | `brake` | `brake` shorts the windings at rest: no current, resists back-drive |
| `home_overrun` | `150` | steps driven into the top hard stop when homing |

### `binary_sensor` platform

`button:` is one of `select`, `right`, `down`, `up`, `left`.

## Wire protocol

ASCII, one command per line, 115200 8N1. Motors are 1–6.

| Command | Meaning |
| --- | --- |
| `V` | identify; also sent unprompted at boot |
| `Q` | poll → `Q P=… M=… K=… U=… A=… B=…` |
| `T n target sps acc endmask` | run to an absolute step count, then apply `endmask` |
| `S n` | decelerating stop (`n=0` = all) |
| `X n pos` | set the step counter without moving |
| `C n mask` | raw coil mask — `0x00` coast, `0x0F` brake |
| `L row text` | literal text to LCD row 0 or 1 |
| `G v` | backlight bits, R=1 G=2 B=4 |

`K=` is a sticky button mask, ORed at 20 Hz and cleared on each read, so a tap
between polls is never dropped.

## Notes

- **The board keeps no EEPROM.** Position lives in the ESP's flash. Opening the
  USB port pulses DTR, which resets the '328P, so its counters come back as
  zero — the component watches the `U=` uptime field and pushes positions back
  with `X` before acting on anything. A reset board is never trusted.
- **USB host needs an ESP32-S2, S3 or P4.** The C3 cannot do it: its USB block
  is a fixed-function Serial/JTAG device. A C3 can still be used over a plain
  UART to the '328P's RX/TX pins.
- **One motor moves at a time.** Not an ESP limitation — the 400 kHz I²C bus
  tops out near 797 steps/s across all channels.
- Coil phase is tracked separately from the step counter, so `X` and re-homing
  do not make the rotor jump.

## Status

First-draft code. The protocol and the architecture are settled; the component
has had limited real-world use. Issues and fixes welcome.

## License

None yet — a public repo without a license is all-rights-reserved by default.
Add MIT or similar if you want other people to be able to use this.
