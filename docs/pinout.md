# ESP32-H2 Super Mini pinout

This firmware targets the **ESP32-H2 Super Mini** (chip `esp32h2`, FQBN
`esp32:esp32:esp32h2`). The board pinout diagram is in
[`diagram-3200.png`](diagram-3200.png). This page lists the same pins as text.

Source: [espboards.dev ESP32-H2 Super Mini](https://www.espboards.dev/esp32/esp32-h2-super-mini/)
(CC BY-NC 4.0). Pin functions are from the ESP32-H2 Series Datasheet v1.3.

## Pins used by this firmware

| Function | GPIO | Notes |
|---|---|---|
| RGB LED (WS2812) | 8 | Status LED, driven with `rgbLedWrite()`. Strapping: ROM log output |
| BOOT button | 9 | Strapping pin. Normal press (120 ms to 2 s) starts BLE advertising |
| On-board blue LED | 13 | Plain LED, labelled `LED` on the board. Not used by the firmware |
| UART0 TX / RX | 24 / 23 | `U0TXD` / `U0RXD`, the AT command serial port |
| USB D- / D+ | 26 / 27 | Native USB, used for flashing and the USB serial port |
| JTAG source | 25 | Strapping/JTAG, do not use as a general GPIO |

The BAT charge LED has no GPIO and can't be controlled by software.

## Full pin table

Columns: board pad, GPIO, the pin's functions (from the diagram). `PWM` means the
pin can be used with PWM. `ADC` pins can be read with analogRead.

| Pad | GPIO | Functions | Notes |
|---|---|---|---|
| 24 | GPIO24 | U0TXD, FSPICS2 | UART0 TX, safe |
| 23 | GPIO23 | U0RXD, FSPICS1 | UART0 RX, safe |
| 0 | GPIO0 | FSPIQ | use with care |
| 1 | GPIO1 | ADC1_CH0, FSPICS0, LP GPIO | use with care |
| 2 | GPIO2 | ADC1_CH1, MTMS, FSPIWP | use with care |
| 3 | GPIO3 | ADC1_CH2, MTDO, FSPIHD | use with care |
| 4 | GPIO4 | ADC1_CH3, MTCK, FSPICLK | use with care |
| 5 | GPIO5 | ADC1_CH4, MTDI, FSPID | use with care |
| 8 | GPIO8 | RGB LED, ROM log, LP GPIO | strapping, WS2812 status LED |
| 9 | GPIO9 | BOOT, LP GPIO | strapping, BOOT button |
| 10 | GPIO10 | LP GPIO | safe |
| 11 | GPIO11 | LP GPIO | safe |
| 12 | GPIO12 | LP GPIO | safe |
| 13 | GPIO13 | LED, XTAL_32K_P, LP GPIO | on-board blue LED |
| 14 | GPIO14 | XTAL_32K_N, LP GPIO | safe |
| 22 | GPIO22 | (bottom pad) | safe |
| 25 | GPIO25 | FSPICS3, JTAG src | use with care |
| 26 | GPIO26 | FSPICS4, USB_D- | serial, USB |
| 27 | GPIO27 | FSPICS5, USB_D+ | serial, USB |

Power pads: 5V, 3V3, GND (and B+ / B- for the battery, bottom side).

Notes from the diagram:
- Strapping pins (GPIO8, GPIO9, and others marked "strapping") must keep their
  boot levels. The BOOT button on GPIO9 is pulled high by default.
- GPIO8 is the RGB LED, not a plain GPIO. Drive it with `rgbLedWrite()`, not
  with `digitalWrite()` or PWM.
- The `LOOP_DELAY` workaround that moves the button to GPIO3 is limited to
  ESP32-C3 and ESP32-S3 in `zigbee/esp-at.ino`, so on the H2 the BOOT button is
  read on GPIO9. Light-sleep wakeup from GPIO9 on the H2 is not tested yet.
