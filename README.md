# esp-zigbee

Firmware for an **ESP32-H2 Super Mini** (chip `esp32h2`) that runs as a **Zigbee coordinator** and is controlled
over an AT-command interface (UART, or BLE via the Nordic UART Service). It is
built on the ESP-AT template in `zigbee/esp-at.ino`; the Zigbee functionality
lives in `zigbee/zigbee.ino` and is hooked into the template through the weak
`PLUGINS::*` functions.

## Layout

| Path | Purpose |
|---|---|
| `zigbee/zigbee.ino` | Zigbee coordinator: endpoints, permit-join, device list, `AT+ZB*` commands |
| `zigbee/zigbee.h` | Zigbee plugin declarations and build-mode guard |
| `zigbee/esp-at.ino` | ESP-AT template: AT parser, config storage, main loop |
| `zigbee/common.{h,cpp}` | Logging and shared helpers |
| `zigbee/partitions.csv` | Flash layout (includes Zigbee storage partitions) |
| `boards.toml` | The boards: MAC, chip and role of each |
| `deploy.py` | Build, flash and monitor (replaces the old `build.sh`) |
| `ble_uart.pl`, `BLE_UART.md` | Perl terminal client for the BLE UART service |
| `monitor.py` | Quick helper: enable permit-join for 90 s and print the log |

## Zigbee AT commands

| Command | Description |
|---|---|
| `AT+ZBJOIN=<sec>` | Open the network for joining for `<sec>` seconds; `AT+ZBJOIN=0` closes it |
| `AT+ZBLIST?` | List bound devices, then every device on the network (neighbour table) |
| `AT+ZBNEIGHBORS?` | List the neighbour table only: who is on our network, with type, relationship, LQI and RSSI |
| `AT+ZBSCAN=1` | Scan for nearby Zigbee networks |
| `AT+ZBDEVICES=1` | Take a snapshot of the bound devices |
| `AT+ZBDEVICES?` | Show the snapshot taken by `AT+ZBDEVICES=1` |
| `AT+ZBTOGGLE` | Toggle a light, as a long (2-5 s) BOOT press does: on an end device the coordinator's, on the coordinator its own |
| `AT+ZBTOGGLE=<short>` | Toggle the light on the device with that short address, e.g. `AT+ZBTOGGLE=87FF` |
| `AT+ZBLIGHT=<0\|1>` | Switch our own light endpoint on or off |
| `AT+ZBLIGHT?` | Get our own light state |

The device exposes four endpoints: gateway (1), switch (2), light (3) and
thermostat (4). On boot the network is open for 180 seconds so devices can join.

A device that joins but binds nothing is in the neighbour table only, so
`AT+ZBLIST?` reports both: bound devices (`Endpoint=`) and network members
(`Rel=child`). The IEEE address of an ESP32-H2 is its base MAC with `FF:FE`
inserted in the middle, so board `48:31:B7:C2:19:45` appears as
`48:31:B7:FF:FE:C2:19:45`.

The template's general commands (`AT`, `AT+HELP?`, `AT+RESET`, `AT+ERASE`,
`AT+LOOP_DELAY`, `AT+VERBOSE`, and others) are also available. Run `AT+HELP?`
on the device for the full list of what is compiled in.

## Boards

The boards are ESP32-H2 Super Mini modules. The BOOT button is GPIO9 and the RGB
status LED is GPIO8 (WS2812, driven with `rgbLedWrite()`).

`boards.toml` has one table per board. The ESP32-H2 has native USB, so its USB
serial number is its MAC, and `deploy.py` finds the board by that. No reset is
needed to find it.

```toml
[boards.coordinator]
mac  = "48:31:B7:C2:18:A8"
chip = "esp32h2"
role = "COORDINATOR"
```

To add a board, plug it in and run `python3 deploy.py identify <port>`. It prints
the entry to paste into `boards.toml`.

## Two boards: a button and a light

`boards.toml` holds a `COORDINATOR` and an `ENDDEVICE`. They run the same
sketch; the role picks the Zigbee mode (`ZigbeeMode=zczr` with
`-DZIGBEE_MODE_ZCZR`, or `ZigbeeMode=ed` with `-DZIGBEE_MODE_ED`).

```bash
python3 deploy.py deploy coordinator
python3 deploy.py deploy enddevice
```

The end device joins on boot while the coordinator's network is open (180 s
after a coordinator reboot, or after `AT+ZBJOIN=<sec>`). Check it joined with
`AT+ZBLIST?` on the coordinator: it appears as `Rel=child`.

Then a long (2-5 s) press of BOOT on the end device toggles the coordinator's
light endpoint, and the coordinator's RGB LED turns green while it is on.
`AT+ZBTOGGLE` does the same thing without the button. A joined device needs no
binding for this: the command is addressed straight at endpoint 3 of short
address 0x0000.

The two boards are easy to tell apart by their idle LED: the coordinator glows
dim blue, the end device (the one with the button) dim purple. The
coordinator turns bright orange while its light is on (orange rather than
green: against the dim blue idle glow, green is hard to tell apart).

The status LED shows the light state only while nothing more urgent needs it
(BLE advertising and a BLE connection still win). A plugin asks for that
through the `PLUGINS::led_idle_color()` hook in `plugins.h`, and takes the
otherwise unused long press through `PLUGINS::button_long_press()`.

## IKEA STYRBAR remote

The STYRBAR (E2002) is a Zigbee 3.0 remote that joins the coordinator as a
sleepy end device. Tested with the coordinator's light on endpoint 3.

Steps that worked:

1. Open the network: `AT+ZBJOIN=254` (the join window is capped at 254 s).
2. Pair the remote: hold it next to the coordinator, then press the small pair
   button on the back four times quickly. Put the lid back on first: the back
   cover holds the buttons against the board, so presses don't register without it.
3. Check it joined: `AT+ZBLIST?` shows it as `Rel=child`, or `AT+ZBNEIGHBORS?`.
4. Find its endpoint: `AT+ZBEPS=<short>` (short address from step 3), then
   `AT+ZBEPS?`. The STYRBAR has endpoint 1 with On/Off as an output cluster (`0x0006`).
5. Bind it to our light: `AT+ZBBIND=<short>,<endpoint>`. The coordinator resends
   every 6 s for up to 120 s, so press a button on the remote while it retries.
   The log shows `Bind OK` when it works.
6. Press ON and OFF on the remote. The log shows `Light switched ON/OFF`, and the
   coordinator's LED is orange while the light is on, dim blue when it is off.

Things that caught us out:

- A sleeping device answers a request only in a short window after a button
  press. Requests time out after 5 s (status 133), which is why the bind retries.
- `openNetwork()` takes a `uint8_t`, so a timeout over 255 s wraps. Use 254 or less.
- `getBoundDevices()` only lists devices bound to our endpoints, so a plain join
  does not show up there. `AT+ZBLIST?` reads the neighbour table as well.

Background reading:

- [Zigbee2MQTT: IKEA E2001/E2002/E2313](https://www.zigbee2mqtt.io/devices/E2001_E2002_E2313.html):
  cluster notes and pairing.
- [Zigbee2MQTT discussion #20625](https://github.com/Koenkk/zigbee2mqtt/discussions/20625):
  binding on newer firmware.
- [Home Assistant: STYRBAR bind/unbind issue](https://community.home-assistant.io/t/ikea-styrbar-e2001-e2002-bind-unbind-issue/922254).
- [Home Assistant: STYRBAR not pairing because of Tuya lights](https://community.home-assistant.io/t/solved-ikea-styrbar-e2001-e2002-not-pairing-via-zigbee2mqtt-due-to-tuya-lights/929237).
- [Telldus: installing the STYRBAR](https://telldus.com/en/blogs/faq/installera-styrbar-till-hub-1-med-telldus-flow-app):
  pairing LED sequence.
- [IKEA STYRBAR manual summary](https://manuals.plus/ikea/styrbar-zigbee-remote-control-manual):
  range 10 m with no walls, indoor use only, up to 10 lights per remote.
- [IKEA product page](https://www.ikea.com/fi/en/p/styrbar-remote-control-smart-white-30488363).
- [Zigbee 3.0 certification listing](https://csa-iot.org/csa_product/styrbar-remote-control/).

## Other Zigbee devices

### Sonoff TRVZB (radiator valve)

A thermostatic radiator valve with a built-in temperature sensor. The coordinator
reads and writes its thermostat cluster (0x0201) on endpoint 1 directly.

**Works (tested on channel 13):**
- Joining: hold BOOT on the coordinator for 5 s (magenta), then put the valve in
  pairing mode: Auto off, dial to `OF`, hold the middle button for 3 s until the
  signal icon flashes. The log shows `A device joined`, and `AT+ZBLIST?` lists it.
- `AT+ZBEPS=<short>` then `AT+ZBEPS?`: endpoints and clusters.
- `AT+ZBREAD=<short>,<ep>`: room temperature and setpoint. The valve only answers
  while awake, so press a button on it and retry.
- `AT+ZBSETPOINT=<short>,<ep>,<C>` (5-30 °C): tested 19 → 21 → 19 °C.
- `AT+ZBVALVE?`: the last values read.

**Not supported yet:**
- Driving the valve opening directly. Zigbee2MQTT exposes `valveOpeningDegree` and
  `valveClosingDegree` as manufacturer attributes, but we haven't found their IDs.
  Manufacturer clusters `0xFC11` and `0xFC57` both appear on endpoint 1.
- Heat demand for the boiler (the thermostat's running state): not checked.
- Pairing on channel 15: the valve never joined there in our tests.
- Reads and writes need the valve awake. The firmware retries each one every 6 s
  for up to 120 s.

- Product: [Sonoff Zigbee thermostatic radiator valve](https://sonoff.tech/en-eu/products/sonoff-zigbee-thermostatic-radiator-valve)
- Zigbee2MQTT: [TRVZB](https://zigbee2mqtt.io/devices/TRVZB.html) (setpoint and local temperature attributes)
- Setup notes: [PrivateHomelab: TRVZB with Zigbee2MQTT](https://privatehomelab.com/sonoff-trvzb-radiator-valve-zigbee2mqtt-home-assistant/)
- The body sensor reads warmer than the room, since the valve sits on the radiator. An external room reading can be used instead.

## Building and flashing

Requires Python 3.11+, `pyserial`, and `arduino-cli`.

```bash
python3 deploy.py update                  # install the esp32 core and SerialCommands
python3 deploy.py list                    # ports, and the board on each
python3 deploy.py build coordinator       # compile into dist/esp32h2/zigbee/
python3 deploy.py flash coordinator       # flash the last build
python3 deploy.py deploy coordinator --monitor   # build, flash, then show the serial output
python3 deploy.py monitor coordinator     # serial output, prefixed with the board name
python3 deploy.py deploy all              # every board in boards.toml that is connected
```

`build.sh` commands map to `deploy.py` as: `build` → `build`, `upload` → `flash`,
`monitor` → `monitor`, `deploy` → `deploy`, `update` → `update`, `all` → `deploy --monitor`.

### Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `ARDUINO_DIRECTORIES_DATA` | `./.arduino15` if present, else arduino-cli's | Arduino data directory |
| `DEBUG` | unset | Set to `1` to add `-DDEBUG` |
| `VERBOSE` | `1` | Set to `0` to omit `-DVERBOSE` |
| `EXTRA_FLAGS` | (none) | Extra compiler flags, appended as-is (also `--flags`) |

Example:

```bash
DEBUG=1 python3 deploy.py deploy coordinator
```

## Host tools

### `monitor.py`

Opens `/dev/ttyACM0` at 460800 baud, sends `AT+ZBJOIN=90` and prints the
device's output for 95 seconds. Edit the port and timing inside the script if
your setup differs.

### `ble_uart.pl`

A terminal client for devices that expose the Nordic UART Service over BLE.
See `BLE_UART.md` or run `perl ble_uart.pl --man`.

```bash
./ble_uart.pl --help
./ble_uart.pl XX:XX:XX:XX:XX:XX    # connect to a device
```

## License

Released into the public domain under the Unlicense. See `LICENSE`.
