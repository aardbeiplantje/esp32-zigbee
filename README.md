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
