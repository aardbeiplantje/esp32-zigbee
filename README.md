### Supported #define Options and Capabilities

The firmware supports the following compile-time options and capabilities (set via `#define` or build flags):

- `VERBOSE` (default enabled): Enables verbose logging.
- `DEBUG`: Enables debug logging.
- `DEFAULT_HOSTNAME` (default: "uart"): Default device hostname.
- `UART_AT` (default enabled): Enables UART AT command interface.
- `BLUETOOTH_UART_AT` (default enabled): Enables Bluetooth UART AT command interface.
- `BLUETOOTH_UART_DEVICE_NAME` (default: `DEFAULT_HOSTNAME`): Bluetooth device name.
- `BLUETOOTH_UART_DEFAULT_PIN` (default: "1234"): Bluetooth pairing PIN (Classic BT).
- `DEFAULT_NTP_SERVER` (default: "at.pool.ntp.org"): Default NTP server.
- `DEFAULT_DNS_IPV4` (default: "1.1.1.1"): Default DNS for static IPv4.
- `SUPPORT_NTP` (default enabled): Enables NTP support.
- `SUPPORT_UDP` / `SUPPORT_TCP`: Enables UDP/TCP forwarding (if defined).

### Supported AT Commands

The following AT commands are supported via UART, BLE, or Bluetooth (if enabled). Use `AT+?` for a quick command list or `AT+HELP?` for detailed help with descriptions and examples.

#### Basic Commands
- `AT` — Test AT startup
- `AT?` — Test AT startup  
- `AT+?` — Show quick command list
- `AT+HELP?` — Show detailed help with descriptions
- `AT+RESET` — Restart device
- `AT+ERASE` — Erase all configuration, reset to factory defaults
- `AT+ERASE=1` — Erase all configuration and restart immediately

#### UART Commands (when SUPPORT_UART1 enabled)
- `AT+UART1=baud,data,parity,stop,rx,tx` — Configure UART1 parameters
- `AT+UART1?` — Get current UART1 configuration

#### BLE Commands (when BLUETOOTH_UART_AT enabled)
- `AT+BLE_PIN=<pin>` / `AT+BLE_PIN?` — Set/get BLE PIN (6 digits)
- `AT+BLE_SECURITY=<mode>` / `AT+BLE_SECURITY?` — Set/get BLE security mode
- `AT+BLE_IO_CAP=<cap>` / `AT+BLE_IO_CAP?` — Set/get BLE IO capability
- `AT+BLE_AUTH_REQ=<req>` / `AT+BLE_AUTH_REQ?` — Set/get authentication requirements
- `AT+BLE_STATUS?` — Get BLE connection and security status

#### System Commands
- `AT+LOOP_DELAY=<ms>` / `AT+LOOP_DELAY?` — Set/get main loop delay
- `AT+VERBOSE=<0|1>` / `AT+VERBOSE?` — Enable/disable verbose logging (when VERBOSE enabled)
- `AT+TIMELOG=<0|1>` / `AT+TIMELOG?` — Enable/disable time logging (when TIMELOG enabled)
- `AT+LOG_UART=<0|1>` / `AT+LOG_UART?` — Enable/disable UART logging (when LOGUART enabled)

**Note**: Commands with `?` are queries, commands with `=` set values. Many commands are conditionally compiled based on `#define` flags.
## esp-at-template

This project provides a template for building and uploading ESP-AT firmware using Arduino CLI. It includes a flexible `build.sh` script for managing builds, uploads, and serial monitoring, with support for environment variable overrides and custom configuration. The template is designed to simplify development and deployment for ESP32-based AT firmware projects.

## BLE UART Client

The project includes `ble_uart.pl`, a Perl script that provides a BLE UART (Nordic UART Service) client for connecting to and communicating with ESP32 devices over Bluetooth Low Energy. This script allows you to:

- Connect to one or more BLE devices implementing the Nordic UART Service (NUS)
- Send and receive UART-style data over BLE interactively
- Execute AT commands and see responses in real-time
- Manage multiple simultaneous BLE connections
- Run scripted commands from files
- Support for colored terminal output and command history

For detailed usage information and command reference, see the manual page:

```bash
perl ble_uart.pl --man
```

Or refer to the [BLE_UART.md](BLE_UART.md) documentation.

### build.sh Commands

The `build.sh` script supports the following commands (pass as the first argument):

- `build` or `compile`: Update platform/libs and build the project.
- `update`: Update platform/libs only (no build).
- `upload`: Upload the compiled firmware to the device.
- `monitor`: Open a serial monitor to the device.
- `deploy`: Build and upload the firmware.
- *(no argument or unknown command)*: Update, build, upload, and monitor in sequence.

Example:

```
bash ./build.sh build
bash ./build.sh upload
bash ./build.sh monitor
```

## BUILD


### Environment Variables for `build.sh`

You can customize the build and upload process using the following environment variables:

- `MODULE` (default: `esp-at`): The Arduino project directory or sketch name.
- `DEV_PLATFORM` (default: `esp32:esp32`): Arduino platform to use.
- `DEV_BOARD` (default: `esp32:esp32:esp32c3`): Board identifier for arduino-cli.
- `DEV_PORT` (default: `/dev/ttyACM0`): Serial port for upload/monitor.
- `DEV_BOARD_BAUDRATE` (default: `460800`): Baudrate for monitor.
- `ARDUINO_DIRECTORIES_DATA` (default: `$HERE/.arduino15`): Arduino data directory.
- `TMPDIR` (default: `/var/tmp`): Temporary directory.
- `DEV_URLS` (default: `https://dl.espressif.com/dl/package_esp32_index.json`): Additional URLs for board manager.
- `DEV_UPDATE` (default: `0`): Set to `1` to update platform/libs.
- `DEBUG`: Set to `1` to add `-DDEBUG` flag.
- `VERBOSE`: Set to any value to add `-DVERBOSE` flag.
- `DEFAULT_NTP_SERVER`: Set to override default NTP server.
- `DEFAULT_HOSTNAME`: Set to override default hostname.
- `DEFAULT_BLUETOOTH_NAME`: Set to override default Bluetooth name.
- `DEFAULT_BLUETOOTH_PIN`: Set to override default Bluetooth PIN.

Example usage:

```
DEV_PORT=/dev/ttyUSB0 DEFAULT_HOSTNAME=myesp DEBUG=1 bash ./build.sh build
```

First update platform/modules:
```
DEV_PORT=/dev/ttyUSB0 bash ./build.sh update
```

Then build:
```
DEV_PORT=/dev/ttyUSB0 bash ./build.sh build
```

## BLE UART Client Tool

The repository includes `ble_uart.pl`, a Perl script that provides a terminal interface for communicating with BLE devices that implement the Nordic UART Service (NUS). This tool can be used to interact with ESP32 devices running the BLE UART AT firmware.

### Usage

```bash
./ble_uart.pl --help                # Show full documentation
./ble_uart.pl XX:XX:XX:XX:XX:XX    # Connect to a BLE device
```

### Available Commands

Once connected, you can use these commands in the interactive terminal:

- `/help` — Show available commands
- `/connect XX:XX:XX:XX:XX:XX` — Connect to a BLE device
- `/disconnect` — Disconnect from current device
- `/switch XX:XX:XX:XX:XX:XX` — Switch between connected devices
- `/info` — Read device name using the Device Name characteristic (00002a00)
- `/primary` — Discover primary services
- `/char-desc` — Discover characteristic descriptors
- `/debug on|off` — Enable/disable debug logging
- `/exit` — Exit the application

### Device Information

The `/info` command reads the Device Name characteristic (UUID: 00002a00-0000-1000-8000-00805f9b34fb) to display basic device information:

```
> /info
Device Name: ESP32-BLE-Device
```

This is useful for quickly identifying connected devices and verifying the connection is working properly.
