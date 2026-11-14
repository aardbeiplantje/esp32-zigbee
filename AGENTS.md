# Agent instructions

## Git workflow

- Do not use branches. Do not create, switch to or work on any branch (including feature branches and worktrees) unless the user explicitly asks for one. Work and commit on `main`, which is checked out by default.
- When the user asks to finish or ship work, do all of these:
  1. Commit the changes.
  2. Merge every other local and remote branch into `main`.
  3. Delete the branches that were merged, locally and on the remote.
  4. Push `main`.
- Commit messages describe the change in plain words. They may name the model that made the change, and they end with a `Co-Authored-By:` trailer for that model.

## Hardware testing

- Build and flash with `python3 deploy.py deploy coordinator`. The board's serial AT port is the one with the `ESP-AT` output; find it with `python3 deploy.py list`.
- To test BLE without pressing BOOT, send `AT+BLE_ADV=1` over serial. It starts advertising for 10 seconds, the same as a BOOT press. `AT+BLE_ADV=0` stops it. Connect within that window, or send the command again.
- BLE client: `./ble_uart.pl 48:31:B7:C2:18:A8,addr_type=public`. Pass `addr_type=public` explicitly. The script guesses the address type from the first octet, and `48` is read as a private random address, so the connection times out without it. Confirmed by a connect that got `OK` and `BLE: connected`.
- BLE sniffer: `nrfutil ble-sniffer sniff --port /dev/ttyACM1 --output-pcap-file <file>`. Decode the capture with `tshark`.
- Power meter: `usb_logger log --id 2e3c:5558 --duration <N>s -t csv -o <file>`. Start it before the test so the baseline is in the same file.
- Keep test output in the scratchpad directory, not in the repository.
- Two boards are connected: `coordinator` and `enddevice` (see `boards.toml`). Find their ports with `python3 deploy.py list`; opening a port resets that board unless DTR/RTS are left low.
- A device that joined the network is in the stack's neighbour table, not in `getBoundDevices()`. `AT+ZBLIST?` reports both. An ESP32-H2's IEEE address is its base MAC with `FF:FE` in the middle.
- The Zigbee storage lives in its own flash partition, so `AT+ERASE` does not clear a stored network. To make a board forget it, erase the flash (`esptool --chip esp32h2 -p <port> erase-flash`) and reflash.

## Test devices

- Sonoff TRVZB valves. IEEE addresses are fixed. Short addresses change when a valve rejoins, so read the current one from `AT+ZBLIST?` before `AT+ZBREAD=<short>,1`.
  - Valve 1: IEEE `D4:48:67:FF:FE:0B:8E:D7`. Last short `0x2C26`; earlier joins gave `0x181F`. Its display and rotary dial misbehaved with fresh alkaline batteries.
  - Valve 2: IEEE `D4:48:67:FF:FE:0B:CA:BF`. Joined as short `0xE9F1` on 2026-11-13.
