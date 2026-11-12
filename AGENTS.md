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
