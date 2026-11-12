#!/usr/bin/env python3
"""Build, flash and monitor the Zigbee coordinator firmware (zigbee/zigbee.ino).

Boards are listed in boards.toml (MAC, chip, role, and optionally a port). The
ESP32-H2 has native USB, so its USB serial number is its MAC: a board is found by
that, and no reset or esptool call is needed for it. Note that opening the serial
port itself reboots the board (USB reset), so monitoring restarts it.

  python3 deploy.py list                         ports and the board on each
  python3 deploy.py identify /dev/ttyACM2        chip + MAC of a new board (boards.toml entry)
  python3 deploy.py update                       install the arduino core and libraries
  python3 deploy.py build   coordinator|all
  python3 deploy.py flash   coordinator          flash the last build
  python3 deploy.py deploy  coordinator --monitor   build + flash, then serial output
  python3 deploy.py monitor coordinator          serial output, prefixed with the board name

Needs Python 3.11+ (tomllib), pyserial, arduino-cli with the esp32 core (see 'update').
"""

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import tomllib

HERE = os.path.dirname(os.path.abspath(__file__))
BOARDS_FILE = os.path.join(HERE, "boards.toml")
DIST = os.path.join(HERE, "dist")
BUILD_CACHE = os.path.join(HERE, ".build")
ROLES = ("COORDINATOR",)
FLASH_BAUD = 460800      # native USB: the transfer does not depend on it
MONITOR_BAUD = 115200    # the firmware's serial console (esp-at.ino setup)

# The arduino cores, by the name arduino-cli uses for them. 'update' installs them.
CORES = {
    "esp32:esp32": {"version": "3.3.12",
                    "index": "https://espressif.github.io/arduino-esp32/package_esp32_index.json"},
}
# The Zigbee library comes with the esp32 core; SerialCommands does not.
LIBRARIES = ("SerialCommands@2.2.0",)

# Per chip: the arduino-cli FQBN, the esptool (a path under the arduino packages directory, and
# the --chip name), the compiler flags, the board options, the sketch folder per role and the
# flash layout ({sketch} is the sketch name).
# zigbee/partitions.csv has no otadata partition, so no boot_app0.bin is written.
CHIPS = {
    "esp32h2": {
        "fqbn": "esp32:esp32:esp32h2",
        "esptool": "esp32/tools/esptool_py/*/esptool",
        "esptool_chip": "esp32h2",
        "flags": "-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -D_ARDUINO_BLE_H_ -DZIGBEE_MODE_ZCZR",
        "board_options": ("PartitionScheme=no_ota", "ZigbeeMode=zczr"),
        "sketches": {"COORDINATOR": "zigbee"},
        "layout": (
            ("0x0", "{sketch}.ino.bootloader.bin"),
            ("0x8000", "{sketch}.ino.partitions.bin"),
            ("0x10000", "{sketch}.ino.bin"),
        ),
    },
}


def die(msg):
    sys.exit(f"deploy: {msg}")


def norm_mac(mac):
    """'48:31:B7:C2:18:A8', '48-31-...' or '4831b7c218a8' -> '48:31:b7:c2:18:a8' (None if not a MAC)."""
    h = re.sub(r"[^0-9a-fA-F]", "", mac or "")
    if len(h) != 12:
        return None
    return ":".join(h[i:i + 2] for i in range(0, 12, 2)).lower()


# ---- boards.toml ------------------------------------------------------------

def load_boards():
    with open(BOARDS_FILE, "rb") as f:
        cfg = tomllib.load(f)
    boards = {}
    for name, b in cfg.get("boards", {}).items():
        role = str(b.get("role", "")).upper()
        chip = str(b.get("chip", "")).lower()
        if role not in ROLES:
            die(f"boards.toml: [boards.{name}] role must be one of {', '.join(ROLES)}")
        if chip not in CHIPS:
            die(f"boards.toml: [boards.{name}] chip must be one of {', '.join(CHIPS)}")
        mac = b.get("mac", "")
        if mac and not norm_mac(mac):
            die(f"boards.toml: [boards.{name}] mac '{mac}' is not a MAC address")
        boards[name] = {"name": name, "role": role, "chip": chip,
                        "mac": norm_mac(mac), "port": b.get("port", "")}
    return boards


def select(boards, names):
    """Board names, the role (any case) or 'all' -> (boards in boards.toml order, strict).
    strict means a named board must be connected; 'all' skips the ones that are not."""
    if not names:
        die("name a board, the role or 'all' (see 'deploy.py list')")
    if "all" in names:
        return list(boards.values()), False
    picked = []
    for n in names:
        if n in boards:
            hits = [boards[n]]
        elif n.upper() in ROLES:
            hits = [b for b in boards.values() if b["role"] == n.upper()]
        else:
            die(f"no board or role '{n}' in boards.toml (have: {', '.join(boards)})")
        picked += [b for b in hits if b not in picked]
    return picked, True


# ---- ports ------------------------------------------------------------------

def list_ports():
    """[(device, serial number, description)] for the USB serial ports."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        die("pyserial is missing: pip install pyserial (or apt install python3-serial)")
    return [(p.device, p.serial_number or "", p.description or "")
            for p in sorted(lp.comports(), key=lambda p: p.device) if p.vid is not None]


def stable_port(dev):
    """The /dev/serial/by-id/... name of a port, which survives replugging; the port itself otherwise."""
    for link in sorted(glob.glob("/dev/serial/by-id/*")):
        if os.path.realpath(link) == os.path.realpath(dev):
            return link
    return dev


def open_quiet(port):
    """Open a serial port with DTR and RTS low from the start, so the open does not add a reset."""
    import serial
    s = serial.Serial()
    s.port = port
    s.baudrate = MONITOR_BAUD
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
    return s


def locate(board):
    """The port of a board: the USB port whose serial number is its MAC. A board without a MAC uses its
    configured port. None when it is not connected."""
    if board["mac"]:
        for dev, serial_no, _ in list_ports():
            if norm_mac(serial_no) == board["mac"]:
                return dev
        return None
    return board["port"] if board["port"] and os.path.exists(board["port"]) else None


def present(picked, strict, port):
    """The selected boards that are connected now, each with 'found' set to its port."""
    if port:
        if len(picked) != 1:
            die("--port works with one board only")
        picked[0]["found"] = port
        return picked
    out = []
    for b in picked:
        b["found"] = locate(b)
        if b["found"]:
            out.append(b)
        elif strict:
            die(f"{b['name']} ({b['mac'] or b['port'] or '?'}) is not connected (see 'deploy.py list')")
        else:
            print(f"(skipping {b['name']}: not connected)")
    if not out:
        die("none of the selected boards is connected (see 'deploy.py list')")
    return out


# ---- tools ------------------------------------------------------------------

def arduino_data_dir():
    """$ARDUINO_DIRECTORIES_DATA, else ./.arduino15 if it exists, else arduino-cli's own default."""
    d = os.environ.get("ARDUINO_DIRECTORIES_DATA")
    if not d and os.path.isdir(os.path.join(HERE, ".arduino15")):
        d = os.path.join(HERE, ".arduino15")
    if d:
        os.environ["ARDUINO_DIRECTORIES_DATA"] = d
        return d
    try:
        r = subprocess.run(["arduino-cli", "config", "get", "directories.data"],
                           capture_output=True, text=True, check=True)
        return r.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return os.path.expanduser("~/.arduino15")


def version_key(path):
    return [int(x) if x.isdigit() else 0 for x in re.split(r"[.\-]", path)]


def core_file(pattern):
    """Newest match of a glob under the arduino packages directory."""
    found = glob.glob(os.path.join(arduino_data_dir(), "packages", pattern))
    return max(found, key=version_key) if found else None


def esptool_cmd(chip):
    """The esptool of the chip's core, else one on PATH, else 'python3 -m esptool'."""
    found = core_file(CHIPS[chip]["esptool"])
    if found:
        return [sys.executable, found] if found.endswith(".py") else [found]
    for name in ("esptool", "esptool.py"):
        if shutil.which(name):
            return [name]
    return [sys.executable, "-m", "esptool"]


def esptool_v5(cmd):
    """esptool 5 spells commands write-flash/chip-id, older ones write_flash/chip_id."""
    try:
        r = subprocess.run(cmd + ["version"], capture_output=True, text=True, timeout=30)
        m = re.search(r"(\d+)\.\d+", r.stdout + r.stderr)
        return bool(m) and int(m.group(1)) >= 5
    except (OSError, subprocess.TimeoutExpired):
        return True


V4_NAMES = {"write-flash": "write_flash", "chip-id": "chip_id",
            "--flash-mode": "--flash_mode", "--flash-freq": "--flash_freq", "--flash-size": "--flash_size"}


def esptool_argv(chip, args):
    """The command for the chip's esptool with args, spelled for its version."""
    cmd = esptool_cmd(chip)
    if not esptool_v5(cmd):
        args = [V4_NAMES.get(a, a) for a in args]
    return cmd + args


def run_esptool(chip, args):
    """Run the chip's esptool, echo its output, return (exit code, output)."""
    cmd = esptool_argv(chip, args)
    print("+ " + " ".join(cmd), flush=True)
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    except OSError as e:
        die(f"cannot run esptool: {e}")
    out = []
    for line in p.stdout:
        sys.stdout.write(line)
        out.append(line)
    return p.wait(), "".join(out)


CHIP_RE = r"(ESP32-[CHS]\d|ESP32|ESP8266)"


def parse_esptool(out):
    """Chip ('esp32h2', ...) and MAC from esptool output."""
    chip = re.search(r"Detecting chip type\.\.\. *" + CHIP_RE, out) or \
        re.search(r"Chip (?:is|type:) *" + CHIP_RE, out)
    # esptool prints 'BASE MAC:' (6 bytes, the one USB serial numbers use) and, on the H2, a longer 'MAC:'
    mac = re.search(r"BASE MAC: *([0-9a-fA-F:]{17})", out) or re.search(r"MAC: *([0-9a-fA-F:]{17})", out)
    return (chip.group(1).replace("-", "").lower() if chip else None,
            norm_mac(mac.group(1)) if mac else None)


# ---- build ------------------------------------------------------------------

def sketch_name(board):
    return CHIPS[board["chip"]]["sketches"][board["role"]]


def out_dir(board):
    return os.path.join(DIST, board["chip"], sketch_name(board))


def build_flags(board, args):
    """Compiler defines: the chip's, -DDEBUG with DEBUG=1, -DVERBOSE unless VERBOSE=0, then EXTRA_FLAGS and --flags."""
    flags = [CHIPS[board["chip"]]["flags"]]
    if os.environ.get("DEBUG") == "1":
        flags.append("-DDEBUG")
    if os.environ.get("VERBOSE", "1") == "1":
        flags.append("-DVERBOSE")
    flags += (os.environ.get("EXTRA_FLAGS", "") + " " + " ".join(args.flags or [])).split()
    return " ".join(flags)


def build_properties(flags):
    """The arduino-cli build properties of the firmware (the settings the old build.sh passed)."""
    opt = f"{flags} -ffunction-sections -fdata-sections -fno-exceptions"
    return {
        "compiler.cpp.extra_flags": opt,
        "compiler.c.extra_flags": opt,
        "compiler.S.extra_flags": opt,
        "compiler.c.elf.extra_flags": "-O2 -Wl,--gc-sections",
        "compiler.ldflags.extra_flags": "-Wl,--gc-sections",
        "build.flags.lto": "false",
        "build.extra_flags": flags,
        "build.custom_partitions": "partitions",     # zigbee/partitions.csv
        "build.zigbee_mode": "2",
        "upload.maximum_size": "1851392",
    }


def build(board, flags, args):
    chip = CHIPS[board["chip"]]
    sketch = sketch_name(board)
    dst = out_dir(board)
    os.makedirs(dst, exist_ok=True)
    print(f"== build {sketch} for {board['chip']} -> {os.path.relpath(dst, HERE)}")
    print(f"   flags: {flags}", flush=True)
    arduino_data_dir()
    cmd = ["arduino-cli", "compile", "-b", chip["fqbn"],
           "--output-dir", dst,
           "--build-path", os.path.join(BUILD_CACHE, f"{board['chip']}-{sketch}"),
           "--jobs", "16"]
    for k, v in build_properties(flags).items():
        cmd += ["--build-property", f"{k}={v}"]
    for opt in chip["board_options"]:
        cmd += ["--board-options", opt]
    cmd.append(os.path.join(HERE, sketch))
    if args.verbose:
        cmd.insert(2, "-v")
    if subprocess.run(cmd).returncode:
        die(f"build of {sketch} for {board['chip']} failed")
    with open(os.path.join(dst, "build.json"), "w") as f:
        json.dump({"sketch": sketch, "role": board["role"], "chip": board["chip"], "flags": flags,
                   "built": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)


def build_boards(picked, args):
    done = set()     # one build per chip + role, also for a second board of the same role
    for b in picked:
        key = (b["chip"], b["role"])
        if key not in done:
            build(b, build_flags(b, args), args)
            done.add(key)


# ---- flash and monitor ------------------------------------------------------

def flash(board, args):
    chip = CHIPS[board["chip"]]
    src = out_dir(board)
    sketch = sketch_name(board)
    bins = [(addr, os.path.join(src, name.format(sketch=sketch))) for addr, name in chip["layout"]]
    if any(not os.path.exists(p) for _, p in bins):
        die(f"{board['name']}: no build in {os.path.relpath(src, HERE)}, run 'deploy.py build {board['name']}'")
    try:
        with open(os.path.join(src, "build.json")) as f:
            info = json.load(f)
    except (OSError, ValueError):
        info = {}
    newest_src = max(os.path.getmtime(p) for p in glob.glob(os.path.join(HERE, sketch, "*.*")))
    if os.path.getmtime(bins[-1][1]) < newest_src:
        print(f"WARNING: {board['name']}: the build is older than the sketch sources")
    port = board["found"]
    print(f"== flash {board['name']} ({board['role']}, {board['chip']}) on {port}"
          f"  built {info.get('built', '?')}  flags: {info.get('flags') or '(none)'}", flush=True)
    rc, out = run_esptool(
        board["chip"],
        ["--chip", chip["esptool_chip"], "--port", port, "--baud", str(FLASH_BAUD),
         "--before", "default-reset", "--after", "hard-reset",
         "write-flash", "-z", "--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep"]
        + [x for addr, path in bins for x in (addr, path)])
    chip_found, mac = parse_esptool(out)
    if rc:
        die(f"flashing {board['name']} on {port} failed (esptool exit {rc}); "
            f"is a monitor still open on {port}?")
    if board["mac"] and mac and mac != board["mac"]:
        die(f"{port} turned out to be {mac}, not {board['name']} ({board['mac']})")
    print(f"== {board['name']} flashed OK ({chip_found or '?'}, {mac or '?'})")


def cmd_flash(args, boards):
    picked, strict = select(boards, args.boards)
    for b in present(picked, strict, args.port):
        flash(b, args)


_say_lock = threading.Lock()


def say(line):
    """One whole line per write, so the lines of two boards never interleave."""
    with _say_lock:
        sys.stdout.write(line + "\n")
        sys.stdout.flush()


def monitor_one(board, port_override, prefix, stop):
    import serial
    tag = f"[{board['name']}] " if prefix else ""
    while not stop.is_set():
        port = port_override or locate(board)
        if not port:
            time.sleep(0.5)          # not plugged in (yet): wait for it to come back
            continue
        try:
            s = open_quiet(port)
        except (serial.SerialException, OSError):
            time.sleep(0.5)
            continue
        say(f"{tag}--- connected on {port} ---")
        buf = b""
        try:
            while not stop.is_set():
                buf += s.read(4096)
                *lines, buf = buf.split(b"\n")
                for line in lines:
                    say(tag + line.decode("utf-8", "replace").rstrip("\r"))
        except (serial.SerialException, OSError):
            say(f"{tag}--- disconnected ---")
        finally:
            s.close()


def cmd_monitor(args, boards):
    picked, _ = select(boards, args.boards)     # monitor does not require the boards: it waits for them
    if args.port and len(picked) > 1:
        die("--port works with one board only")
    stop = threading.Event()
    threads = [threading.Thread(target=monitor_one, args=(b, args.port, len(picked) > 1, stop), daemon=True)
               for b in picked]
    for t in threads:
        t.start()
    print("--- monitor: Ctrl-C to quit ---", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()


# ---- commands ---------------------------------------------------------------

def cmd_list(args, boards):
    by_mac = {b["mac"]: b for b in boards.values() if b["mac"]}
    seen = set()
    print(f"{'PORT':24} {'BOARD':12} {'ROLE':12} {'CHIP':9} NOTE")
    for dev, serial_no, desc in list_ports():
        b = by_mac.get(norm_mac(serial_no))
        if b:
            seen.add(b["name"])
            print(f"{dev:24} {b['name']:12} {b['role']:12} {b['chip']:9}")
        else:
            print(f"{dev:24} {'-':12} {'-':12} {'-':9} unknown: deploy.py identify {dev}  ({desc})")
    for b in boards.values():
        if b["name"] not in seen:
            print(f"{b['port'] or '-':24} {b['name']:12} {b['role']:12} {b['chip']:9} (not connected)")


def cmd_identify(args, boards):
    rc, out = run_esptool("esp32h2", ["--port", args.port, "chip-id"])
    chip, mac = parse_esptool(out)
    if rc or not mac:
        die(f"esptool could not read {args.port} (is the board in download mode?)")
    known = [b for b in boards.values() if b["mac"] == mac]
    print()
    if known:
        b = known[0]
        print(f"{args.port}: {mac} is [boards.{b['name']}] ({b['role']}, {b['chip']})")
        if chip and chip != b["chip"]:
            print(f"WARNING: boards.toml says chip = \"{b['chip']}\", esptool found {chip}")
    else:
        print("New board, add to boards.toml (pick a name):\n")
        print(f"[boards.NAME]\nmac  = \"{mac}\"\nchip = \"{chip or '?'}\"\nrole = \"COORDINATOR\"\n"
              f"port = \"{stable_port(args.port)}\"")


def cmd_build(args, boards):
    picked, _ = select(boards, args.boards)
    build_boards(picked, args)


def cmd_deploy(args, boards):
    picked, strict = select(boards, args.boards)
    use = present(picked, strict, args.port)     # fails before building when a named board is not there
    build_boards(use, args)
    for b in use:
        flash(b, args)
    if args.monitor:
        cmd_monitor(args, boards)


def cmd_update(args, boards):
    """Install the core and libraries in CORES and LIBRARIES, so each chip in boards.toml can be built."""
    arduino_data_dir()
    urls = list(dict.fromkeys(c["index"] for c in CORES.values()))
    base = [x for u in urls for x in ("--additional-urls", u)]
    steps = [base + ["update"]]
    steps += [base + ["core", "install", f"{name}@{c['version']}"] for name, c in CORES.items()]
    steps += [["lib", "update-index"], ["lib", "install", *LIBRARIES]]
    for step in steps:
        if subprocess.run(["arduino-cli"] + step).returncode:
            die("arduino-cli " + " ".join(step) + " failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="serial ports, and the board on each")
    p = sub.add_parser("identify", help="chip and MAC of the board on a port")
    p.add_argument("port")
    sub.add_parser("update", help="install the arduino core and libraries")
    for name, hlp in (("build", "compile"), ("flash", "flash the last build"),
                      ("deploy", "build + flash"), ("monitor", "serial output")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("boards", nargs="+", help="board name, 'coordinator' (the role) or 'all'")
        p.add_argument("--port", help="use this port instead of finding the board (one board only)")
        if name in ("build", "deploy"):
            p.add_argument("--flags", action="append",
                           help="extra compiler flags, written as --flags='-DX -UY' (also $EXTRA_FLAGS)")
            p.add_argument("-v", "--verbose", action="store_true", help="verbose arduino-cli compile")
        if name == "deploy":
            p.add_argument("--monitor", action="store_true", help="monitor the boards afterwards")
    args = ap.parse_args()
    boards = load_boards()
    {"list": cmd_list, "identify": cmd_identify, "update": cmd_update, "build": cmd_build,
     "flash": cmd_flash, "deploy": cmd_deploy, "monitor": cmd_monitor}[args.cmd](args, boards)


if __name__ == "__main__":
    main()
