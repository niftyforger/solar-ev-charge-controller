#!/usr/bin/env python3
"""Dumps the Sungrow WiNet-S battery/grid registers so their meaning can be
confirmed against the Sungrow app before trusting them in firmware.

Exists because register 13021 (battery power) had its sign convention guessed
wrong three times (see CLAUDE.md "Resolved"): the register is magnitude-only on
this firmware - always positive - so no sign convention applied to it alone can
be right in both directions. Direction comes from the Power Flow Status bitfield
at register 13000 instead. This script prints both side by side so a single
observation of a *charging* battery and a single observation of a *discharging*
battery settle it on direct evidence.

Deliberately a from-scratch Modbus TCP client rather than pymodbus: no install
step, and it mirrors src/modbus_tcp_client.cpp exactly (same FC4, same
big-endian-per-register unpacking, same 2s timeout), so what it sees here is
what the firmware can see.

Usage:
    python tools/probe_sungrow.py <inverter-ip>
    python tools/probe_sungrow.py 192.168.1.50 --interval 5 --csv probe.csv
    python tools/probe_sungrow.py 192.168.1.50 --once

The WiNet-S generally accepts only one Modbus TCP client at a time and the
ESP32 holds a persistent connection, so power the ESP32 down while probing if
the connection is refused.
"""
import argparse
import csv
import socket
import struct
import sys
import time
from datetime import datetime

MODBUS_PORT = 502
UNIT_ID = 1
TIMEOUT_S = 2.0
FC_READ_INPUT_REGISTERS = 0x04

# Wire addresses, in the same frame src/grid_source_sungrow_winet.cpp uses (they
# match the 0-based `address:` field in mkaiser/Sungrow-SHx-Inverter-Modbus-
# Home-Assistant, one below the "reg NNNNN" numbers that repo annotates them
# with). 13009 is the one already confirmed against real hardware, so it anchors
# the convention for the rest.
REG_POWER_FLOW_STATUS = 13000  # U16 bitfield
REG_GRID_POWER = 13009         # S32, low word first, positive = exporting
REG_BATTERY_VOLTAGE = 13019    # U16, x0.1 V
REG_BATTERY_CURRENT = 13020    # U16, x0.1 A
REG_BATTERY_POWER = 13021      # magnitude-only on this firmware (the point of this script)
REG_BATTERY_LEVEL = 13022      # U16, x0.1 %
REG_BATTERY_POWER_WIDE = 5213  # S32 "battery power, wide range" - not present on every model

# Power Flow Status bits. Bits 1 and 2 are the ones that matter here; the rest
# are printed because they cross-check the decode for free (e.g. "exporting"
# should agree with the sign of 13009).
FLOW_BITS = (
    (0x0001, "pv_generating"),
    (0x0002, "battery_charging"),
    (0x0004, "battery_discharging"),
    (0x0008, "positive_load_power"),
    (0x0010, "exporting"),
    (0x0020, "importing"),
    (0x0080, "negative_load_power"),
)

FLOW_BIT_BATTERY_CHARGING = 0x0002
FLOW_BIT_BATTERY_DISCHARGING = 0x0004

MODBUS_EXCEPTIONS = {
    0x01: "illegal function",
    0x02: "illegal data address",
    0x03: "illegal data value",
    0x04: "slave device failure",
    0x06: "slave device busy",
}


class ModbusError(Exception):
    pass


class ModbusTcpProbe:
    """Minimal FC4 client mirroring src/modbus_tcp_client.cpp."""

    def __init__(self, host, port=MODBUS_PORT, unit_id=UNIT_ID, timeout=TIMEOUT_S):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.sock = None
        self.transaction_id = 0

    def connect(self):
        if self.sock is not None:
            return
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.settimeout(self.timeout)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _recv_exactly(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ModbusError("connection closed by peer")
            buf += chunk
        return buf

    def read_input_registers(self, start_addr, count):
        """Returns a list of `count` raw 16-bit words. Raises ModbusError."""
        self.connect()
        self.transaction_id = (self.transaction_id + 1) & 0xFFFF
        request = struct.pack(
            ">HHHBBHH",
            self.transaction_id,
            0,  # protocol id
            6,  # length: unit id + PDU
            self.unit_id,
            FC_READ_INPUT_REGISTERS,
            start_addr,
            count,
        )
        try:
            self.sock.sendall(request)
            header = self._recv_exactly(9)
        except (socket.timeout, OSError) as exc:
            self.close()
            raise ModbusError(str(exc) or "timeout")

        txn_id, _, _, unit_id, function_code, byte_count = struct.unpack(">HHHBBB", header)
        if function_code & 0x80:
            # The exception code sits where the byte count would be.
            self.close()
            raise ModbusError(
                "exception 0x%02X (%s)"
                % (byte_count, MODBUS_EXCEPTIONS.get(byte_count, "unknown"))
            )
        if txn_id != self.transaction_id or unit_id != self.unit_id:
            self.close()
            raise ModbusError("mismatched transaction/unit id in response")
        if function_code != FC_READ_INPUT_REGISTERS or byte_count != count * 2:
            self.close()
            raise ModbusError("malformed response header")

        try:
            data = self._recv_exactly(byte_count)
        except (socket.timeout, OSError) as exc:
            self.close()
            raise ModbusError(str(exc) or "timeout")
        return list(struct.unpack(">%dH" % count, data))

    def read_block(self, start_addr, count):
        """Block read, falling back to one-register-at-a-time so a single
        unmapped address in the span can't blank the whole dump. Returns a list
        of `count` entries, each an int or None where that register failed."""
        try:
            return self.read_input_registers(start_addr, count)
        except ModbusError:
            pass
        regs = []
        for offset in range(count):
            try:
                regs.append(self.read_input_registers(start_addr + offset, 1)[0])
            except ModbusError:
                regs.append(None)
        return regs


def to_s16(word):
    return word - 0x10000 if word >= 0x8000 else word


def to_s32(low_word, high_word):
    raw = (high_word << 16) | low_word
    return raw - 0x100000000 if raw >= 0x80000000 else raw


def decode_flags(status_word):
    return [name for mask, name in FLOW_BITS if status_word & mask]


def predicted_battery_w(status_word, power_word):
    """What the fixed firmware would report for outBatteryW: magnitude from
    13021, direction from 13000. Positive = charging, negative = discharging,
    matching grid_data_source.h's convention. Returns (watts, note)."""
    if status_word is None or power_word is None:
        return None, "no reading"
    magnitude = abs(to_s16(power_word))
    charging = bool(status_word & FLOW_BIT_BATTERY_CHARGING)
    discharging = bool(status_word & FLOW_BIT_BATTERY_DISCHARGING)
    if charging and discharging:
        return None, "INDETERMINATE - both direction bits set"
    if charging:
        return float(magnitude), "charging"
    if discharging:
        return -float(magnitude), "discharging"
    if magnitude > 100:
        return None, "INDETERMINATE - no direction bit, but %d W of flow" % magnitude
    return 0.0, "idle"


def fmt_reg(word):
    if word is None:
        return "  --  "
    return "0x%04X" % word


def fmt_scaled(word, scale, unit, signed=False):
    if word is None:
        return "--"
    value = to_s16(word) if signed else word
    return "%.1f %s" % (value * scale, unit)


def poll_once(probe, probe_wide=True):
    """One full sample. Returns a dict of decoded values; raw words are None
    where the read failed."""
    flow = probe.read_block(REG_POWER_FLOW_STATUS, 1)
    grid = probe.read_block(REG_GRID_POWER, 2)
    battery = probe.read_block(REG_BATTERY_VOLTAGE, 4)
    # Models without 5213 answer "illegal data address", which costs a reconnect
    # each time - so once it's known absent, stop asking on every poll.
    wide = probe.read_block(REG_BATTERY_POWER_WIDE, 2) if probe_wide else [None, None]

    status_word = flow[0]
    grid_w = None
    if grid[0] is not None and grid[1] is not None:
        # Negated to the codebase's negative = exporting convention, and
        # low-word-first, both as confirmed on real hardware.
        grid_w = -float(to_s32(grid[0], grid[1]))

    wide_low_first = None
    wide_high_first = None
    if wide[0] is not None and wide[1] is not None:
        wide_low_first = to_s32(wide[0], wide[1])
        wide_high_first = to_s32(wide[1], wide[0])

    predicted_w, predicted_note = predicted_battery_w(status_word, battery[2])

    return {
        "status_word": status_word,
        "flags": decode_flags(status_word) if status_word is not None else [],
        "grid_regs": grid,
        "grid_w": grid_w,
        "battery_regs": battery,
        "wide_regs": wide,
        "wide_low_first": wide_low_first,
        "wide_high_first": wide_high_first,
        "predicted_w": predicted_w,
        "predicted_note": predicted_note,
    }


def print_sample(sample):
    voltage_w, current_w, power_w, level_w = sample["battery_regs"]

    print("  13000 power flow status : %s  [%s]"
          % (fmt_reg(sample["status_word"]), ", ".join(sample["flags"]) or "none set"))

    grid_str = "--" if sample["grid_w"] is None else "%.0f W (%s)" % (
        abs(sample["grid_w"]), "exporting" if sample["grid_w"] <= 0 else "importing")
    print("  13009 grid power        : %s %s -> %s"
          % (fmt_reg(sample["grid_regs"][0]), fmt_reg(sample["grid_regs"][1]), grid_str))

    print("  13019 battery voltage   : %s -> %s"
          % (fmt_reg(voltage_w), fmt_scaled(voltage_w, 0.1, "V")))
    print("  13020 battery current   : %s -> %s unsigned / %s signed"
          % (fmt_reg(current_w), fmt_scaled(current_w, 0.1, "A"),
             fmt_scaled(current_w, 0.1, "A", signed=True)))
    print("  13021 battery power     : %s -> %s W unsigned / %s W signed   <-- KEY"
          % (fmt_reg(power_w),
             "--" if power_w is None else power_w,
             "--" if power_w is None else to_s16(power_w)))
    print("  13022 battery level     : %s -> %s"
          % (fmt_reg(level_w), fmt_scaled(level_w, 0.1, "%")))

    if sample["wide_low_first"] is None:
        print("  5213  battery power wide: unavailable on this model (expected on some)")
    else:
        print("  5213  battery power wide: %s %s -> %d W low-word-first / %d W high-word-first"
              % (fmt_reg(sample["wide_regs"][0]), fmt_reg(sample["wide_regs"][1]),
                 sample["wide_low_first"], sample["wide_high_first"]))

    predicted = sample["predicted_w"]
    print("  => firmware would report: %s (%s)"
          % ("no value" if predicted is None else "%+.0f W" % predicted,
             sample["predicted_note"]))


CSV_COLUMNS = (
    "timestamp",
    "flow_status_hex",
    "flags",
    "grid_w",
    "battery_voltage_v",
    "battery_current_raw",
    "battery_power_raw_u16",
    "battery_power_raw_s16",
    "battery_level_pct",
    "battery_power_wide_low_first",
    "predicted_battery_w",
    "predicted_note",
)


def csv_row(sample):
    voltage_w, current_w, power_w, level_w = sample["battery_regs"]
    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "flow_status_hex": fmt_reg(sample["status_word"]).strip(),
        "flags": "|".join(sample["flags"]),
        "grid_w": "" if sample["grid_w"] is None else "%.0f" % sample["grid_w"],
        "battery_voltage_v": "" if voltage_w is None else "%.1f" % (voltage_w * 0.1),
        "battery_current_raw": "" if current_w is None else current_w,
        "battery_power_raw_u16": "" if power_w is None else power_w,
        "battery_power_raw_s16": "" if power_w is None else to_s16(power_w),
        "battery_level_pct": "" if level_w is None else "%.1f" % (level_w * 0.1),
        "battery_power_wide_low_first": "" if sample["wide_low_first"] is None
            else sample["wide_low_first"],
        "predicted_battery_w": "" if sample["predicted_w"] is None
            else "%.0f" % sample["predicted_w"],
        "predicted_note": sample["predicted_note"],
    }


def open_csv(path):
    """Opens `path` for append, writing a header row only if it's new/empty.
    Returns (file, DictWriter), or (None, None) when no path was given."""
    if not path:
        return None, None
    need_header = True
    try:
        with open(path, "r") as existing:
            need_header = not existing.read(1)
    except OSError:
        pass
    csv_file = open(path, "a", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
    if need_header:
        csv_writer.writeheader()
    return csv_file, csv_writer


def main():
    parser = argparse.ArgumentParser(
        description="Dump Sungrow WiNet-S battery/grid registers for direction confirmation.")
    parser.add_argument("host", help="inverter/WiNet-S IP address")
    parser.add_argument("--interval", type=float, default=5.0,
                        help="seconds between polls (default: 5)")
    parser.add_argument("--once", action="store_true", help="poll once and exit")
    parser.add_argument("--csv", metavar="PATH",
                        help="also append each sample to this CSV file")
    args = parser.parse_args()

    probe = ModbusTcpProbe(args.host)
    probe_wide = True

    print("Polling %s:%d (unit %d), FC4, %.1fs timeout." % (
        probe.host, probe.port, probe.unit_id, probe.timeout))
    print("Watch 13021 across a battery CHARGE and a battery DISCHARGE:")
    print("  - if it stays positive in both, it is magnitude-only and 13000's")
    print("    bits are the only direction source (the expected result).")
    print("  - cross-check every sample against the Sungrow app.")
    print("Ctrl-C to stop.\n")

    csv_file, csv_writer = open_csv(args.csv)

    exit_code = 0
    try:
        while True:
            print("[%s]" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            try:
                sample = poll_once(probe, probe_wide)
            except ModbusError as exc:
                print("  read failed: %s" % exc)
                exit_code = 1
            except OSError as exc:
                print("  connection failed: %s" % exc)
                print("  (the WiNet-S allows only one Modbus TCP client at a time -")
                print("   power the ESP32 down while probing)")
                exit_code = 1
            else:
                exit_code = 0
                if sample["wide_low_first"] is None:
                    probe_wide = False
                print_sample(sample)
                if csv_writer is not None:
                    csv_writer.writerow(csv_row(sample))
                    csv_file.flush()
            print("")
            if args.once:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("stopped.")
    finally:
        probe.close()
        if csv_file is not None:
            csv_file.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
