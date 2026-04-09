import time

import numpy as np

from serial_utils import read_exact, readline_str, wait_for_line, write_float_line, write_line


def vsq_linspace(v_start, v_stop, count):
    if int(count) < 2:
        raise ValueError("count must be >= 2")
    values = np.linspace(float(v_start) ** 2, float(v_stop) ** 2, int(count))
    return np.sqrt(np.clip(values, 0.0, None))


class TxController:
    def __init__(self, ser, config):
        self.ser = ser
        self.config = config
        self.channels = {item["name"]: dict(item) for item in config.tx_channels()}
        self.channel_order = [item["name"] for item in config.tx_channels()]
        self.gui_ranges = config.tx_gui_ranges()

    def refresh_from_config(self):
        self.config.reload()
        self.channels = {item["name"]: dict(item) for item in self.config.tx_channels()}
        self.channel_order = [item["name"] for item in self.config.tx_channels()]
        self.gui_ranges = self.config.tx_gui_ranges()

    def sync_board_parameters(self):
        desired = self.config.tx_board_params()
        existing = self.list_board_parameters()
        for key in existing:
            if key not in desired:
                self.delete_board_parameter(key)
        for key, value in desired.items():
            self.set_board_parameter(key, value)

    def set_board_parameter(self, key, value):
        write_line(self.ser, "SET_PARAM")
        write_line(self.ser, f"{key} {value}")
        wait_for_line(self.ser, "Done")

    def delete_board_parameter(self, key):
        write_line(self.ser, "DELETE_PARAM")
        write_line(self.ser, key)
        wait_for_line(self.ser, "Done")

    def list_board_parameters(self):
        params = {}
        write_line(self.ser, "LIST_PARAMS")
        wait_for_line(self.ser, "BEGIN_PARAMS")
        while True:
            line = wait_for_line(self.ser, timeout_s=10.0)
            if line == "END_PARAMS":
                return params
            if "=" in line:
                key, value = line.split("=", 1)
                params[key] = float(value)

    def push_all_voltages(self):
        write_line(self.ser, "UPDATE_VOLTAGES")
        wait_for_line(self.ser, "ACK")
        for channel in self.config.tx_channels():
            write_line(self.ser, f"{channel['chip']} {channel['pin']} {channel['voltage']}")
        write_line(self.ser, "END")
        wait_for_line(self.ser, "Done.")

    def set_levels(self, low, high):
        self.config.set_board_param("tx", "tx_mux_low", float(low))
        self.config.set_board_param("tx", "tx_mux_high", float(high))
        write_line(self.ser, "SET_TX_LEVELS")
        write_float_line(self.ser, low)
        write_float_line(self.ser, high)
        wait_for_line(self.ser, "Done")

    def update_channels(self, updates, persist=True):
        if not updates:
            return []

        write_line(self.ser, "UPDATE_VOLTAGES")
        wait_for_line(self.ser, "ACK")
        for name, channel in updates.items():
            write_line(self.ser, f"{channel['chip']} {channel['pin']} {channel['voltage']}")
        write_line(self.ser, "END")
        wait_for_line(self.ser, "Done.")

        changed = []
        for name, channel in updates.items():
            self.channels[name]["voltage"] = float(channel["voltage"])
            changed.append(name)
            if persist:
                self.config.update_tx_voltage(name, channel["voltage"])
        return changed

    def diff_channel_updates(self, gui_values, eps=1e-3):
        updates = {}
        for name in self.channel_order:
            new_voltage = float(gui_values[name])
            current = self.channels[name]
            if abs(new_voltage - float(current["voltage"])) > eps:
                updates[name] = {
                    "chip": current["chip"],
                    "pin": current["pin"],
                    "voltage": new_voltage,
                }
        return updates

    def run_scan_2v(self, name1, name2, s1, e1, n1, s2, e2, n2, pd1, pd2, timeout_s=60.0):
        ch1 = self.channels[name1]
        ch2 = self.channels[name2]

        write_line(self.ser, "SCAN_2V")
        wait_for_line(self.ser, "ACK")
        write_line(self.ser, f"{ch1['chip']} {ch1['pin']} {ch1['voltage']} {s1} {e1} {n1}")
        write_line(self.ser, f"{ch2['chip']} {ch2['pin']} {ch2['voltage']} {s2} {e2} {n2}")
        write_line(self.ser, f"{pd1} {pd2}")
        write_line(self.ser, "END")
        wait_for_line(self.ser, "ACK")
        wait_for_line(self.ser, "Scan2V Finished", timeout_s=timeout_s)

        begin = wait_for_line(self.ser, timeout_s=10.0)
        while not begin.startswith("BEGIN "):
            begin = wait_for_line(self.ser, timeout_s=10.0)
        _, rn1, rn2 = begin.split()
        rn1 = int(rn1)
        rn2 = int(rn2)
        total = rn1 * rn2

        raw1 = read_exact(self.ser, total * 2, timeout_s=timeout_s)
        raw2 = read_exact(self.ser, total * 2, timeout_s=timeout_s)
        wait_for_line(self.ser, "DONE", timeout_s=10.0)

        pd1_values = np.frombuffer(raw1, dtype="<u2").astype(np.float32).reshape((rn1, rn2)).T
        pd2_values = np.frombuffer(raw2, dtype="<u2").astype(np.float32).reshape((rn1, rn2)).T

        return {
            "name1": name1,
            "name2": name2,
            "n1": rn1,
            "n2": rn2,
            "V1_axis": vsq_linspace(s1, e1, n1),
            "V2_axis": vsq_linspace(s2, e2, n2),
            "PD1": pd1_values,
            "PD2": pd2_values,
            "pd1": pd1,
            "pd2": pd2,
        }

    def send_message(self, message):
        write_line(self.ser, "SEND_MESSAGE_d")
        write_line(self.ser, message)
        while True:
            line = readline_str(self.ser).strip()
            if line:
                if line == "Done":
                    return
