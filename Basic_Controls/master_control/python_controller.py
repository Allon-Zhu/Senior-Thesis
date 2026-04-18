# -*- coding: utf-8 -*-
import time
import serial
import collections
import re
import sys
import numpy as np
import matplotlib.pyplot as plt

from tkinter import *
from tkinter.font import Font
from tkinter import simpledialog, messagebox
from tkinter.scrolledtext import ScrolledText

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed to register 3D)
from matplotlib import cm

from automated_calibration.remote_laser_control import DLCProSimple
LASER_HOST = "192.168.1.200"
Tx_port_num = 4
Rx_port_num = 3

from ring_calibration_helper import tx_scan_frequency
from highQ_piezoscan_helper import tx_scan_piezo


# ---------- Serial helpers ----------
def open_arduino(port, baud=115200, to=0.2):
    ser = serial.Serial(port=port, baudrate=baud, timeout=to)
    # time.sleep(1.0)                # allow auto-reset
    return ser

def readline_str(ser):
    try:
        line = ser.readline().decode('utf-8', errors='ignore')
    except Exception:
        return ''
    return line

def _read_exact(ser, n, timeout_s=30.0):
    """
    Read exactly n bytes from serial. Raises on timeout.
    IMPORTANT: call as _read_exact(self.tx, nbytes)
    """
    t0 = time.time()
    buf = bytearray()
    while len(buf) < n:
        if time.time() - t0 > timeout_s:
            raise TimeoutError(f"Timeout: wanted {n} bytes, got {len(buf)}")
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)

def _parse_scan_row(line: str):
    """
    Parse one scan data line from Arduino.
    Accepts:
      "v1 v2 pd1 pd2"
      or "v1,v2,pd1,pd2"
    Returns tuple (v1, v2, pd1, pd2) or None.
    """
    if not line:
        return None

    s = line.strip()
    if not s or s in ("ACK", "Done.", "Done"):
        return None
    if s.startswith("ERR"):
        raise RuntimeError(s)

    if "," in s:
        parts = [p.strip() for p in s.split(",")]
    else:
        parts = s.split()

    if len(parts) < 4:
        return None

    try:
        return tuple(float(x) for x in parts[:4])
    except ValueError:
        return None

def write_line(ser, s):
    if not s.endswith('\n'):
        s += '\n'
    ser.write(s.encode('utf-8'))

def write_float_line(ser, x):
    ser.write(f"{float(x)}\r\n".encode('utf-8'))
    # small pacing to avoid sticking floats together on very short timeouts
    time.sleep(0.02)

def vsq_linspace(v_start: float, v_stop: float, N: int) -> np.ndarray:
    """
    Return N voltages between v_start and v_stop such that V^2 is uniformly spaced.
    (Spacing is constant in V^2, not in V.)
    """
    v_start = float(v_start)
    v_stop = float(v_stop)
    N = int(N)
    if N < 2:
        raise ValueError("N must be >= 2")

    u0 = v_start * v_start
    u1 = v_stop * v_stop
    u = np.linspace(u0, u1, N)           # uniform in V^2
    v = np.sqrt(np.clip(u, 0.0, None))   # back to V
    return v

class Scan2VDialog:
    """
    Dialog to configure a 2-D voltage scan.
    """
    def __init__(self, parent, names):
        self.top = Toplevel(parent)
        self.top.title("Scan 2V")
        self.top.transient(parent)
        self.top.grab_set()

        self.result = None

        # defaults
        self.v1 = StringVar(value=names[0] if names else "")
        self.v2 = StringVar(value=names[1] if len(names) > 1 else (names[0] if names else ""))

        self.a1s = StringVar(value="5.0")
        self.a1e = StringVar(value="20.0")
        self.a1n = StringVar(value="21")

        self.a2s = StringVar(value="5.0")
        self.a2e = StringVar(value="20.0")
        self.a2n = StringVar(value="21")

        self.pd1 = StringVar(value="3")
        self.pd2 = StringVar(value="4")

        f = Frame(self.top)
        f.pack(padx=12, pady=12)

        # helper: label + widget row
        def add_row(r, label_text, widget):
            Label(f, text=label_text).grid(row=r, column=0, sticky="e", padx=6, pady=4)
            widget.grid(row=r, column=1, sticky="w", padx=6, pady=4)

        # voltage selectors
        add_row(0, "Voltage #1", OptionMenu(f, self.v1, *names))
        add_row(1, "Voltage #2", OptionMenu(f, self.v2, *names))

        # axis1 fields
        axis1_frame = Frame(f)
        Entry(axis1_frame, width=10, textvariable=self.a1s).pack(side=LEFT, padx=(0, 6))
        Entry(axis1_frame, width=10, textvariable=self.a1e).pack(side=LEFT, padx=(0, 6))
        Entry(axis1_frame, width=6,  textvariable=self.a1n).pack(side=LEFT)
        add_row(2, "Axis1 start / stop / N", axis1_frame)

        # axis2 fields
        axis2_frame = Frame(f)
        Entry(axis2_frame, width=10, textvariable=self.a2s).pack(side=LEFT, padx=(0, 6))
        Entry(axis2_frame, width=10, textvariable=self.a2e).pack(side=LEFT, padx=(0, 6))
        Entry(axis2_frame, width=6,  textvariable=self.a2n).pack(side=LEFT)
        add_row(3, "Axis2 start / stop / N", axis2_frame)

        # PD indices
        pd_frame = Frame(f)
        Entry(pd_frame, width=6, textvariable=self.pd1).pack(side=LEFT, padx=(0, 8))
        Entry(pd_frame, width=6, textvariable=self.pd2).pack(side=LEFT)
        add_row(4, "PD indices (1–4)", pd_frame)

        # buttons
        btn = Frame(f)
        btn.grid(row=5, column=0, columnspan=2, pady=(10, 0), sticky="e")
        Button(btn, text="Cancel", command=self._cancel).pack(side=RIGHT, padx=6)
        Button(btn, text="Start", command=self._ok).pack(side=RIGHT)

        # nicer sizing
        f.grid_columnconfigure(1, weight=1)

    def _cancel(self):
        self.top.destroy()

    def _ok(self):
        try:
            v1 = self.v1.get().strip()
            v2 = self.v2.get().strip()
            a1s = float(self.a1s.get())
            a1e = float(self.a1e.get())
            a1n = int(self.a1n.get())
            a2s = float(self.a2s.get())
            a2e = float(self.a2e.get())
            a2n = int(self.a2n.get())
            pd1 = int(self.pd1.get())
            pd2 = int(self.pd2.get())

            if a1n < 2 or a2n < 2:
                raise ValueError("N must be >= 2 for both axes.")
            if not (1 <= pd1 <= 4 and 1 <= pd2 <= 4):
                raise ValueError("PD indices must be in 1..4.")

            self.result = (v1, v2, a1s, a1e, a1n, a2s, a2e, a2n, pd1, pd2)
        except Exception as e:
            messagebox.showerror("Invalid input", str(e), parent=self.top)
            return

        self.top.destroy()

# ---------- App ----------
class App:
    def __init__(self):
        # Open both boards
        self.tx = open_arduino('COM' + str(Tx_port_num), 115200, 0.2)
        while True:
            line = readline_str(self.tx)
            if line.strip() == "TX Ready.":
                break
        print("Connected to TX")
        self.rx = open_arduino('COM' + str(Rx_port_num), 115200, 0.2)
        while True:
            line = readline_str(self.rx)
            if line.strip() == "RX Ready.":
                break
        print("Connected to RX")

        try:
            self.laser = DLCProSimple(LASER_HOST).open()
            print(f"Connected to laser at {LASER_HOST}")
        except Exception as e:
            self.laser = None
            print("ERROR: Could not connect to laser:", e)

        # Main root
        self.root = Tk()
        self.root.title("Hybrid TX/RX Control")
        self.root.withdraw()

        # Build two panels as Toplevels so they can be moved independently
        self.build_tx_panel()
        self.build_rx_panel()

        self.root.protocol("WM_DELETE_WINDOW", self._quit_all)
        self.tx_win.protocol("WM_DELETE_WINDOW", self._quit_all)
        self.rx_win.protocol("WM_DELETE_WINDOW", self._quit_all)

    # ---------------- TX Panel ----------------
    def build_tx_panel(self):
        V_max, V_inc = 25.0, 0.05
        ranges = {'PD_DWDM': (1.0, 4.0, 1.0), 'RT_DWDM': (0.0, 1.0, 0.01)}

        # For 1550.12 pump
        self.tx_dac_map = {
            'CQS': (1, 2, 15.0),

            'QS1': (1, 12, 0.0),
            'QS2top': (0, 5, 0.0),
            'QS2bot': (0, 15, 0.0),
            'IntSec': (0, 2, 0.0),

            'RTRtop': (0, 12, 0.0),
            'RTRbot': (1, 13, 0.0),

            'QF11': (0, 13, 0.0),
            'QF12': (0, 14, 0.0),
            'QF21': (0, 1, 0.0),
            'QF22': (0, 0, 0.0),
            'QF31': (0, 3, 0.0),
            'QF32': (0, 4, 0.0),
            'QF41': (0, 7, 0.0),
            'QF42': (0, 6, 0.0),

            'MUXa': (1, 15, 0.0),
            'PCa1': (1, 1, 0.0),
            'PCa2': (1, 4, 0.0),
            'PCa3': (1, 6, 0.0),

            'MUXb': (1, 14, 0.0),
            'PCb1': (1, 0, 0.0),
            'PCb2': (1, 3, 0.0),
            'PCb3': (1, 5, 0.0),

            'MUXc': (0, 11, 0.0),
            'PCc1': (0, 9, 0.0),
            'PCc2': (1, 11, 0.0),
            'PCc3': (1, 9, 0.0),

            'MUXd': (0, 10, 0.0),
            'PCd1': (0, 8, 0.0),
            'PCd2': (1, 10, 0.0),
            'PCd3': (1, 8, 0.0),
        }

        # # For Turn off
        # self.tx_dac_map = {
        #     'CQS': (0, 3, 0.0),
        # }

        self.tx_write_order = list(self.tx_dac_map.keys())

        self.tx_win = Toplevel(self.root)
        self.tx_win.title("TX Control Panel")
        frame = Frame(self.tx_win); frame.pack(padx=40, pady=30)

        self.tx_log = ScrolledText(self.tx_win, height=8, width=90, font=('Consolas', 10))
        self.tx_log.pack(fill='both', expand=False, padx=10, pady=(0,10))
        self.tx_log_print = lambda s: (self.tx_log.insert(END, s + "\n"), self.tx_log.see(END))

        self.voltage_vars = {}
        for name, (chip_id, pin, volt) in self.tx_dac_map.items():
            self.voltage_vars[name] = StringVar(value=str(float(volt)))

        gui_order = self.tx_write_order
        for idx, name in enumerate(gui_order):
            r = idx // 4
            c = (idx % 4) * 2
            Label(frame, text=f"  {name}  ", font=Font(family='Arial', size=16)).grid(row=r, column=c, sticky='e')
            lo, hi, inc = ranges.get(name, (0.0, V_max, V_inc))
            Spinbox(frame, from_=lo, to=hi, increment=inc, width=8,
                    textvariable=self.voltage_vars[name],
                    font=Font(family='Arial', size=18)).grid(row=r, column=c+1, padx=(0,15), pady=5, sticky='w')

        # buttons
        # Button(self.tx_win, text="Pulse Test",  command=self.tx_pulse_test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        # Button(self.tx_win, text="Teensy Test", command=self.tx_teensy_test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Set TX Voltages", command=self.tx_set_levels, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Pulse Test", command=self.tx_pulse_test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Scan Frequency", command=lambda: tx_scan_frequency(self), font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Scan Piezo", command=lambda: tx_scan_piezo(self), font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)

        Button(self.tx_win, text="Scan 2V",
               command=self.tx_scan_2v,
               font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)

        Button(self.tx_win, text="Update", command=self.tx_update_changed_from_gui, font=('Arial', 16)).pack(side=RIGHT, padx=10, pady=10)
        Button(self.tx_win, text="Quit", command=self._quit_all, font=('Arial', 16)).pack(side=RIGHT, padx=10, pady=10)

        # push defaults once
        self.tx_init_from_dac_map()
        self.tx_log_print("[TX] Defaults sent.")

    def tx_send_voltages(self, dac_map_updates: dict):
        """
        dac_map_updates: dict like tx_dac_map
          { name: (chip_id, pin, volt), ... }

        Arduino protocol (unchanged):
          UPDATE_VOLTAGES -> wait ACK -> send "<chip> <pin> <volt>" lines -> END -> wait Done.
        """
        write_line(self.tx, "UPDATE_VOLTAGES")

        # wait ACK
        while True:
            out = readline_str(self.tx)
            if out and out.strip() == "ACK":
                break

        # send updates
        for name, (chip_id, pin, volt) in dac_map_updates.items():
            write_line(self.tx, f"{int(chip_id)} {int(pin)} {float(volt)}")

        write_line(self.tx, "END")

        # wait Done.
        while True:
            out = readline_str(self.tx)
            if out and out.strip() == "Done.":
                break

    def tx_init_from_dac_map(self):
        """
        Initialize TX by sending ALL channels using the *latest* UPDATE_VOLTAGES method,
        with voltages pulled from self.tx_dac_map (name -> (chip_id, pin, volt)).

        Arduino protocol (unchanged):
          UPDATE_VOLTAGES -> wait ACK -> "<chip> <pin> <volt>"... -> END -> wait Done.
        """
        if not hasattr(self, "tx_dac_map"):
            messagebox.showerror("Error", "tx_dac_map not found.")
            return

        # Use the map directly as the payload for tx_send_voltages()
        updates = {name: (chip, pin, float(v)) for name, (chip, pin, v) in self.tx_dac_map.items()}

        try:
            self.tx_send_voltages(updates)
        except Exception as e:
            if hasattr(self, "tx_log_print"):
                self.tx_log_print(f"[TX] Init-from-map failed: {e}")
            messagebox.showerror("TX init failed", str(e))
            return

        if hasattr(self, "tx_log_print"):
            self.tx_log_print(f"[TX] Initialized from tx_dac_map ({len(updates)} channels).")

    def tx_update_changed_from_gui(self, eps: float = 1e-3):
        """
        One-shot:
          - read GUI voltages
          - diff against self.tx_dac_map (name -> (chip,pin,volt))
          - send only changed channels via self.tx_send_voltage_updates(updates)
          - update self.tx_dac_map with the new voltages after success

        eps: tolerance to ignore tiny float changes from GUI formatting.
        """
        if not hasattr(self, "tx_dac_map"):
            messagebox.showerror("Error", "tx_dac_map not found (master DAC map missing).")
            return
        if not hasattr(self, "tx_write_order"):
            messagebox.showerror("Error", "tx_write_order not found.")
            return

        updates = {}

        # 1) Collect diffs from GUI
        try:
            for name in self.tx_write_order:
                if name not in self.voltage_vars:
                    continue
                if name not in self.tx_dac_map:
                    continue

                new_v = float(self.voltage_vars[name].get())
                chip_id, pin, old_v = self.tx_dac_map[name]

                if abs(new_v - float(old_v)) > eps:
                    updates[name] = (chip_id, pin, float(new_v))

        except ValueError as e:
            messagebox.showerror("Invalid input", f"Bad numeric value: {e}")
            return
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        # 2) If nothing changed, done
        if not updates:
            if hasattr(self, "tx_log_print"):
                self.tx_log_print("[TX] No voltage changes.")
            return

        # 3) Send to Arduino (your existing method)
        try:
            self.tx_send_voltages(updates)
        except Exception as e:
            # Don't mutate tx_dac_map if send failed
            if hasattr(self, "tx_log_print"):
                self.tx_log_print(f"[TX] Update failed: {e}")
            messagebox.showerror("TX update failed", str(e))
            return

        # 4) Update master map after success
        for name, (chip_id, pin, new_v) in updates.items():
            old_chip, old_pin, _old_v = self.tx_dac_map[name]
            # optional sanity check
            if int(old_chip) != int(chip_id) or int(old_pin) != int(pin):
                raise ValueError(
                    f"{name}: chip/pin mismatch. master=({old_chip},{old_pin}) update=({chip_id},{pin})"
                )
            self.tx_dac_map[name] = (old_chip, old_pin, float(new_v))

        # 5) Log
        if hasattr(self, "tx_log_print"):
            self.tx_log_print(
                f"[TX] Updated {len(updates)} channel(s): {', '.join(updates.keys())}"
            )

    def _tx_run_scan_2v(self,
                        name1, name2,
                        s1, e1, n1,
                        s2, e2, n2,
                        pd1, pd2,
                        timeout_s=60.0):

        def _read_exact(nbytes, timeout=timeout_s):
            """Read exactly nbytes from self.tx or raise TimeoutError."""
            t0 = time.time()
            buf = bytearray()
            while len(buf) < nbytes:
                if time.time() - t0 > timeout:
                    raise TimeoutError(f"Timeout while reading {nbytes} bytes (got {len(buf)})")
                chunk = self.tx.read(nbytes - len(buf))
                if chunk:
                    buf.extend(chunk)
            return bytes(buf)

        def _read_begin(timeout=10.0):
            """Read lines until BEGIN n1 n2 is found, else raise."""
            t0 = time.time()
            while True:
                if time.time() - t0 > timeout:
                    raise TimeoutError("Timeout waiting for BEGIN")
                line = readline_str(self.tx)
                if not line:
                    continue
                s = line.strip()
                if not s:
                    continue
                if s.startswith("ERR"):
                    raise RuntimeError(s)
                if s.startswith("BEGIN"):
                    parts = s.split()
                    if len(parts) != 3:
                        raise RuntimeError(f"Malformed BEGIN line: {s}")
                    return int(parts[1]), int(parts[2])

        def _read_done(timeout=10.0):
            """Read lines until DONE is found, else raise."""
            t0 = time.time()
            while True:
                if time.time() - t0 > timeout:
                    raise TimeoutError("Timeout waiting for DONE")
                line = readline_str(self.tx)
                if not line:
                    continue
                s = line.strip()
                if not s:
                    continue
                if s.startswith("ERR"):
                    raise RuntimeError(s)
                if s == "DONE":
                    return

        c1, p1, v1_init = self.tx_dac_map[name1]
        c2, p2, v2_init = self.tx_dac_map[name2]

        write_line(self.tx, "SCAN_2V")

        while True:
            if readline_str(self.tx).strip() == "ACK":
                break

        write_line(self.tx, f"{c1} {p1} {v1_init} {s1} {e1} {n1}")
        write_line(self.tx, f"{c2} {p2} {v2_init} {s2} {e2} {n2}")
        write_line(self.tx, f"{pd1} {pd2}")
        write_line(self.tx, "END")

        while True:
            if readline_str(self.tx).strip() == "ACK":
                break

        while True:
            if readline_str(self.tx).strip() == "Scan2V Finished":
                break

        rn1, rn2 = _read_begin(timeout=10.0)

        # Use returned sizes as truth
        N = rn1 * rn2
        nbytes = N * 2  # uint16 => 2 bytes each

        raw1 = _read_exact(nbytes, timeout=timeout_s)
        raw2 = _read_exact(nbytes, timeout=timeout_s)

        _read_done(timeout=10.0)

        PD1 = np.frombuffer(raw1, dtype='<u2').astype(np.float32)
        PD2 = np.frombuffer(raw2, dtype='<u2').astype(np.float32)

        PD1 = PD1.reshape((rn1, rn2)).T
        PD2 = PD2.reshape((rn1, rn2)).T
        V1_axis = vsq_linspace(s1, e1, n1)
        V2_axis = vsq_linspace(s2, e2, n2)

        return dict(
            name1=name1, name2=name2,
            n1=rn1, n2=rn2,
            V1_axis=V1_axis,  # <--- add
            V2_axis=V2_axis,  # <--- add
            PD1=PD1, PD2=PD2,
            pd1=pd1, pd2=pd2
        )

    def tx_scan_2v(self):
        dlg = Scan2VDialog(self.tx_win, list(self.tx_dac_map.keys()))
        self.tx_win.wait_window(dlg.top)
        if dlg.result is None:
            return

        data = self._tx_run_scan_2v(*dlg.result)
        self._tx_save_scan_2v(data)
        self._tx_plot_scan_2v(data)

    def tx_pulse_test(self):
        msg = simpledialog.askstring("Pulse Test", "Enter message to send:", parent=self.tx_win)
        if msg is None: return
        write_line(self.tx, "SEND_MESSAGE_d")
        write_line(self.tx, msg)  # newline-terminated
        self.tx_log_print(f"[TX] SEND_MESSAGE '{msg}'")
        while True:
            resp = readline_str(self.tx)
            if not resp: self.tx_win.update_idletasks(); self.tx_win.update(); continue
            self.tx_log_print("[TX<-] " + resp.strip())
            if resp.strip() == "Done": break

    def tx_teensy_test(self):
        mode = simpledialog.askstring("Teensy Test", "Enter 0 or 1:", parent=self.tx_win)
        if mode not in ("0","1"): return
        write_line(self.tx, "TEST_TEENSY")
        self.tx.write(mode.encode('utf-8'))
        self.tx.write(b'\n')
        while True:
            resp = readline_str(self.tx)
            if not resp: self.tx_win.update_idletasks(); self.tx_win.update(); continue
            self.tx_log_print("[TX<-] " + resp.strip())
            if resp.strip() == "Done": break

    def tx_set_levels(self):
        vl = simpledialog.askfloat("Set TX Levels", "VL (V):", minvalue=0.0, maxvalue=30.0, parent=self.tx_win)
        if vl is None: return
        vh = simpledialog.askfloat("Set TX Levels", "VH (V):", minvalue=0.0, maxvalue=30.0, parent=self.tx_win)
        if vh is None: return
        write_line(self.tx, "SET_TX_LEVELS")
        write_float_line(self.tx, vl)
        write_float_line(self.tx, vh)
        while True:
            resp = readline_str(self.tx)
            if not resp: self.tx_win.update_idletasks(); self.tx_win.update(); continue
            self.tx_log_print("[TX<-] " + resp.strip())
            if resp.strip() == "Done": break

    # ---------------- RX Panel ----------------
    def build_rx_panel(self):
        self.rx_win = Toplevel(self.root)
        self.rx_win.title("RX Control Panel")

        ctrl = Frame(self.rx_win)
        ctrl.pack(padx=12, pady=10, fill='x')

        Button(ctrl, text="Start Stream", command=self.rx_stream_start, font=('Arial', 14)).pack(side=LEFT, padx=6)
        Button(ctrl, text="Stop Stream", command=self.rx_stream_stop, font=('Arial', 14)).pack(side=LEFT, padx=6)
        Button(ctrl, text="Decode Mode (blocking)", command=self.rx_decode_mode, font=('Arial', 14)).pack(side=LEFT,
                                                                                                          padx=6)
        Button(ctrl, text="Exit Decode (Reconnect)", command=self.rx_reconnect, font=('Arial', 14)).pack(side=LEFT,
                                                                                                         padx=6)

        # --- Plot area above the log ---
        plot_frame = Frame(self.rx_win)
        plot_frame.pack(padx=10, pady=(0, 10), fill='both', expand=True)

        self.rx_fig = Figure(figsize=(5, 7), dpi=100)
        self.rx_ax = self.rx_fig.add_subplot(111)
        # self.rx_ax.set_title("RX values (last 60 s)")
        self.rx_ax.set_xlabel("Time (s)")
        self.rx_ax.set_ylabel("Value")

        # line object, initially empty
        self.rx_lines = []
        for i in range(5):
            line, = self.rx_ax.plot([], [], label=f"Ch {i + 1}")
            self.rx_lines.append(line)

        self.rx_canvas = FigureCanvasTkAgg(self.rx_fig, master=plot_frame)
        self.rx_canvas.get_tk_widget().pack(fill='both', expand=True)

        # data buffer: (timestamp, value)
        import collections
        self.rx_history = collections.deque()  # each entry: (timestamp, [v1, v2, v3, v4])

        # poll RX (non-blocking) to show stream lines
        self.rx_log = ScrolledText(self.rx_win, height=8, width=100, font=('Consolas', 10))
        self.rx_log.pack(fill='both', expand=True, padx=10, pady=10)
        self.rx_log_print = lambda s: (self.rx_log.insert(END, s + "\n"), self.rx_log.see(END))

        self.rx_polling = True
        self.rx_poll()

    def rx_poll(self):
        if self.rx_polling:
            new_data = False
            try:
                while self.rx.in_waiting:
                    line = self.rx.readline().decode('utf-8', errors='ignore')
                    if line:
                        self.rx_log_print(line.rstrip())

                        # --- NEW: parse 4 values ---
                        vals = self.rx_parse_values(line)
                        if vals is not None:
                            self.rx_add_sample(vals)
                            new_data = True
            except Exception:
                pass

            if new_data:
                self.rx_update_plot()

            self.rx_win.after(20, self.rx_poll)

    def rx_stream_start(self):
        self.rx_log_print("[RX] STREAM_START")
        self.rx.write(b"STREAM_START\n")

    def rx_stream_stop(self):
        self.rx_log_print("[RX] STREAM_STOP")
        self.rx.write(b"STREAM_STOP\n")

    def rx_decode_mode(self):
        self.rx_log_print("[RX] DECODE_MODE (Arduino will block; reconnect to exit)")
        self.rx.write(b"DECODE_MODE\n")
        # Do NOT wait here; Arduino is now fully dedicated to decoding.
        # Output (decoded messages) will still appear via rx_poll().

    def rx_reconnect(self):
        # Close & reopen COM3 to reset Arduino and exit decode mode
        try:
            self.rx.close()
        except Exception:
            pass
        time.sleep(0.5)
        self.rx = open_arduino('COM' + str(Rx_port_num), 115200, 0.2)
        # Wait for Ready.
        while True:
            line = self.rx.readline().decode('utf-8', errors='ignore')
            if line.strip() == "RX Ready.":
                break
        self.rx_log_print("[RX] Reconnected to COM3 (Ready.)")

    # -------------- Plot Functions --------------
    def rx_parse_values(self, line: str):
        """
        Expect lines like: ' v0  v1  v2  v3  v4'
        Returns [v0, v1, v2, v3, v4] as floats or None on failure.
        """
        parts = line.strip().split()
        if len(parts) < 5:
            return None
        try:
            return [float(p) for p in parts[:5]]
        except ValueError:
            return None

    def rx_add_sample(self, values):
        """
        values: list/tuple of 4 floats
        Keep only last 60 seconds.
        """
        now = time.time()
        self.rx_history.append((now, values))

        cutoff = now - 60.0
        while self.rx_history and self.rx_history[0][0] < cutoff:
            self.rx_history.popleft()

    def rx_update_plot(self):
        if not self.rx_history:
            return

        t0 = self.rx_history[0][0]
        xs = [t - t0 for (t, vals) in self.rx_history]

        # Update each channel
        for idx in range(5):
            ys = [vals[idx] for (t, vals) in self.rx_history]
            self.rx_lines[idx].set_data(xs, ys)

        # X axis: show up to last 60s; don't shrink under 60
        self.rx_ax.set_xlim(0, max(60.0, xs[-1]))

        # Y axis fixed [0, 409]
        self.rx_ax.set_ylim(0, 4096)

        self.rx_canvas.draw_idle()

    def _tx_plot_scan_2v(self, d):
        Zsum = d["PD1"] + d["PD2"]
        Zdiff = (d["PD1"] - d["PD2"])/Zsum

        v1 = d["V1_axis"]  # length n1
        v2 = d["V2_axis"]  # length n2

        # Make grid with shape (n2, n1) to match PD arrays
        X, Y = np.meshgrid(v1, v2, indexing="xy")  # X/Y both (n2, n1)

        # # ---- Surface 1: sum ----
        # fig1 = plt.figure()
        # ax1 = fig1.add_subplot(111, projection="3d")
        # ax1.set_title(f"PD{d['pd1']} + PD{d['pd2']}")
        # ax1.set_xlabel(d["name1"])
        # ax1.set_ylabel(d["name2"])
        # ax1.set_zlabel("PD sum")
        #
        # surf1 = ax1.plot_surface(X, Y, Zsum, rstride=1, cstride=1, linewidth=0, antialiased=True)
        # fig1.colorbar(surf1, ax=ax1, shrink=0.6, pad=0.08)
        #
        # # ---- Surface 2: normalized diff ----
        # fig2 = plt.figure()
        # ax2 = fig2.add_subplot(111, projection="3d")
        # ax2.set_title(f"(PD{d['pd1']} - PD{d['pd2']}) / (PD{d['pd1']} + PD{d['pd2']})")
        # ax2.set_xlabel(d["name1"])
        # ax2.set_ylabel(d["name2"])
        # ax2.set_zlabel("Normalized diff")
        #
        # surf2 = ax2.plot_surface(X, Y, Zdiff, rstride=1, cstride=1, linewidth=0, antialiased=True)
        # fig2.colorbar(surf2, ax=ax2, shrink=0.6, pad=0.08)
        #
        # plt.show()

        extent = [float(v1[0] ** 2), float(v1[-1] ** 2), float(v2[0] ** 2), float(v2[-1] ** 2)]

        plt.figure()
        plt.title(f"PD{d['pd1']}")
        plt.imshow(d["PD1"], origin="lower", aspect="auto", extent=extent)
        plt.colorbar()
        plt.xlabel(d["name1"])
        plt.ylabel(d["name2"])

        plt.figure()
        plt.title(f"PD{d['pd2']}")
        plt.imshow(d["PD2"], origin="lower", aspect="auto", extent=extent)
        plt.colorbar()
        plt.xlabel(d["name1"])
        plt.ylabel(d["name2"])

        plt.figure()
        plt.title(f"PD{d['pd1']} + PD{d['pd2']}")
        plt.imshow(Zsum, origin="lower", aspect="auto", extent=extent)
        plt.colorbar()
        plt.xlabel(d["name1"])
        plt.ylabel(d["name2"])

        plt.figure()
        plt.title(f"PD{d['pd1']} - PD{d['pd2']}")
        plt.imshow(Zdiff, origin="lower", aspect="auto", extent=extent)
        plt.colorbar()
        plt.xlabel(d["name1"])
        plt.ylabel(d["name2"])

        plt.show()

    # -------------- Save Data Functions --------------
    def _tx_save_scan_2v(self, d, filepath=None):
        """
        Save only PDsum and PDdiff (2D arrays) to a compressed NPZ.
        d must contain: PD1, PD2, V1_axis, V2_axis, pd1, pd2, name1, name2
        """
        import numpy as np
        from tkinter import filedialog

        PD1 = np.asarray(d["PD1"], dtype=np.float64)
        PD2 = np.asarray(d["PD2"], dtype=np.float64)

        Zsum = PD1 + PD2
        Zdiff = (PD1 - PD2) / Zsum

        if filepath is None:
            # default file name
            default_name = f"scan2v_{d['name1']}_{d['name2']}.npz"
            filepath = filedialog.asksaveasfilename(
                parent=self.tx_win,
                title="Save Scan 2V Result",
                defaultextension=".npz",
                initialfile=default_name,
                filetypes=[("NumPy compressed", "*.npz")]
            )
            if not filepath:
                if hasattr(self, "tx_log_print"):
                    self.tx_log_print("[TX] Save cancelled.")
                return

        np.savez_compressed(
            filepath,
            PD1=PD1,
            PD2=PD2,
            PDsum=Zsum,
            PDdiff=Zdiff,
            V1_axis=np.asarray(d["V1_axis"], dtype=np.float64),
            V2_axis=np.asarray(d["V2_axis"], dtype=np.float64),
            pd1=int(d["pd1"]),
            pd2=int(d["pd2"]),
            name1=str(d["name1"]),
            name2=str(d["name2"]),
        )

        if hasattr(self, "tx_log_print"):
            self.tx_log_print(f"[TX] Saved: {filepath}")

    # -------------- Quit all --------------
    def _quit_all(self):
        try: self.rx_polling = False
        except: pass
        try: self.tx.close()
        except: pass
        try: self.rx.close()
        except: pass
        self.root.destroy()


if __name__ == '__main__':
    app = App()
    app.root.mainloop()
