# -*- coding: utf-8 -*-
import time
import serial
from tkinter import *
from tkinter.font import Font
from tkinter import simpledialog, messagebox
from tkinter.scrolledtext import ScrolledText
import sys
PROJECT_ROOT = r'C:\Alan Zhu\Senior_Thesis'
sys.path.append(PROJECT_ROOT)
from Basic_Controls.automated_calibration.remote_laser_control import DLCProSimple
from ring_calibration_helper import tx_scan_frequency, tx_calibrate_rings

LASER_HOST = "192.168.1.200"

# ---------- Serial helpers ----------
def open_arduino(port, baud=115200, to=0.2):
    ser = serial.Serial(port=port, baudrate=baud, timeout=to)
    time.sleep(2.0)                # allow auto-reset
    return ser

def readline_str(ser):
    try:
        line = ser.readline().decode('utf-8', errors='ignore')
    except Exception:
        return ''
    return line

def write_line(ser, s):
    if not s.endswith('\n'):
        s += '\n'
    ser.write(s.encode('utf-8'))

def write_float_line(ser, x):
    ser.write(f"{float(x)}\r\n".encode('utf-8'))
    # small pacing to avoid sticking floats together on very short timeouts
    time.sleep(0.02)

# ---------- App ----------
class App:
    def __init__(self):
        # Open both boards
        self.tx = open_arduino('COM4', 115200, 0.2)
        while True:
            line = readline_str(self.tx)
            if line.strip() == "Ready.":
                break
        print("Connected to TX")
        self.rx = open_arduino('COM3', 115200, 0.2)
        while True:
            line = readline_str(self.rx)
            if line.strip() == "Ready.":
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
        default_voltages = {
            'V_MZMtop1': 18.1, 'V_MZMtop2': 21.0, 'V_MZMbot': 29.5,
            'V_PStop1': 21.0,  'V_PStop2': 23.0,  'V_PSbot': 21.9,
            'V_MZM1': 24.0,    'V_MZMQ': 22.8,    'V_MZMC': 30.5,
            'V_QF4': 21.0,     'V_EPStop': 11.25, 'V_QF2top': 27.3,
            'V_QF3top': 19.3,  'V_EPSbot': 11.2,  'V_QF2bot': 26.28,
            'V_QF3bot': 19.6,  'PD_DWDM': 2.0,    'RT_DWDM': 0.29
        }
        self.tx_write_order = [
            'V_EPStop','V_QF2top','V_QF3top','V_EPSbot','V_QF2bot','V_QF3bot','V_QF4',
            'V_MZMtop1','V_MZMtop2','V_MZMbot','V_PStop1','V_PStop2','V_PSbot',
            'V_MZM1','V_MZMQ','V_MZMC','PD_DWDM','RT_DWDM'
        ]
        V_max, V_inc = 32.0, 0.05
        ranges = {'PD_DWDM': (1.0, 4.0, 1.0), 'RT_DWDM': (0.0, 1.0, 0.01)}

        self.tx_win = Toplevel(self.root)
        self.tx_win.title("TX Control Panel")
        frame = Frame(self.tx_win); frame.pack(padx=40, pady=30)

        self.tx_log = ScrolledText(self.tx_win, height=8, width=90, font=('Consolas', 10))
        self.tx_log.pack(fill='both', expand=False, padx=10, pady=(0,10))
        self.tx_log_print = lambda s: (self.tx_log.insert(END, s + "\n"), self.tx_log.see(END))

        # vars
        self.voltage_vars = {k: StringVar(value=str(v)) for k, v in default_voltages.items()}
        for k in self.tx_write_order:
            self.voltage_vars.setdefault(k, StringVar(value="0.0"))

        # grid of spinboxes
        gui_order = [
            'V_EPStop','V_QF2top','V_QF3top',
            'V_EPSbot','V_QF2bot','V_QF3bot',
            'V_QF4','V_MZM1','V_MZMQ',
            'V_MZMtop1','V_MZMtop2','V_MZMbot',
            'V_PStop1','V_PStop2','V_PSbot',
            'V_MZMC','PD_DWDM','RT_DWDM'
        ]
        for idx, name in enumerate(gui_order):
            r = idx // 3
            c = (idx % 3) * 2
            Label(frame, text=f"  {name}  ", font=Font(family='Arial', size=16)).grid(row=r, column=c, sticky='e')
            lo, hi, inc = ranges.get(name, (0.0, V_max, V_inc))
            Spinbox(frame, from_=lo, to=hi, increment=inc, width=8,
                    textvariable=self.voltage_vars[name],
                    font=Font(family='Arial', size=18)).grid(row=r, column=c+1, padx=(0,15), pady=5, sticky='w')

        # buttons
        Button(self.tx_win, text="Pulse Test",  command=self.tx_pulse_test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Teensy Test", command=self.tx_teensy_test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Set TX Levels", command=self.tx_set_levels, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Update", command=self.tx_update_from_gui, font=('Arial', 16)).pack(side=RIGHT, padx=10, pady=10)
        Button(self.tx_win, text="Quit",   command=self._quit_all, font=('Arial', 16)).pack(side=RIGHT, padx=10, pady=10)
        Button(self.tx_win, text="Scan Frequency", command=lambda: tx_scan_frequency(self), font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Calibrate Rings", command=lambda: tx_calibrate_rings(self), font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)

        # push defaults once
        self.tx_send_voltages({k: float(self.voltage_vars[k].get()) for k in self.tx_write_order})
        self.tx_log_print("[TX] Defaults sent.")

    def tx_send_voltages(self, vd):
        write_line(self.tx, "UPDATE_VOLTAGES")
        # wait for ACK
        while True:
            out = readline_str(self.tx)
            if out.strip() == "ACK": break
        # send floats
        for key in self.tx_write_order:
            write_float_line(self.tx, vd[key])
        # wait done
        while True:
            out = readline_str(self.tx)
            if out.strip() == "Done.": break

    def tx_update_from_gui(self):
        try:
            vd = {k: float(self.voltage_vars[k].get()) for k in self.tx_write_order}
        except ValueError as e:
            messagebox.showerror("Invalid input", f"Bad numeric value: {e}")
            return
        self.tx_send_voltages(vd)
        self.tx_log_print("[TX] GUI voltages sent.")

    def tx_pulse_test(self):
        msg = simpledialog.askstring("Pulse Test", "Enter message to send:", parent=self.tx_win)
        if msg is None: return
        write_line(self.tx, "SEND_MESSAGE")
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

        ctrl = Frame(self.rx_win);
        ctrl.pack(padx=12, pady=10, fill='x')
        self.rx_log = ScrolledText(self.rx_win, height=16, width=100, font=('Consolas', 10))
        self.rx_log.pack(fill='both', expand=True, padx=10, pady=10)
        self.rx_log_print = lambda s: (self.rx_log.insert(END, s + "\n"), self.rx_log.see(END))

        Button(ctrl, text="Start Stream", command=self.rx_stream_start, font=('Arial', 14)).pack(side=LEFT, padx=6)
        Button(ctrl, text="Stop Stream", command=self.rx_stream_stop, font=('Arial', 14)).pack(side=LEFT, padx=6)
        Button(ctrl, text="Decode Mode (blocking)", command=self.rx_decode_mode, font=('Arial', 14)).pack(side=LEFT,
                                                                                                          padx=6)
        Button(ctrl, text="Exit Decode (Reconnect)", command=self.rx_reconnect, font=('Arial', 14)).pack(side=LEFT,
                                                                                                         padx=6)

        # poll RX (non-blocking) to show stream lines
        self.rx_polling = True
        self.rx_poll()

    def rx_poll(self):
        if self.rx_polling:
            try:
                while self.rx.in_waiting:
                    line = self.rx.readline().decode('utf-8', errors='ignore')
                    if line:
                        self.rx_log_print(line.rstrip())
            except Exception:
                pass
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
        self.rx = open_arduino('COM3', 115200, 0.2)
        # Wait for Ready.
        while True:
            line = self.rx.readline().decode('utf-8', errors='ignore')
            if line.strip() == "Ready.":
                break
        self.rx_log_print("[RX] Reconnected to COM3 (Ready.)")

    def rx_measure_pd(self, pd_channel: int, timeout_s: float = 0.1) -> float:
        """
        Send MEASURE_PD{n} to RX and return the numeric reading.
        Assumes RX is in command mode (not streaming).
        """
        cmd = f"MEASURE_PD{int(pd_channel)}"
        self.rx_log_print(f"[RX] {cmd}")
        write_line(self.rx, cmd)

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            line = readline_str(self.rx)
            if not line:
                # allow Tk to breathe a bit
                self.rx_win.update_idletasks()
                self.rx_win.update()
                continue
            line = line.strip()
            if not line:
                continue

            # Expect something like: "PD1 1234"
            self.rx_log_print("[RX<-] " + line)
            parts = line.split()
            # Try last token as a number
            for tok in reversed(parts):
                try:
                    return float(tok)
                except ValueError:
                    continue

        raise TimeoutError(f"Timeout waiting for PD{pd_channel} reading")

    # -------------- Quit all --------------
    def _quit_all(self):
        try: self.rx_polling = False
        except: pass
        try: self.tx.close()
        except: pass
        try: self.rx.close()
        except: pass
        try:
            if getattr(self, "laser", None) is not None:
                self.laser.close()
        except:
            pass
        self.root.destroy()

if __name__ == '__main__':
    app = App()
    app.root.mainloop()
