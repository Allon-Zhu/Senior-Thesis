import time
import sys
from pathlib import Path
from tkinter import BOTH, END, LEFT, RIGHT, Button, Frame, Label, Spinbox, StringVar, Tk, Toplevel, messagebox, simpledialog
from tkinter.font import Font
from tkinter.scrolledtext import ScrolledText

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

CURRENT_DIR = Path(__file__).resolve().parent
PARENT_DIR = CURRENT_DIR.parent
for path in (CURRENT_DIR, PARENT_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from automated_calibration.remote_laser_control import DLCProSimple
from config_manager import ConfigManager
from dialogs import Scan2VDialog
from rx_controller import RxController
from serial_utils import open_arduino, readline_str, wait_for_line
from tx_controller import TxController

try:
    from ring_calibration_helper import tx_scan_frequency
except Exception:
    tx_scan_frequency = None

try:
    from highQ_piezoscan_helper import tx_scan_piezo
except Exception:
    tx_scan_piezo = None


class HybridControlApp:
    def __init__(self):
        self.config = ConfigManager()
        connection = self.config.connection()

        self.tx_serial = open_arduino(connection["tx_port"], connection["baudrate"], connection["timeout"])
        wait_for_line(self.tx_serial, "TX Ready.", timeout_s=10.0)

        self.rx_serial = open_arduino(connection["rx_port"], connection["baudrate"], connection["timeout"])
        wait_for_line(self.rx_serial, "RX Ready.", timeout_s=10.0)

        self.tx_controller = TxController(self.tx_serial, self.config)
        self.rx_controller = RxController(self.rx_serial, self.config)

        self.tx_controller.sync_board_parameters()
        self.rx_controller.sync_board_parameters()
        self.tx_controller.push_all_voltages()

        try:
            self.laser = DLCProSimple(connection["laser_host"]).open()
        except Exception:
            self.laser = None

        self.root = Tk()
        self.root.title("Hybrid TX/RX Control")
        self.root.withdraw()

        self.rx_history = []
        self.rx_polling = True

        self._build_tx_panel()
        self._build_rx_panel()

        self.root.protocol("WM_DELETE_WINDOW", self.quit_all)
        self.tx_win.protocol("WM_DELETE_WINDOW", self.quit_all)
        self.rx_win.protocol("WM_DELETE_WINDOW", self.quit_all)

    def _build_tx_panel(self):
        self.tx_win = Toplevel(self.root)
        self.tx_win.title("TX Control Panel")
        frame = Frame(self.tx_win)
        frame.pack(padx=40, pady=30)

        self.tx_log = ScrolledText(self.tx_win, height=8, width=90, font=("Consolas", 10))
        self.tx_log.pack(fill=BOTH, expand=False, padx=10, pady=(0, 10))

        self.voltage_vars = {}
        default_range = self.tx_controller.gui_ranges.get("default", {"min": 0.0, "max": 30.0, "step": 0.05})

        for idx, name in enumerate(self.tx_controller.channel_order):
            channel = self.tx_controller.channels[name]
            self.voltage_vars[name] = StringVar(value=str(float(channel["voltage"])))
            row = idx // 4
            col = (idx % 4) * 2
            Label(frame, text=f"  {name}  ", font=Font(family="Arial", size=16)).grid(row=row, column=col, sticky="e")
            range_cfg = self.tx_controller.gui_ranges.get(name, default_range)
            Spinbox(
                frame,
                from_=range_cfg["min"],
                to=range_cfg["max"],
                increment=range_cfg["step"],
                width=8,
                textvariable=self.voltage_vars[name],
                font=Font(family="Arial", size=18),
            ).grid(row=row, column=col + 1, padx=(0, 15), pady=5, sticky="w")

        Button(self.tx_win, text="Set TX Levels", command=self.tx_set_levels, font=("Arial", 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Pulse Test", command=self.tx_pulse_test, font=("Arial", 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Scan 2V", command=self.tx_scan_2v, font=("Arial", 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Reload Config", command=self.reload_tx_config, font=("Arial", 16)).pack(side=LEFT, padx=10, pady=10)
        if tx_scan_frequency is not None:
            Button(self.tx_win, text="Scan Frequency", command=lambda: tx_scan_frequency(self), font=("Arial", 16)).pack(side=LEFT, padx=10, pady=10)
        if tx_scan_piezo is not None:
            Button(self.tx_win, text="Scan Piezo", command=lambda: tx_scan_piezo(self), font=("Arial", 16)).pack(side=LEFT, padx=10, pady=10)
        Button(self.tx_win, text="Update", command=self.tx_update_changed_from_gui, font=("Arial", 16)).pack(side=RIGHT, padx=10, pady=10)
        Button(self.tx_win, text="Quit", command=self.quit_all, font=("Arial", 16)).pack(side=RIGHT, padx=10, pady=10)

        self.tx_log_print("[TX] Config synced to board.")

    def _build_rx_panel(self):
        self.rx_win = Toplevel(self.root)
        self.rx_win.title("RX Control Panel")

        controls = Frame(self.rx_win)
        controls.pack(padx=12, pady=10, fill="x")

        Button(controls, text="Start Stream", command=self.rx_stream_start, font=("Arial", 14)).pack(side=LEFT, padx=6)
        Button(controls, text="Stop Stream", command=self.rx_stream_stop, font=("Arial", 14)).pack(side=LEFT, padx=6)
        Button(controls, text="Decode Mode", command=self.rx_decode_mode, font=("Arial", 14)).pack(side=LEFT, padx=6)
        Button(controls, text="Reconnect", command=self.rx_reconnect, font=("Arial", 14)).pack(side=LEFT, padx=6)

        plot_frame = Frame(self.rx_win)
        plot_frame.pack(padx=10, pady=(0, 10), fill=BOTH, expand=True)

        self.rx_fig = Figure(figsize=(5, 7), dpi=100)
        self.rx_ax = self.rx_fig.add_subplot(111)
        self.rx_ax.set_xlabel("Time (s)")
        self.rx_ax.set_ylabel("Value")
        self.rx_lines = [self.rx_ax.plot([], [], label=f"Ch {idx + 1}")[0] for idx in range(4)]
        self.rx_canvas = FigureCanvasTkAgg(self.rx_fig, master=plot_frame)
        self.rx_canvas.get_tk_widget().pack(fill=BOTH, expand=True)

        self.rx_log = ScrolledText(self.rx_win, height=8, width=100, font=("Consolas", 10))
        self.rx_log.pack(fill=BOTH, expand=True, padx=10, pady=10)

        self.rx_poll()

    def tx_log_print(self, text):
        self.tx_log.insert(END, text + "\n")
        self.tx_log.see(END)

    def rx_log_print(self, text):
        self.rx_log.insert(END, text + "\n")
        self.rx_log.see(END)

    def reload_tx_config(self):
        self.tx_controller.refresh_from_config()
        self.tx_log_print("[TX] Reloaded config file. Restart the app to rebuild controls.")

    def tx_update_changed_from_gui(self):
        try:
            gui_values = {name: float(var.get()) for name, var in self.voltage_vars.items()}
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return

        updates = self.tx_controller.diff_channel_updates(gui_values)
        if not updates:
            self.tx_log_print("[TX] No voltage changes.")
            return

        changed = self.tx_controller.update_channels(updates, persist=True)
        self.tx_log_print(f"[TX] Updated {len(changed)} channel(s): {', '.join(changed)}")

    def tx_set_levels(self):
        low = simpledialog.askfloat("Set TX Levels", "VL (V):", minvalue=0.0, maxvalue=30.0, parent=self.tx_win)
        if low is None:
            return
        high = simpledialog.askfloat("Set TX Levels", "VH (V):", minvalue=0.0, maxvalue=30.0, parent=self.tx_win)
        if high is None:
            return
        self.tx_controller.set_levels(low, high)
        self.tx_log_print(f"[TX] Levels updated: low={low}, high={high}")

    def tx_pulse_test(self):
        message = simpledialog.askstring("Pulse Test", "Enter message to send:", parent=self.tx_win)
        if message is None:
            return
        self.tx_controller.send_message(message)
        self.tx_log_print(f"[TX] SEND_MESSAGE '{message}'")

    def tx_scan_2v(self):
        dialog = Scan2VDialog(self.tx_win, self.tx_controller.channel_order)
        self.tx_win.wait_window(dialog.top)
        if dialog.result is None:
            return

        data = self.tx_controller.run_scan_2v(*dialog.result)
        self._save_scan_2v(data)
        self._plot_scan_2v(data)

    def _plot_scan_2v(self, data):
        z_sum = data["PD1"] + data["PD2"]
        z_diff = (data["PD1"] - data["PD2"]) / np.where(z_sum == 0, 1, z_sum)
        extent = [
            float(data["V1_axis"][0] ** 2),
            float(data["V1_axis"][-1] ** 2),
            float(data["V2_axis"][0] ** 2),
            float(data["V2_axis"][-1] ** 2),
        ]

        for title, values in (
            (f"PD{data['pd1']}", data["PD1"]),
            (f"PD{data['pd2']}", data["PD2"]),
            (f"PD{data['pd1']} + PD{data['pd2']}", z_sum),
            (f"PD{data['pd1']} - PD{data['pd2']}", z_diff),
        ):
            plt.figure()
            plt.title(title)
            plt.imshow(values, origin="lower", aspect="auto", extent=extent)
            plt.colorbar()
            plt.xlabel(data["name1"])
            plt.ylabel(data["name2"])
        plt.show()

    def _save_scan_2v(self, data):
        from tkinter import filedialog

        filepath = filedialog.asksaveasfilename(
            parent=self.tx_win,
            title="Save Scan 2V Result",
            defaultextension=".npz",
            initialfile=f"scan2v_{data['name1']}_{data['name2']}.npz",
            filetypes=[("NumPy compressed", "*.npz")],
        )
        if not filepath:
            self.tx_log_print("[TX] Save cancelled.")
            return

        np.savez_compressed(
            filepath,
            PD1=data["PD1"],
            PD2=data["PD2"],
            PDsum=data["PD1"] + data["PD2"],
            PDdiff=(data["PD1"] - data["PD2"]) / np.where((data["PD1"] + data["PD2"]) == 0, 1, (data["PD1"] + data["PD2"])),
            V1_axis=data["V1_axis"],
            V2_axis=data["V2_axis"],
            pd1=int(data["pd1"]),
            pd2=int(data["pd2"]),
            name1=str(data["name1"]),
            name2=str(data["name2"]),
        )
        self.tx_log_print(f"[TX] Saved: {filepath}")

    def rx_poll(self):
        if not self.rx_polling:
            return

        new_data = False
        try:
            while self.rx_controller.ser.in_waiting:
                line = self.rx_controller.ser.readline().decode("utf-8", errors="ignore")
                if line:
                    self.rx_log_print(line.rstrip())
                    values = self._rx_parse_values(line)
                    if values is not None:
                        self._rx_add_sample(values)
                        new_data = True
        except Exception:
            pass

        if new_data:
            self._rx_update_plot()

        self.rx_win.after(20, self.rx_poll)

    def _rx_parse_values(self, line):
        parts = line.strip().split()
        if len(parts) < 4:
            return None
        try:
            return [float(part) for part in parts[:4]]
        except ValueError:
            return None

    def _rx_add_sample(self, values):
        now = time.time()
        self.rx_history.append((now, values))
        window_s = float(self.config.data["rx"].get("plot_window_s", 60.0))
        cutoff = now - window_s
        self.rx_history = [sample for sample in self.rx_history if sample[0] >= cutoff]

    def _rx_update_plot(self):
        if not self.rx_history:
            return
        start = self.rx_history[0][0]
        xs = [timestamp - start for timestamp, _ in self.rx_history]
        for idx in range(4):
            ys = [values[idx] for _, values in self.rx_history]
            self.rx_lines[idx].set_data(xs, ys)
        self.rx_ax.set_xlim(0, max(float(self.config.data["rx"].get("plot_window_s", 60.0)), xs[-1]))
        self.rx_ax.set_ylim(0, float(self.config.data["rx"].get("plot_y_max", 4096)))
        self.rx_canvas.draw_idle()

    def rx_stream_start(self):
        self.rx_log_print("[RX] STREAM_START")
        self.rx_controller.stream_start()

    def rx_stream_stop(self):
        self.rx_log_print("[RX] STREAM_STOP")
        self.rx_controller.stream_stop()

    def rx_decode_mode(self):
        self.rx_log_print("[RX] DECODE_MODE")
        self.rx_controller.decode_mode()

    def rx_reconnect(self):
        self.rx_controller.reconnect()
        self.rx_controller.sync_board_parameters()
        self.rx_log_print("[RX] Reconnected and resynced parameters.")

    def quit_all(self):
        self.rx_polling = False
        try:
            self.tx_controller.ser.close()
        except Exception:
            pass
        try:
            self.rx_controller.ser.close()
        except Exception:
            pass
        self.root.destroy()
