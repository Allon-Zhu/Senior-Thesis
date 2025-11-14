# Importing Libraries
import time
import serial.tools.list_ports
from tkinter import *
from tkinter.font import Font
from tkinter import simpledialog, messagebox
from tkinter.scrolledtext import ScrolledText

def read():
    data = arduino.readline()
    data = data.decode('UTF-8')
    return data

def query(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

def write(x):
    arduino.write(bytes(str(x), 'utf-8'))

def write_float(x):
    arduino.write(bytes(str(x), 'utf-8'))
    arduino.write(b'\r\n')
    time.sleep(0.05)


def Hybrid_Control():
    default_voltages = {
        'V_MZMtop1': 18.1,
        'V_MZMtop2': 21.0,
        'V_MZMbot': 29.5,
        'V_PStop1': 21.0,
        'V_PStop2': 23.0,
        'V_PSbot': 21.9,
        'V_MZM1': 24.0,
        'V_MZMQ': 22.8,
        'V_MZMC': 30.5,
        'V_QF4': 21.0,
        'V_EPStop': 11.25,
        'V_QF2top': 27.3,
        'V_QF3top': 19.3,
        'V_EPSbot': 11.2,
        'V_QF2bot': 26.28,
        'V_QF3bot': 19.6,
        'PD_DWDM': 2.0,
        'RT_DWDM': 0.29
    }
    V_max = 32.0
    V_inc = 0.05

    # Optional per-channel ranges (falls back to 0..32 V with 0.05 step if not present)
    ranges = {
        'PD_DWDM':  (1.0, 4.0, 1.0),
        'RT_DWDM':  (0.0, 1.0, 0.01),
        # Everything else uses default V_max/V_inc
    }

    # GUI setup
    tk_root = Tk()
    tk_root.title('Hybrid Control')
    tk_frame = Frame(tk_root)
    tk_frame.pack(padx=40, pady=30)

    # A small log window for test outputs
    log = ScrolledText(tk_root, height=8, width=80, font=('Consolas', 10))
    log.pack(fill='both', expand=False, padx=10, pady=(0,10))

    def log_print(s: str):
        log.insert(END, s + "\n")
        log.see(END)

    # Create StringVars dynamically from default_voltages
    voltage_vars = {name: StringVar() for name in default_voltages}

    # Lay out spinboxes in a nice grid, three columns of pairs (Label, Spinbox)
    # Preserve a stable order by using a fixed list (also used for Arduino write order).
    write_order = [
        'V_EPStop', 'V_QF2top', 'V_QF3top', 'V_EPSbot', 'V_QF2bot', 'V_QF3bot', 'V_QF4',
        'V_MZMtop1', 'V_MZMtop2', 'V_MZMbot', 'V_PStop1', 'V_PStop2', 'V_PSbot',
        'V_MZM1', 'V_MZMQ', 'V_MZMC', 'PD_DWDM', 'RT_DWDM'
    ]

    # Build any missing keys (so GUI shows them if they weren’t in defaults yet)
    for k in write_order:
        if k not in voltage_vars:
            voltage_vars[k] = StringVar()
            default_voltages.setdefault(k, 0.0)

    # Initialize GUI variables to defaults
    for name, val in default_voltages.items():
        voltage_vars[name].set(str(val))

    # Helper: send full voltage set to Arduino in the required order
    def write_voltage_sequence(voltage_dict):
        write("UPDATE_VOLTAGES\n")
        while True:
            out = read()
            if out.strip() == "ACK":
                break
        for key in write_order:
            write_float(float(voltage_dict[key]))
        while True:
            output = read()
            if output.strip() == "Done.":
                break

    # Public: update from dict (if provided) or from GUI
    def update_voltage(voltage_dict=None):
        if voltage_dict is not None:
            # Push dict values into GUI
            for name, val in voltage_dict.items():
                if name in voltage_vars:
                    voltage_vars[name].set(str(val))
            # Then send to Arduino
            write_voltage_sequence(voltage_dict)
            log_print("[INFO] Sent default voltages to Arduino.")
        else:
            # Read GUI -> dict, then send to Arduino
            current_vals = {}
            try:
                for k, sv in voltage_vars.items():
                    current_vals[k] = float(sv.get())
            except ValueError as e:
                messagebox.showerror("Invalid input", f"One or more entries are not valid floats.\n{e}")
                return
            write_voltage_sequence(current_vals)
            log_print("[INFO] Sent GUI voltages to Arduino.")

    # ---- Test actions (non-blocking dialogs) ----
    def Pulse_Classical_Header_Test():
        # Ask user for a message without blocking the GUI loop
        msg = simpledialog.askstring("Pulse Test", "Enter message to send:")
        if msg is None:
            return
        write("SEND_MESSAGE\n")
        write(msg)
        log_print(f"[TX] Sending Message: {msg!r}")
        # Read until Done
        while True:
            resp = read()
            if resp == '':
                tk_root.update_idletasks()
                tk_root.update()
                continue
            log_print(f"[RX] {resp.strip()}")
            if resp.strip() == "Done":
                break

    def Teensy_Control_Test():
        msg = simpledialog.askstring("Teensy Test", "Enter message to send:")
        if msg is None:
            return
        write("TEST_TEENSY\n")
        write(msg)
        log_print(f"[TX] Setting Pulse: {msg!r}")
        while True:
            resp = read()
            if resp == '':
                tk_root.update_idletasks()
                tk_root.update()
                continue
            log_print(f"[RX] {resp.strip()}")
            if resp.strip() == "Done":
                break

    def set_tx_levels():
        # Ask user for low and high; prefill with something sensible if you like
        vl = simpledialog.askfloat("Set TX Levels", "Enter VL (low) in volts:", minvalue=0.0, maxvalue=30.0,
                                   parent=tk_root)
        if vl is None:
            return
        vh = simpledialog.askfloat("Set TX Levels", "Enter VH (high) in volts:", minvalue=0.0, maxvalue=30.0,
                                   parent=tk_root)
        if vh is None:
            return
        write("SET_TX_LEVELS\n")
        write_float(float(vl))
        write_float(float(vh))
        # wait for Arduino to acknowledge
        while True:
            resp = read()
            if resp == '':
                tk_root.update_idletasks()
                tk_root.update()
                continue
            log_print(f"[RX] {resp.strip()}")
            if resp.strip() == "Done":
                break

    # ---- Build the grid of controls ----
    # Use a stable order that groups things similarly to your original layout
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
        Label(tk_frame, text=f"  {name}  ", font=Font(family='Arial', size=16)).grid(row=r, column=c, sticky='e')
        lo, hi, inc = ranges.get(name, (0.0, V_max, V_inc))
        sp = Spinbox(
            tk_frame, from_=lo, to=hi, increment=inc, width=8,
            textvariable=voltage_vars[name], font=Font(family='Arial', size=18)
        )
        sp.grid(row=r, column=c+1, padx=(0,15), pady=5, sticky='w')

    # Buttons
    def _quit():
        tk_root.quit()
        tk_root.destroy()

    Button(tk_root, text="Pulse Test",  command=Pulse_Classical_Header_Test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
    Button(tk_root, text="Teensy Test", command=Teensy_Control_Test, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
    Button(tk_root, text="Update",      command=lambda: update_voltage(None), font=('Arial', 16)).pack(side=RIGHT, padx=10, pady=10)
    Button(tk_root, text="Quit",        command=_quit, font=('Arial', 16)).pack(side=RIGHT, padx=10, pady=10)
    Button(tk_root, text="Set TX Levels", command=set_tx_levels, font=('Arial', 16)).pack(side=LEFT, padx=10, pady=10)
    # Initialize Arduino + GUI with defaults
    update_voltage(default_voltages)

    mainloop()



if __name__ == '__main__':
    time.sleep(1)
    arduino = serial.Serial(port='COM4', baudrate=115200, timeout=0.03)
    while True:
        output = read()
        if output.strip() == "Ready.":
            break
    Hybrid_Control()
    arduino.close()
