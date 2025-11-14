# Importing Libraries
import time
import serial.tools.list_ports
from tkinter import *
from tkinter.font import Font

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=0.03)

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
    V_max = 32.0
    V_inc = 0.05
    tk_root = Tk()
    tk_root.title('Hybrid Control')
    tk_frame = Frame(tk_root)
    tk_frame.pack(padx=40, pady=30)

    # region Initialization
    V_EPStop = float(read())
    V_QF2top = float(read())
    V_QF3top = float(read())
    V_EPSbot = float(read())
    V_QF2bot = float(read())
    V_QF3bot = float(read())
    V_QF4 = float(read())
    V_MZMtop1 = float(read())
    V_MZMtop2 = float(read())
    V_MZMbot = float(read())
    V_PStop1 = float(read())
    V_PStop2 = float(read())
    V_PSbot = float(read())
    V_MZM1 = float(read())
    V_MZMQ = float(read())
    V_MZMC = float(read())
    PD_DWDM = 2.0
    RT_DWDM = 0.29

    V_EPStop_str = StringVar()
    V_EPSbot_str = StringVar()
    V_QF2top_str = StringVar()
    V_QF2bot_str = StringVar()
    V_QF3top_str = StringVar()
    V_QF3bot_str = StringVar()
    V_QF4_str = StringVar()
    V_MZMtop1_str = StringVar()
    V_MZMtop2_str = StringVar()
    V_MZMbot_str = StringVar()
    V_PStop1_str = StringVar()
    V_PStop2_str = StringVar()
    V_PSbot_str = StringVar()
    V_MZM1_str = StringVar()
    V_MZMQ_str = StringVar()
    V_MZMC_str = StringVar()
    PD_DWDM_str = StringVar()
    RT_DWDM_str = StringVar()

    V_EPStop_str.set(str(V_EPStop))
    V_QF2top_str.set(str(V_QF2top))
    V_QF3top_str.set(str(V_QF3top))
    V_EPSbot_str.set(str(V_EPSbot))
    V_QF2bot_str.set(str(V_QF2bot))
    V_QF3bot_str.set(str(V_QF3bot))
    V_QF4_str.set(str(V_QF4))
    V_MZMtop1_str.set(str(V_MZMtop1))
    V_MZMtop2_str.set(str(V_MZMtop2))
    V_MZMbot_str.set(str(V_MZMbot))
    V_PStop1_str.set(str(V_PStop1))
    V_PStop2_str.set(str(V_PStop2))
    V_PSbot_str.set(str(V_PSbot))
    V_MZM1_str.set(str(V_MZM1))
    V_MZMQ_str.set(str(V_MZMQ))
    V_MZMC_str.set(str(V_MZMC))
    PD_DWDM_str.set(str(PD_DWDM))
    RT_DWDM_str.set(str(RT_DWDM))
    # endregion

    time.sleep(1)
    Stb_flag = 0

    # region Buntton Functions
    def _quit():
        tk_root.quit()
        tk_root.destroy()

    def update_voltage():
        nonlocal V_EPStop, V_EPSbot, V_QF2top, V_QF2bot, V_QF3top, V_QF3bot, V_QF4, V_MZMtop1, V_MZMtop2, V_MZMbot, \
            V_MZM1, V_MZMQ, V_PStop1, V_PStop2, V_PSbot, V_MZMC, PD_DWDM, RT_DWDM

        V_EPStop = float(V_EPStop_str.get())
        V_QF2top = float(V_QF2top_str.get())
        V_QF3top = float(V_QF3top_str.get())
        V_EPSbot = float(V_EPSbot_str.get())
        V_QF2bot = float(V_QF2bot_str.get())
        V_QF3bot = float(V_QF3bot_str.get())
        V_QF4 = float(V_QF4_str.get())
        V_MZMtop1 = float(V_MZMtop1_str.get())
        V_MZMtop2 = float(V_MZMtop2_str.get())
        V_MZMbot = float(V_MZMbot_str.get())
        V_PStop1 = float(V_PStop1_str.get())
        V_PStop2 = float(V_PStop2_str.get())
        V_PSbot = float(V_PSbot_str.get())
        V_MZM1 = float(V_MZM1_str.get())
        V_MZMQ = float(V_MZMQ_str.get())
        V_MZMC = float(V_MZMC_str.get())
        PD_DWDM = float(PD_DWDM_str.get())
        RT_DWDM = float(RT_DWDM_str.get())

        write("UPDATE_VOLTAGES\n")
        write_float(V_EPStop)
        write_float(V_QF2top)
        write_float(V_QF3top)
        write_float(V_EPSbot)
        write_float(V_QF2bot)
        write_float(V_QF3bot)
        write_float(V_QF4)
        write_float(V_MZMtop1)
        write_float(V_MZMtop2)
        write_float(V_MZMbot)
        write_float(V_PStop1)
        write_float(V_PStop2)
        write_float(V_PSbot)
        write_float(V_MZM1)
        write_float(V_MZMQ)
        write_float(V_MZMC)
        write_float(PD_DWDM)
        write_float(RT_DWDM)
        while True:
            output = read()
            if output == "Done.\r\n":
                break

    def Pulse_Classical_Header_Test():
        message = str(input("Please write your message here: "))
        write("p")
        write(message)
        while True:
            test = read()
            while(test == ''):
                test = read()
            print(test)
            if test == "Done\r\n":
                break

    def Teensy_Control_Test():
        message = str(input("Please write your message here: "))
        write("t")
        write(message)
        while True:
            test = read()
            while(test == ''):
                test = read()
            print(test)
            if test == "Done\r\n":
                break

    # endregion

    # region Buttons and Spinboxes
    # region Rings
    Label(tk_frame, text="   EPStop   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=0, column=0)
    EPStop_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_EPStop_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    EPStop_Spinbox.grid(row=0, column=1)
    Label(tk_frame, text="   QF2top   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=0, column=2)
    QF2top_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_QF2top_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    QF2top_Spinbox.grid(row=0, column=3)
    Label(tk_frame, text="   QF3top   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=0, column=4)
    QF3top_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_QF3top_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    QF3top_Spinbox.grid(row=0, column=5)
    Label(tk_frame, text="     ", font=Font(family='Arial', size=18, weight='normal')).grid(row=1, column=0)

    Label(tk_frame, text="   EPSbot   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=2, column=0)
    EPSbot_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_EPSbot_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    EPSbot_Spinbox.grid(row=2, column=1)
    Label(tk_frame, text="   QF2bot   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=2, column=2)
    QF2bot_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_QF2bot_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    QF2bot_Spinbox.grid(row=2, column=3)
    Label(tk_frame, text="   QF3bot   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=2, column=4)
    QF3bot_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_QF3bot_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    QF3bot_Spinbox.grid(row=2, column=5)
    Label(tk_frame, text="     ", font=Font(family='Arial', size=18, weight='normal')).grid(row=3, column=0)
    # endregion

    # region Macros
    Label(tk_frame, text="   QF4      ", font=Font(family='Arial', size=18, weight='normal')).grid(row=4, column=0)
    QF4_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_QF4_str,
                          font=Font(family='Arial', size=24, weight='normal'))
    QF4_Spinbox.grid(row=4, column=1)
    Label(tk_frame, text="   MZM1     ", font=Font(family='Arial', size=18, weight='normal')).grid(row=4, column=2)
    MZM1_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_MZM1_str,
                           font=Font(family='Arial', size=24, weight='normal'))
    MZM1_Spinbox.grid(row=4, column=3)
    Label(tk_frame, text="   MZMQ     ", font=Font(family='Arial', size=18, weight='normal')).grid(row=4, column=4)
    MZMQ_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_MZMQ_str,
                           font=Font(family='Arial', size=24, weight='normal'))
    MZMQ_Spinbox.grid(row=4, column=5)
    # endregion

    # region MZM and PS
    Label(tk_frame, text="   MZMtop1  ", font=Font(family='Arial', size=18, weight='normal')).grid(row=5, column=0)
    MZMtop1_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_MZMtop1_str,
                              font=Font(family='Arial', size=24, weight='normal'))
    MZMtop1_Spinbox.grid(row=5, column=1)
    Label(tk_frame, text="   MZMtop2  ", font=Font(family='Arial', size=18, weight='normal')).grid(row=5, column=2)
    MZMtop2_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_MZMtop2_str,
                              font=Font(family='Arial', size=24, weight='normal'))
    MZMtop2_Spinbox.grid(row=5, column=3)
    Label(tk_frame, text="   MZMbot   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=5, column=4)
    MZMbot_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_MZMbot_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    MZMbot_Spinbox.grid(row=5, column=5)

    Label(tk_frame, text="   PStop1   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=6, column=0)
    PStop1_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_PStop1_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    PStop1_Spinbox.grid(row=6, column=1)
    Label(tk_frame, text="   PStop2   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=6, column=2)
    PStop2_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_PStop2_str,
                             font=Font(family='Arial', size=24, weight='normal'))
    PStop2_Spinbox.grid(row=6, column=3)
    Label(tk_frame, text="   PSbot    ", font=Font(family='Arial', size=18, weight='normal')).grid(row=6, column=4)
    PSbot_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_PSbot_str,
                            font=Font(family='Arial', size=24, weight='normal'))
    PSbot_Spinbox.grid(row=6, column=5)
    Label(tk_frame, text="     ", font=Font(family='Arial', size=18, weight='normal')).grid(row=7, column=0)

    Label(tk_frame, text="   MZMC     ", font=Font(family='Arial', size=18, weight='normal')).grid(row=8, column=0)
    MZMC_Spinbox = Spinbox(tk_frame, from_=0, to=V_max, increment=V_inc, width=6, textvariable=V_MZMC_str,
                           font=Font(family='Arial', size=24, weight='normal'))
    MZMC_Spinbox.grid(row=8, column=1)
    Label(tk_frame, text="   PDDWDM   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=8, column=2)
    PD_DWDM_Spinbox = Spinbox(tk_frame, from_=1, to=4, increment=1, width=6, textvariable=PD_DWDM_str,
                           font=Font(family='Arial', size=24, weight='normal'))
    PD_DWDM_Spinbox.grid(row=8, column=3)
    Label(tk_frame, text="   RTDWDM   ", font=Font(family='Arial', size=18, weight='normal')).grid(row=8, column=4)
    PD_DWDM_Spinbox = Spinbox(tk_frame, from_=0.0, to=1.0, increment=0.01, width=6, textvariable=RT_DWDM_str,
                              font=Font(family='Arial', size=24, weight='normal'))
    PD_DWDM_Spinbox.grid(row=8, column=5)
    # endregion

    # region Buttons
    PulseButton = Button(master=tk_root,
                     text="Pulse Test",
                     command=Pulse_Classical_Header_Test,
                     font=('Arial', 18))
    PulseButton.pack(side=LEFT, fill='both', expand=False)

    TeensyButton = Button(master=tk_root,
                         text="Teensy Test",
                         command=Teensy_Control_Test,
                         font=('Arial', 18))
    TeensyButton.pack(side=LEFT, fill='both', expand=False)

    QuitButton = Button(master=tk_root, text="Quit", command=_quit, font=('Arial', 18))
    QuitButton.pack(side=RIGHT, fill='both', expand=False)
    UpdateButton = Button(master=tk_root, text="Update", command=update_voltage, font=('Arial', 18))
    UpdateButton.pack(side=RIGHT, fill='both', expand=False)

    mainloop()


if __name__ == '__main__':
    time.sleep(1)
    while True:
        output = read()
        if output == "Ready.\r\n":
            break
    Hybrid_Control()
