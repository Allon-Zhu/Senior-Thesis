# Importing Libraries
import time
from turtledemo.penrose import start

import serial.tools.list_ports
from matplotlib.lines import lineStyles
from numpy import zeros, sqrt, sin, pi, roll, arcsin, array, mean
from scipy import optimize
import matplotlib
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import threading

matplotlib.use('TkAgg', force=True)
from tkinter.font import Font

from Tx_Control_func import *

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=0.03)


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
    R_EPStop = -14.875
    R_EPSbot = -14.538

    LUT_Slope_MZMtop2 = -0.6098
    LUT_Slope_PStop2 = -0.6105
    LUT_Slope_PStop1 = -0.5435

    V_PStop2_cent = 23.0
    V_MZMtop2_cent = 21.0
    V_PStop1_cent = 21.0
    GPC1_VIS12_threshold = 0.9
    GPC1_VIS34_threshold = 0.9
    GPC1_MIN34_threshold = 0.15

    RPS_dS12 = 0.0
    RPS_dS34 = 0.0
    RPSY_dS12 = 0.0
    RPSY_dS34 = 0.0

    LUT_calib_dV2 = 50
    LUT_calib_N = 5

    tk_root = Tk()
    tk_root.title('Hybrid Control')
    tk_frame = Frame(tk_root)
    tk_frame.pack(padx=40, pady=30)

    time.sleep(1)
    write("0")
    time.sleep(1)

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
    V_MZMtop2_ref = V_MZMtop2
    V_PStop2_ref = V_PStop2
    V_PStop1_ref = V_PStop1
    V_EPStop_ref = V_EPStop
    V_EPSbot_ref = V_EPSbot

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
        write("0")
        write("0")
        tk_root.quit()
        tk_root.destroy()

    def _quit2():
        write("0")
        # write("0")
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

        write("1")
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

    def RPC_voltage():
        V_EPStop_str.set(str(round(V_EPStop - 4.0, 3)))
        V_EPSbot_str.set(str(round(V_EPSbot - 4.0, 3)))
        V_MZMC_str.set(str(round(23.5, 3)))
        V_MZMQ_str.set(str(round(14.0, 3)))
        update_voltage()

    def RPS_voltage():
        V_EPStop_str.set(str(round(V_EPStop + 4.0, 3)))
        V_EPSbot_str.set(str(round(V_EPSbot + 4.0, 3)))
        V_MZMC_str.set(str(round(30.5, 3)))
        V_MZMQ_str.set(str(round(16.6, 3)))
        update_voltage()

    def stabilize_laser():
        nonlocal V_EPStop, V_EPSbot, Stb_flag
        Stb_flag = 1
        Stb_rate = 5
        Stb_period = 1 / float(Stb_rate)
        V_EPStop = float(V_EPStop_str.get())
        vmb = VmbSystem.get_instance()
        ExposureTime = input("Exposure time: ")
        Stb_fraction = input("Stabilization reference: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    print("--- Instruments Connected ---")

                    # Firstly do a wide scan with PZ
                    print("--- Piezo-scan Calibration ---")
                    # PZ_c = dlc.laser1.dl.pc.voltage_act.get()
                    [piezo_calib, fraction_calib, I_max, I_min] = DLC_Scan_FromCenter(dlc, cam)
                    dlc.laser1.dl.pc.voltage_set.set(piezo_calib[0])
                    time.sleep(0.1)
                    piezo_ref = ResStb_findPZ(piezo_calib, fraction_calib, Stb_fraction)
                    piezo_init = ResStb_findPZ(piezo_calib, fraction_calib, 0.25)
                    dlc.laser1.dl.pc.voltage_set.set(piezo_ref)
                    # dlc.laser1.dl.pc.voltage_set.set(piezo_init)
                    time.sleep(0.1)
                    piezo_v = piezo_ref

                    # Then constantly calculate ratio and PZ feedback
                    print("--- Stabilization Start ---")
                    t0 = time.time()
                    Stb_cycle = 1
                    while True:
                        tk_root.update()
                        t1 = time.time()
                        if t1 - t0 < Stb_cycle * Stb_period:
                            continue

                        Stb_cycle = Stb_cycle + 1
                        if Stb_flag == 0:
                            dlc.laser1.dl.pc.voltage_set.set(70.0)
                            return
                        elif Stb_flag == 2:
                            continue
                        # while Stb_flag != 1:
                        #     tk_root.update()
                        #     if Stb_flag == 0:
                        #         return
                        #     elif Stb_flag == 2:
                        #         continue

                        photo = Get_Vmb_Photo(cam)
                        # fraction = (max(photo) - I_min) / (I_max - I_min)
                        fraction = (sum(photo) - I_min) / (I_max - I_min)
                        if fraction < 0:
                            fraction = 0
                        elif fraction > 1:
                            fraction = 1
                        piezo_new = ResStb_findPZ(piezo_calib, fraction_calib, fraction)
                        piezo_v = piezo_v - (piezo_new - piezo_ref)
                        dlc.laser1.dl.pc.voltage_set.set(piezo_v)
                        print(fraction)
                        # print([fraction, piezo_new - piezo_ref, piezo_v])

                    # write("11")
                    # write_float(V_EPStop)
                    # write_float(V_QF2top)
                    # write_float(V_QF3top)
                    # while True:
                    #     output = read()
                    #     if output == "Done.\r\n":
                    #         break
                    #
                    # time.sleep(0.5)
                    # V_EPStop = float(read())

    def stabilize_end():
        nonlocal Stb_flag
        Stb_flag = 0

    def calibration_ring():
        nonlocal V_EPStop, V_EPSbot, R_EPStop, R_EPSbot
        Calib_Vdiff = 0.4
        Calib_rate = 5
        Calib_period = 1 / float(Calib_rate)
        V_EPStop = float(V_EPStop_str.get())
        V_EPSbot = float(V_EPSbot_str.get())
        vmb = VmbSystem.get_instance()
        ExposureTime = input("Exposure time: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    print("--- Instruments Connected ---")
                    print("--- Ring Calibration Start ---")
                    V_EPStop_1 = V_EPStop - Calib_Vdiff
                    V_EPSbot_1 = V_EPSbot - Calib_Vdiff
                    V_EPStop_2 = V_EPStop + Calib_Vdiff
                    V_EPSbot_2 = V_EPSbot + Calib_Vdiff

                    V_EPStop_str.set(str(round(V_EPStop_1, 3)))
                    V_EPSbot_str.set(str(round(V_EPSbot_1, 3)))
                    tk_root.update()
                    update_voltage()
                    time.sleep(0.1)
                    [_, _, _, _, PZ_top1, PZ_bot1] = DLC_Scan_From2Centers(dlc, cam)
                    time.sleep(0.2)

                    V_EPStop_str.set(str(round(V_EPStop_2, 3)))
                    V_EPSbot_str.set(str(round(V_EPSbot_2, 3)))
                    tk_root.update()
                    update_voltage()
                    time.sleep(0.1)
                    [_, _, _, _, PZ_top2, PZ_bot2] = DLC_Scan_From2Centers(dlc, cam)
                    time.sleep(0.2)

                    V_EPStop_str.set(str(round(V_EPStop, 3)))
                    V_EPSbot_str.set(str(round(V_EPSbot, 3)))
                    tk_root.update()
                    update_voltage()
                    time.sleep(0.1)

                    # Data processing
                    R_EPStop = (PZ_top2 - PZ_top1) / (2 * Calib_Vdiff)
                    R_EPSbot = (PZ_bot2 - PZ_bot1) / (2 * Calib_Vdiff)
                    print([R_EPStop, R_EPSbot])
                    print("--- Ring Calibration Finished ---")

    def Detuning_correction():
        PZ_detuning_mzmtop2 = (V_MZMtop2 - V_MZMtop2_ref) * LUT_Slope_MZMtop2
        PZ_detuning_pstop2 = (V_PStop2 - V_PStop2_ref) * LUT_Slope_PStop2
        PZ_detuning_pstop1 = (V_PStop1 - V_PStop1_ref) * LUT_Slope_PStop1
        PZ_detuning = PZ_detuning_mzmtop2 + PZ_detuning_pstop2 + PZ_detuning_pstop1

        # This is a experience formula, which works pretty well as far as I see
        if sum(array([V_MZMtop2, V_PStop2, V_PStop1]) < array([V_MZMtop2_ref, V_PStop2_ref, V_PStop1_ref])) > 1:
            PZ_detuning = PZ_detuning / sqrt(2)

        Vdiff_EPStop = PZ_detuning / R_EPStop
        Vdiff_EPSbot = PZ_detuning / R_EPSbot
        V_EPStop_str.set(str(round(V_EPStop_ref - Vdiff_EPStop, 3)))
        V_EPSbot_str.set(str(round(V_EPSbot_ref - Vdiff_EPSbot, 3)))
        update_voltage()
        tk_root.update()

    def stabilize_laser2():
        nonlocal V_EPStop, V_EPSbot, Stb_flag
        Stb_flag = 1
        Stb_rate = 5
        Stb_period = 1 / float(Stb_rate)
        V_EPStop = float(V_EPStop_str.get())
        vmb = VmbSystem.get_instance()
        ExposureTime = input("Exposure time: ")
        Stb_fraction = input("Stabilization reference: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    print("--- Instruments Connected ---")

                    # Firstly do a wide scan with PZ
                    print("--- Piezo-scan Calibration ---")
                    # PZ_c = dlc.laser1.dl.pc.voltage_act.get()
                    [piezo_calib, fraction_calib, I_max, I_min, PZ_top, PZ_bot] = DLC_Scan_From2Centers(dlc, cam)
                    V_EPStop = V_EPStop - (PZ_top - 70) / R_EPStop
                    V_EPSbot = V_EPSbot - (PZ_bot - 70) / R_EPSbot
                    V_EPStop_str.set(str(round(V_EPStop, 3)))
                    V_EPSbot_str.set(str(round(V_EPSbot, 3)))
                    tk_root.update()
                    update_voltage()
                    time.sleep(1.0)
                    dlc.laser1.dl.pc.voltage_set.set(piezo_calib[0])
                    time.sleep(0.1)
                    piezo_ref = ResStb_findPZ(piezo_calib, fraction_calib, Stb_fraction)
                    # dlc.laser1.dl.pc.voltage_set.set(piezo_ref)
                    piezo_v = 70.0 + piezo_ref - PZ_top
                    dlc.laser1.dl.pc.voltage_set.set(piezo_v)
                    time.sleep(0.1)

                    # Then constantly calculate ratio and PZ feedback
                    print("--- Stabilization Start ---")
                    t0 = time.time()
                    Stb_cycle = 1
                    while True:
                        tk_root.update()
                        t1 = time.time()
                        if t1 - t0 < Stb_cycle * Stb_period:
                            continue

                        Stb_cycle = Stb_cycle + 1
                        if Stb_flag == 0:
                            dlc.laser1.dl.pc.voltage_set.set(70.0)
                            return
                        elif Stb_flag == 2:
                            continue

                        photo1, photo2 = Get_Vmb_2Photos(cam)
                        fraction = (sum(photo1) - I_min) / (I_max - I_min)
                        fraction2 = (sum(photo2) - I_min) / (I_max - I_min)
                        if fraction < 0:
                            fraction = 0
                        elif fraction > 1:
                            fraction = 1
                        piezo_new = ResStb_findPZ(piezo_calib, fraction_calib, fraction)
                        piezo_v = piezo_v - (piezo_new - piezo_ref)
                        dlc.laser1.dl.pc.voltage_set.set(piezo_v)
                        print(fraction, fraction2)

    def stabilize_laser3():
        nonlocal V_EPStop, V_EPSbot, Stb_flag
        Stb_flag = 1
        Stb_rate = 7
        Stb_period = 1 / float(Stb_rate)
        V_EPStop = float(V_EPStop_str.get())
        vmb = VmbSystem.get_instance()
        ExposureTime = input("Exposure time: ")
        Stb_fraction = input("Stabilization reference: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    print("--- Instruments Connected ---")

                    # Firstly do a wide scan with PZ
                    print("--- Piezo-scan Calibration ---")
                    # PZ_c = dlc.laser1.dl.pc.voltage_act.get()
                    [piezo_calib, fraction_calib, I_max, I_min, PZ_top, PZ_bot] = DLC_Scan_From2Centers(dlc, cam)
                    V_EPSbot = V_EPSbot - (PZ_bot - PZ_top) / R_EPSbot
                    V_EPSbot_str.set(str(round(V_EPSbot, 3)))
                    tk_root.update()
                    update_voltage()
                    time.sleep(1.0)
                    dlc.laser1.dl.pc.voltage_set.set(piezo_calib[0])
                    time.sleep(0.1)
                    piezo_ref = ResStb_findPZ(piezo_calib, fraction_calib, Stb_fraction)
                    print(piezo_ref)
                    # dlc.laser1.dl.pc.voltage_set.set(piezo_ref)
                    piezo_v = piezo_ref
                    dlc.laser1.dl.pc.voltage_set.set(piezo_v)
                    time.sleep(0.1)

                    # Then constantly calculate ratio and PZ feedback
                    print("--- Stabilization Start ---")
                    t0 = time.time()
                    Stb_cycle = 1
                    while True:
                        tk_root.update()
                        t1 = time.time()
                        if t1 - t0 < Stb_cycle * Stb_period:
                            continue

                        Stb_cycle = Stb_cycle + 1
                        if Stb_flag == 0:
                            dlc.laser1.dl.pc.voltage_set.set(70.0)
                            return
                        elif Stb_flag == 2:
                            continue

                        photo1, photo2 = Get_Vmb_2Photos(cam)
                        fraction = (sum(photo1) - I_min) / (I_max - I_min)
                        fraction2 = (sum(photo2) - I_min) / (I_max - I_min)
                        if fraction < 0:
                            fraction = 0
                        elif fraction > 1:
                            fraction = 1
                        piezo_new = ResStb_findPZ(piezo_calib, fraction_calib, fraction)
                        piezo_v = piezo_v - (piezo_new - piezo_ref)
                        dlc.laser1.dl.pc.voltage_set.set(piezo_v)

                        if piezo_v < 45.0:
                            print("Locking failed!")

                        # print(fraction, fraction2)

    def stabilize_laser3_revision1():
        nonlocal V_EPStop, V_EPSbot, Stb_flag
        Stb_flag = 1
        Stb_rate = 7
        Stb_period = 1 / float(Stb_rate)
        V_EPStop = float(V_EPStop_str.get())
        vmb = VmbSystem.get_instance()
        ExposureTime = input("Exposure time: ")
        Stb_fraction = input("Stabilization reference: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    print("--- Instruments Connected ---")

                    # Firstly do a wide scan with PZ
                    print("--- Piezo-scan Calibration ---")
                    # PZ_c = dlc.laser1.dl.pc.voltage_act.get()
                    [piezo_calib, fraction_calib, I_max, I_min, PZ_top] = DLC_Scan_FromCenter_revision1(dlc, cam)
                    tk_root.update()
                    update_voltage()
                    time.sleep(1.0)
                    dlc.laser1.dl.pc.voltage_set.set(piezo_calib[0])
                    time.sleep(0.1)
                    piezo_ref = ResStb_findPZ(piezo_calib, fraction_calib, Stb_fraction)
                    print(piezo_ref)
                    # dlc.laser1.dl.pc.voltage_set.set(piezo_ref)
                    piezo_v = piezo_ref
                    dlc.laser1.dl.pc.voltage_set.set(piezo_v)
                    time.sleep(0.1)

                    # Then constantly calculate ratio and PZ feedback
                    print("--- Stabilization Start ---")
                    t0 = time.time()
                    Stb_cycle = 1
                    while True:
                        tk_root.update()
                        t1 = time.time()
                        if t1 - t0 < Stb_cycle * Stb_period:
                            continue

                        Stb_cycle = Stb_cycle + 1
                        if Stb_flag == 0:
                            dlc.laser1.dl.pc.voltage_set.set(70.0)
                            return
                        elif Stb_flag == 2:
                            continue

                        photo1, photo2 = Get_Vmb_2Photos(cam)
                        fraction = (sum(photo1) - I_min) / (I_max - I_min)
                        if fraction < 0:
                            fraction = 0
                        elif fraction > 1:
                            fraction = 1
                        piezo_new = ResStb_findPZ(piezo_calib, fraction_calib, fraction)
                        piezo_v = piezo_v - (piezo_new - piezo_ref)
                        dlc.laser1.dl.pc.voltage_set.set(piezo_v)

                        if piezo_v < 45.0:
                            print("Locking failed!")

                        # print(fraction, fraction2)

    def stabilize_laser_align():
        nonlocal V_EPStop, V_EPSbot
        V_EPStop = float(V_EPStop_str.get())
        vmb = VmbSystem.get_instance()
        ExposureTime = 30
        # ExposureTime = input("Exposure time: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    [_, _, _, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers(dlc, cam, dlc_step=0.4)
                    V_EPSbot = V_EPSbot - (PZ_bot - PZ_top) / R_EPSbot
                    V_EPSbot_str.set(str(round(V_EPSbot, 3)))
                    tk_root.update()
                    update_voltage()

    def Classic_Tx():
        write("5")
        # print("Write Message: ")
        Msg = input("Write Message: ")
        write(Msg)
        while True:
            output = read()
            if output == "Done.\r\n":
                break

    def GPC0():
        # This GPC0 is used to: find out the correct MZMtop1 value for 10 and 11 states
        write("4")
        while True:
            output = read()
            if output == "Finished.\r\n":
                break

        # Read sweeping data - dS12
        write("1")
        time.sleep(1)
        N_alpha_12 = int(read())
        N_beta_12 = int(read())
        V_alpha = zeros(N_alpha_12 * N_beta_12, dtype=float)
        V_beta = zeros(N_alpha_12 * N_beta_12, dtype=float)
        dS12 = zeros(N_alpha_12 * N_beta_12, dtype=float)
        for i in range(N_alpha_12):
            for j in range(N_beta_12):
                V_alpha[i * N_beta_12 + j] = float(read())
                V_beta[i * N_beta_12 + j] = float(read())
                dS12[i * N_beta_12 + j] = float(read())

        V_alpha_12 = reshape(V_alpha, [N_alpha_12, N_beta_12])
        V_beta_12 = reshape(V_beta, [N_alpha_12, N_beta_12])
        dS12 = reshape(dS12, [N_alpha_12, N_beta_12])

        # Read sweeping data - dS34
        write("1")
        time.sleep(1)
        N_alpha_34 = int(read())
        N_beta_34 = int(read())
        V_alpha = zeros(N_alpha_34 * N_beta_34, dtype=float)
        V_beta = zeros(N_alpha_34 * N_beta_34, dtype=float)
        dS34 = zeros(N_alpha_34 * N_beta_34, dtype=float)
        for i in range(N_alpha_34):
            for j in range(N_beta_34):
                V_alpha[i * N_beta_34 + j] = float(read())
                V_beta[i * N_beta_34 + j] = float(read())
                dS34[i * N_beta_34 + j] = float(read())

        dS34 = reshape(dS34, [N_alpha_34, N_beta_34])

        # Data analysis - find V_MZMtop1 that makes VIS34 minimized
        V_alpha = V_alpha_12[:, 0].flatten()
        VIS12_alpha = (max(dS12, axis=1) - min(dS12, axis=1)) / 2
        VIS34_alpha = (max(dS34, axis=1) - min(dS34, axis=1)) / 2
        # MIN34_alpha = min(abs(dS34), axis=1)
        plt.figure(0)
        plt.plot(V_alpha, VIS12_alpha, label="sweep")
        plt.plot(V_alpha, VIS34_alpha, label="sweep")
        plt.grid(True)
        ax = plt.gca()
        ax.set_ylim([0, 1])

        plt.show()

    def GPC1():
        # region Step 1 - 2D global scanning
        write("2")
        while True:
            output = read()
            if output == "Finished.\r\n":
                break

        # Read sweeping data - dS12
        write("1")
        time.sleep(1)
        N_alpha_12 = int(read())
        N_beta_12 = int(read())
        V_alpha = zeros(N_alpha_12 * N_beta_12, dtype=float)
        V_beta = zeros(N_alpha_12 * N_beta_12, dtype=float)
        dS12 = zeros(N_alpha_12 * N_beta_12, dtype=float)
        for i in range(N_alpha_12):
            for j in range(N_beta_12):
                V_alpha[i * N_beta_12 + j] = float(read())
                V_beta[i * N_beta_12 + j] = float(read())
                dS12[i * N_beta_12 + j] = float(read())

        V_alpha_12 = reshape(V_alpha, [N_alpha_12, N_beta_12])
        V_beta_12 = reshape(V_beta, [N_alpha_12, N_beta_12])
        dS12 = reshape(dS12, [N_alpha_12, N_beta_12])

        # Read sweeping data - dS34
        write("1")
        time.sleep(1)
        N_alpha_34 = int(read())
        N_beta_34 = int(read())
        V_alpha = zeros(N_alpha_34 * N_beta_34, dtype=float)
        V_beta = zeros(N_alpha_34 * N_beta_34, dtype=float)
        dS34 = zeros(N_alpha_34 * N_beta_34, dtype=float)
        for i in range(N_alpha_34):
            for j in range(N_beta_34):
                V_alpha[i * N_beta_34 + j] = float(read())
                V_beta[i * N_beta_34 + j] = float(read())
                dS34[i * N_beta_34 + j] = float(read())

        dS34 = reshape(dS34, [N_alpha_34, N_beta_34])

        # Data analysis - find V_PStop2 that makes VIS34 maximized
        V_alpha = V_alpha_12[:, 0].flatten()
        VIS12_alpha = (max(dS12, axis=1) - min(dS12, axis=1)) / 2
        # VIS34_alpha = (max(dS34, axis=1) - min(dS34, axis=1)) / 2
        MIN34_alpha = min(abs(dS34), axis=1)
        plt.figure(0)
        plt.plot(V_alpha, VIS12_alpha, label="sweep")
        plt.plot(V_alpha, MIN34_alpha, label="sweep")
        plt.grid(True)
        ax = plt.gca()
        ax.set_ylim([0, 1])

        # Look for PStop2 best values
        # idx_alpha0 = argmax(VIS12_alpha)
        try:
            # idx_alpha0_candidates = array(where((VIS12_alpha > GPC1_VIS12_threshold) & (VIS34_alpha > GPC1_VIS34_threshold))).flatten()
            idx_alpha0_candidates = array(
                where((VIS12_alpha >= GPC1_VIS12_threshold) & (MIN34_alpha <= GPC1_MIN34_threshold))).flatten()
            if idx_alpha0_candidates.size == 0: raise RuntimeError
            idx_alpha0 = idx_alpha0_candidates[argmin(abs(V_alpha[idx_alpha0_candidates] - V_PStop2_cent ** 2))]
        except RuntimeError:
            print("GPC failed. No alpha meets the requirement.")
            plt.show()

        V_PStop2_str.set(str(round(sqrt(V_alpha[idx_alpha0]), 3)))

        # # Sine wave fitting for MZMtop2
        # V_beta = V_beta_12[0, :].flatten()
        # dS12_temp = S_12[idx_alpha0]
        # try:
        #     params, params_covariance = optimize.curve_fit(MZM, V_beta, dS12_temp, p0=[1, 0.006, 0, 0.05])
        #     VIS12_f = MZM(V_beta, *params)
        #     if params[0] > 0:
        #         phi0 = 0
        #     if params[0] < 0:
        #         phi0 = pi
        #     V_max1 = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
        #     V_max2 = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi) + pi))
        #     if abs(V_max1 ** 2 - V_PStop2_cent ** 2) <= abs(V_max2 ** 2 - V_PStop2_cent ** 2):
        #         V_max = V_max1
        #     elif abs(V_max1 ** 2 - V_PStop2_cent ** 2) > abs(V_max2 ** 2 - V_PStop2_cent ** 2):
        #         V_max = V_max2
        #     print("Fitting results:" + str(params))
        #     print("Fitted extremes: " + str(round(V_max1, 3)) + ", " + str(round(V_max2, 3)))
        #     # if sqrt(V_alpha[idx_alpha0]) < V_PStop2_cent:
        #
        #     # print(str(round(V_max, 3)))
        #     plt.figure(1)
        #     plt.plot(V_beta, dS12_temp, label="sweep")
        #     plt.grid(True)
        #     plt.plot(V_beta, VIS12_f, label="fit")
        #     ax = plt.gca()
        #     ax.set_ylim([-1, 1])
        # except RuntimeError:
        #     print("Fitting failed.")

        # Sine wave fitting for MZMtop2
        V_beta = V_beta_12[0, :].flatten()
        dS12_temp = dS12[idx_alpha0]
        dS34_temp = dS34[idx_alpha0]
        plt.figure(1)
        plt.plot(V_beta, dS34_temp, label="sweep")
        plt.plot(V_beta, dS12_temp, label="sweep")
        plt.grid(True)
        try:
            params, params_covariance = optimize.curve_fit(MZM, V_beta, dS34_temp, p0=[1, 0.006, 0, 0.05])
            if abs(params[3]) > abs(params[0]): raise RuntimeError
            VIS34_f = MZM(V_beta, *params)
            if params[0] > 0:
                phi0 = 0
            if params[0] < 0:
                phi0 = pi

            V_med1 = sqrt(1 / params[1] * ((arcsin(-params[3] / params[0]) - params[2]) % (2 * pi)))
            V_med2 = sqrt(1 / params[1] * ((pi - arcsin(-params[3] / params[0]) - params[2]) % (2 * pi)))
            if abs(V_med1 ** 2 - V_MZMtop2_cent ** 2) <= abs(V_med2 ** 2 - V_MZMtop2_cent ** 2):
                V_med = V_med1
            elif abs(V_med1 ** 2 - V_MZMtop2_cent ** 2) > abs(V_med2 ** 2 - V_MZMtop2_cent ** 2):
                V_med = V_med2
            V_MZMtop2_str.set(str(round(V_med, 3)))

            print("Fitting results:" + str(params))
            print("Fitted extremes: " + str(round(V_med1, 3)) + ", " + str(round(V_med2, 3)))
            print(str(round(V_med, 3)))
            # if sqrt(V_alpha[idx_alpha0]) < V_PStop2_cent:

            plt.plot(V_beta, VIS34_f, label="fit")
            ax = plt.gca()
            ax.set_ylim([-1, 1])
        except RuntimeError:
            print("Fitting failed.")
            plt.show()

        plt.show()

    def GPC3():
        # region Step 2 - 1D scanning on V_PStop1 to observe interference
        write("3")
        # write("c")
        while True:
            output = read()
            if output == "GPC.\r\n":
                break

        # Read sweeping data - dS34
        write("1")
        time.sleep(1)
        N_gamma = int(read())
        V_gamma = zeros(N_gamma, dtype=float)
        S_34 = zeros(N_gamma, dtype=float)
        for i in range(N_gamma):
            V_gamma[i] = float(read())
            S_34[i] = float(read())

        # Sine wave fitting for PStop1
        try:
            params, params_covariance = optimize.curve_fit(MZM, V_gamma, S_34, p0=[1, 0.006, 0, 0.5])
            S_34f = MZM(V_gamma, *params)
            if params[0] > 0:
                phi0 = 0
            if params[0] < 0:
                phi0 = pi
            V_max1 = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
            print(V_max1)
            V_max2 = sqrt(1 / params[1] * ((pi / 2 * 3 - params[2] - phi0) % (2 * pi)))
            print(V_max2)
            # if abs(V_max1 ** 2 - V_PStop1_cent ** 2) <= abs(V_max2 ** 2 - V_PStop1_cent ** 2):
            #     V_max = V_max1
            # elif abs(V_max1 ** 2 - V_PStop1_cent ** 2) > abs(V_max2 ** 2 - V_PStop1_cent ** 2):
            #     V_max = V_max2
            if abs(V_max1 - V_PStop1_cent) <= abs(V_max2 - V_PStop1_cent):
                V_max = V_max1
            elif abs(V_max1 - V_PStop1_cent) > abs(V_max2 - V_PStop1_cent):
                V_max = V_max2
            # print("Fitting results:" + str(params))
            # V_PStop1_str.set(str(round(V_max, 3)))
            plt.figure(2)
            plt.plot(V_gamma, S_34, label="sweep")
            plt.grid(True)
            plt.plot(V_gamma, S_34f, label="fit")
            ax = plt.gca()
            ax.set_ylim([-1, 1])
        except RuntimeError:
            print("Fitting failed.")

        plt.show()

    def GPCc():
        # region Step 2 - 1D scanning on V_PStop1 to observe interference
        # write("3")
        write("c")
        while True:
            output = read()
            if output == "GPC.\r\n":
                break

        # Read sweeping data - dS34
        write("1")
        time.sleep(1)
        N_gamma = int(read())
        V_gamma = zeros(N_gamma, dtype=float)
        S_34 = zeros(N_gamma, dtype=float)
        for i in range(N_gamma):
            V_gamma[i] = float(read())
            S_34[i] = float(read())

        # Sine wave fitting for PStop1
        try:
            params, params_covariance = optimize.curve_fit(MZM, V_gamma, S_34, p0=[1, 0.006, 0, 0.5])
            S_34f = MZM(V_gamma, *params)
            if params[0] > 0:
                phi0 = 0
            if params[0] < 0:
                phi0 = pi
            V_max1 = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
            print(V_max1)
            V_max2 = sqrt(1 / params[1] * ((pi / 2 * 3 - params[2] - phi0) % (2 * pi)))
            print(V_max2)
            # if abs(V_max1 ** 2 - V_PStop1_cent ** 2) <= abs(V_max2 ** 2 - V_PStop1_cent ** 2):
            #     V_max = V_max1
            # elif abs(V_max1 ** 2 - V_PStop1_cent ** 2) > abs(V_max2 ** 2 - V_PStop1_cent ** 2):
            #     V_max = V_max2
            if abs(V_max1 - V_PStop1_cent) <= abs(V_max2 - V_PStop1_cent):
                V_max = V_max1
            elif abs(V_max1 - V_PStop1_cent) > abs(V_max2 - V_PStop1_cent):
                V_max = V_max2
            print("Fitting results:" + str(params))
            V_PStop1_str.set(str(round(V_max, 3)))
            plt.figure(2)
            plt.plot(V_gamma, S_34, label="sweep")
            plt.grid(True)
            plt.plot(V_gamma, S_34f, label="fit")
            ax = plt.gca()
            ax.set_ylim([-1, 1])
        except RuntimeError:
            print("Fitting failed.")

        plt.show()

    def RPC():
        # region Step 3+ - Real-time correction
        write("7")
        while True:
            output = read()
            if output == "RPC.\r\n":
                break

        # Read data
        V_PStop2 = float(read())
        V_MZMtop2 = float(read())
        V_PStop1 = float(read())
        V_PStop2_str.set(str(round(V_PStop2, 3)))
        V_MZMtop2_str.set(str(round(V_MZMtop2, 3)))
        V_PStop1_str.set(str(round(V_PStop1, 3)))

    def RPSpre():
        # region RPS record target values
        nonlocal RPS_dS12, RPS_dS34, RPSY_dS12, RPSY_dS34

        write("d")
        while True:
            output = read()
            if output == "RPS.\r\n":
                break

        RPS_dS12 = float(read())
        RPS_dS34 = float(read())
        RPSY_dS12 = float(read())
        RPSY_dS34 = float(read())
        print(str(round(RPS_dS12, 3)) + " " + str(round(RPS_dS34, 3)) + " " + str(round(RPSY_dS12, 3)) + " " + str(round(RPSY_dS34, 3)))

    def QWN():
        QWN_dir = '0'
        t0 = time.time()
        period = 0.25
        t_idx = 0
        t_idx_limit = 30 / period

        T_ch0 = zeros(int(t_idx_limit / 2))
        T_ch1 = zeros(int(t_idx_limit / 2))
        dS12_ch0 = zeros(int(t_idx_limit / 2))
        dS12_ch1 = zeros(int(t_idx_limit / 2))
        dS34_ch0 = zeros(int(t_idx_limit / 2))
        dS34_ch1 = zeros(int(t_idx_limit / 2))

        while True:
            if time.time() < t0 + t_idx * period: continue

            write("8")
            if t_idx % 2 == 0:
                write("0")
            elif t_idx % 2 == 1:
                write("1")
            while True:
                output = read()
                if output == "Header.\r\n":
                    break
            V_PStop2 = float(read())
            V_MZMtop2 = float(read())
            V_PStop2_str.set(str(round(V_PStop2, 3)))
            V_MZMtop2_str.set(str(round(V_MZMtop2, 3)))
            if t_idx % 2 == 0:
                T_ch0[int(t_idx / 2)] = t_idx * period
                dS12_ch0[int(t_idx / 2)] = float(read())
                dS34_ch0[int(t_idx / 2)] = float(read())
            elif t_idx % 2 == 1:
                T_ch1[int(t_idx / 2)] = t_idx * period
                dS12_ch1[int(t_idx / 2)] = float(read())
                dS34_ch1[int(t_idx / 2)] = float(read())

            t_idx = t_idx + 1
            tk_root.update()

            if t_idx == t_idx_limit: break

        T_ref = array([0, t_idx_limit * period])
        dS12_ref = array([0.92, 0.92])
        dS34_ref = array([0.08, 0.08])
        plt.figure(0)
        plt.grid(True)
        plt.plot(T_ch0, dS12_ch0, label="ch0, dS12")
        plt.plot(T_ch0, dS34_ch0, label="ch0, dS34")
        plt.plot(T_ref, dS12_ref, linestyle="--")
        plt.plot(T_ch1, dS12_ch1, label="ch1, dS12")
        plt.plot(T_ch1, dS34_ch1, label="ch1, dS34")
        plt.plot(T_ref, dS34_ref, linestyle="--")
        ax = plt.gca()
        ax.legend(loc='lower left')
        ax.set_ylim([-0.3, 1.1])

        plt.show()

    def QWNC_single():
        QWN_dir = input("Direction: ")
        write("i")
        if QWN_dir == "0":
            write("0")
        elif QWN_dir == "1":
            write("1")

        while True:
            output = read()
            if output == "Header.\r\n":
                break

    def QWNC_random():
        N_switch = 10
        T_switch = (np.random.rand(N_switch) * 0.8 + 0.2) * 0.5
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            write("i")
            if t_idx % 2 == 0:
                write("0")
            elif t_idx % 2 == 1:
                write("1")
            while True:
                output = read()
                if output == "Header.\r\n":
                    break

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNC_period():
        N_switch = 10
        T_switch = np.ones([N_switch, 1]) * 0.02
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            # write("i")
            write("j")
            if t_idx % 2 == 0:
                write("0")
            elif t_idx % 2 == 1:
                write("1")
            # while True:
            #     output = read()
            #     if output == "Header.\r\n":
            #         break

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNC_direct():
        N_switch = 10
        T_switch = np.ones([N_switch, 1]) * 0.2
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            write("j")
            if t_idx % 2 == 0:
                write("0")
            elif t_idx % 2 == 1:
                write("1")

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNQ_period():
        Duration = float(input("Measurement duration (in seconds): "))
        P_switch = float(input("Measurement duration (in seconds): "))
        N_switch = int(Duration / P_switch)
        T_switch = np.ones([N_switch, 1]) * P_switch
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            # write("i")
            write("j")
            if t_idx % 2 == 0:
                write("0")
            elif t_idx % 2 == 1:
                write("1")

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNQ_period_1x2():
        Duration = float(input("Measurement duration (in seconds): "))
        P_switch = float(input("Switching period (in seconds): "))
        N_switch = int(Duration / P_switch)
        T_switch = np.ones([N_switch, 1]) * P_switch
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            write("n")
            if t_idx % 2 == 0:
                # write("0")
                write("5")
            elif t_idx % 2 == 1:
                # write("1")
                write("4")

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNQ_period_2x2():
        Duration = float(input("Measurement duration (in seconds): "))
        P_switch = float(input("Switching period (in seconds): "))
        N_switch = int(Duration / P_switch)
        T_switch = np.ones([N_switch, 1]) * P_switch
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            # write("i")
            write("j")
            if t_idx % 4 == 0:
                write("0")
            elif t_idx % 4 == 1:
                write("1")
            elif t_idx % 4 == 2:
                write("2")
            elif t_idx % 4 == 3:
                write("3")

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNQ_period_2x2x3():
        Duration = float(input("Measurement duration (in seconds): "))
        P_switch = float(input("Switching period (in seconds): "))
        N_switch = int(Duration / P_switch)
        T_switch = np.ones([N_switch, 1]) * P_switch
        T_switch = cumsum(T_switch)
        t0 = time.time()
        t_idx = 0

        while True:
            if time.time() < t0 + T_switch[t_idx]: continue

            write("m")
            if t_idx % 6 == 0:
                write("0")
            elif t_idx % 6 == 1:
                write("1")
            elif t_idx % 6 == 2:
                write("2")
            elif t_idx % 6 == 3:
                write("3")
            elif t_idx % 6 == 4:
                write("4")
            elif t_idx % 6 == 5:
                write("5")

            t_idx = t_idx + 1

            if t_idx == N_switch: break

    def QWNQ_period_1x2():
    #     Duration = float(input("Measurement duration (in seconds): "))
    #     P_switch = float(input("Switching period (in seconds): "))
    #     N_switch = int(Duration / P_switch)
    #     T_switch = np.ones([N_switch, 1]) * P_switch
    #     T_switch = cumsum(T_switch)
    #     t0 = time.time()
    #     t_idx = 0
    #
    #     while True:
    #         if time.time() < t0 + T_switch[t_idx]: continue
    #
    #         # write("i")
    #         write("j")
    #         if t_idx % 2 == 0:
    #             write("0")
    #         elif t_idx % 2 == 1:
    #             write("1")
    #
    #         t_idx = t_idx + 1
    #
    #         if t_idx == N_switch: break

    # def QWNQ_test():
    #     while True:
    #         write("k")
    #         t_idx = int(input("Direction"))
    #         if t_idx == 0:
    #             write("0")
    #         elif t_idx == 1:
    #             write("1")
    #         elif t_idx == 2:
    #             break
        pass

    def RPCtrace():
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)

        # Create tkinter window
        RPC_root = Tk()
        RPC_root.state('zoomed')
        RPC_root.wm_title("RPC trace")
        # Configure row and column weights
        RPC_root.grid_rowconfigure(0, weight=0)
        RPC_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPC_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPC_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPC_root.grid_columnconfigure(1, weight=1)
        RPC_root.grid_columnconfigure(2, weight=1)
        RPC_root.grid_columnconfigure(3, weight=1)

        # region Create text label
        dHV = StringVar()
        dHV.set('0')
        dHV_text = Label(master=RPC_root, textvariable=dHV, fg='red', font=('Arial', 16, 'bold'))
        dHV_text.grid(row=0, column=0)
        dDA = StringVar()
        dDA.set('0')
        dDA_text = Label(master=RPC_root, textvariable=dDA, fg='red', font=('Arial', 16, 'bold'))
        dDA_text.grid(row=0, column=1)
        dHV_AVG = StringVar()
        dHV_AVG.set('0')
        dHV_AVG_text = Label(master=RPC_root, textvariable=dHV_AVG, fg='red', font=('Arial', 16, 'bold'))
        dHV_AVG_text.grid(row=0, column=2)
        dDA_AVG = StringVar()
        dDA_AVG.set('0')
        dDA_AVG_text = Label(master=RPC_root, textvariable=dDA_AVG, fg='red', font=('Arial', 16, 'bold'))
        dDA_AVG_text.grid(row=0, column=3)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPC_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(5, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPC_root.quit()
            RPC_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPC_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 at H input')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 at H input')
        line3, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 at D input')
        line4, = ax.plot([], [], linewidth=1.0, label='Error=4%', linestyle="--")
        line5, = ax.plot([], [], linewidth=1.0, label='Error=4%', linestyle="--")
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        ax.set_ylim(-0.2, 1.1)

        t1 = time.time()
        t_idx = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("7")
            while True:
                output = read()
                if output == "RPC.\r\n":
                    break

            V_PStop2 = float(read())
            V_MZMtop2 = float(read())
            V_PStop1 = float(read())
            # print(str(V_PStop2) + " " + str(V_MZMtop2))
            V_PStop2_str.set(str(round(V_PStop2, 3)))
            V_MZMtop2_str.set(str(round(V_MZMtop2, 3)))
            V_PStop1_str.set(str(round(V_PStop1, 3)))
            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())
            Sig[2, -1] = float(read())
            Sig[3, -1] = 0.92
            Sig[4, -1] = 0.08
            # Sig[2, -1] = 0.90
            # Sig[3, -1] = 0.10

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            dHV.set("dHV: " + str(float(Sig[0, -1])))
            dDA.set("dDA: " + str(float(Sig[1, -1])))
            dHV_AVG.set("dHV: " + str(mean(Sig[0, -NumBins_real:-1])))
            dDA_AVG.set("dDA: " + str(mean(Sig[1, -NumBins_real:-1])))
            # print(str(round(mean(Sig[0, -NumBins_real:-1]), 3))
            #       + " ", str(round(mean(Sig[1, -NumBins_real:-1]), 3))
            #       + " ", str(round(mean(Sig[2, -NumBins_real:-1]), 3)))
            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            line3.set_data(Time[-NumBins_real:-1], Sig[2, -NumBins_real:-1])
            line4.set_data(Time[-NumBins_real:-1], Sig[3, -NumBins_real:-1])
            line5.set_data(Time[-NumBins_real:-1], Sig[4, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPC_root.after(next_call, update)
            # RPC_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPC_root.mainloop()

    def RPStrace():
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)

        # Create tkinter window
        RPS_root = Tk()
        RPS_root.state('zoomed')
        RPS_root.wm_title("RPS trace")
        # Configure row and column weights
        RPS_root.grid_rowconfigure(0, weight=0)
        RPS_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPS_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPS_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPS_root.grid_columnconfigure(1, weight=1)
        RPS_root.grid_columnconfigure(2, weight=1)
        RPS_root.grid_columnconfigure(3, weight=1)

        # region Create text label
        dHV = StringVar()
        dHV.set('0')
        dHV_text = Label(master=RPS_root, textvariable=dHV, fg='red', font=('Arial', 16, 'bold'))
        dHV_text.grid(row=0, column=0)
        dDA = StringVar()
        dDA.set('0')
        dDA_text = Label(master=RPS_root, textvariable=dDA, fg='red', font=('Arial', 16, 'bold'))
        dDA_text.grid(row=0, column=1)
        dHV_AVG = StringVar()
        dHV_AVG.set('0')
        dHV_AVG_text = Label(master=RPS_root, textvariable=dHV_AVG, fg='red', font=('Arial', 16, 'bold'))
        dHV_AVG_text.grid(row=0, column=2)
        dDA_AVG = StringVar()
        dDA_AVG.set('0')
        dDA_AVG_text = Label(master=RPS_root, textvariable=dDA_AVG, fg='red', font=('Arial', 16, 'bold'))
        dDA_AVG_text.grid(row=0, column=3)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPS_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(6, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPS_root.quit()
            RPS_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPS_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / H')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / H')
        line3, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / D')
        line4, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / H (target)', linestyle="--")
        line5, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / H (target)', linestyle="--")
        line6, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / D (target)', linestyle="--")
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        ax.set_ylim(-1.1, 1.1)

        t1 = time.time()
        t_idx = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("g")
            while True:
                output = read()
                if output == "RPS.\r\n":
                    break

            V_PStop2 = float(read())
            V_MZMtop2 = float(read())
            V_PStop1 = float(read())
            V_PStop2_str.set(str(round(V_PStop2, 3)))
            V_MZMtop2_str.set(str(round(V_MZMtop2, 3)))
            V_PStop1_str.set(str(round(V_PStop1, 3)))
            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())
            Sig[2, -1] = float(read())
            Sig[3, -1] = RPS_dS12
            Sig[4, -1] = RPS_dS34
            Sig[5, -1] = RPSY_dS34

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            dHV.set("dHV: " + str(float(Sig[0, -1])))
            dDA.set("dDA: " + str(float(Sig[1, -1])))
            dHV_AVG.set("dHV: " + str(mean(Sig[0, -NumBins_real:-1])))
            dDA_AVG.set("dDA: " + str(mean(Sig[1, -NumBins_real:-1])))
            # print(str(round(mean(Sig[0, -NumBins_real:-1]), 3))
            #       + " ", str(round(mean(Sig[1, -NumBins_real:-1]), 3))
            #       + " ", str(round(mean(Sig[2, -NumBins_real:-1]), 3)))
            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            line3.set_data(Time[-NumBins_real:-1], Sig[2, -NumBins_real:-1])
            line4.set_data(Time[-NumBins_real:-1], Sig[3, -NumBins_real:-1])
            line5.set_data(Time[-NumBins_real:-1], Sig[4, -NumBins_real:-1])
            line6.set_data(Time[-NumBins_real:-1], Sig[5, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPS_root.after(next_call, update)
            # RPS_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPS_root.mainloop()

    def RPShtrace():
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)

        # Create tkinter window
        RPS_root = Tk()
        RPS_root.state('zoomed')
        RPS_root.wm_title("RPS trace")
        # Configure row and column weights
        RPS_root.grid_rowconfigure(0, weight=0)
        RPS_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPS_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPS_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPS_root.grid_columnconfigure(1, weight=1)
        RPS_root.grid_columnconfigure(2, weight=1)
        RPS_root.grid_columnconfigure(3, weight=1)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPS_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(8, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPS_root.quit()
            RPS_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPS_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / H', color='cornflowerblue')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / H', color='darkorange')
        line3, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / D', color='limegreen')
        line4, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / D', color='crimson')
        line5, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / H (target)', linestyle="--", color='cornflowerblue')
        line6, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / H (target)', linestyle="--", color='darkorange')
        line7, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / D (target)', linestyle="--", color='limegreen')
        line8, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / D (target)', linestyle="--", color='crimson')
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        ax.set_ylim(-1.1, 1.1)

        t1 = time.time()
        t_idx = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("h")
            while True:
                output = read()
                if output == "RPS.\r\n":
                    break

            V_PStop2 = float(read())
            V_MZMtop2 = float(read())
            V_PStop1 = float(read())
            V_PStop2_str.set(str(round(V_PStop2, 3)))
            V_MZMtop2_str.set(str(round(V_MZMtop2, 3)))
            V_PStop1_str.set(str(round(V_PStop1, 3)))
            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())
            Sig[2, -1] = float(read())
            Sig[3, -1] = float(read())
            Sig[4, -1] = RPS_dS12
            Sig[5, -1] = RPS_dS34
            Sig[6, -1] = RPSY_dS12
            Sig[7, -1] = RPSY_dS34

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            line3.set_data(Time[-NumBins_real:-1], Sig[2, -NumBins_real:-1])
            line4.set_data(Time[-NumBins_real:-1], Sig[3, -NumBins_real:-1])
            line5.set_data(Time[-NumBins_real:-1], Sig[4, -NumBins_real:-1])
            line6.set_data(Time[-NumBins_real:-1], Sig[5, -NumBins_real:-1])
            line7.set_data(Time[-NumBins_real:-1], Sig[6, -NumBins_real:-1])
            line8.set_data(Time[-NumBins_real:-1], Sig[7, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPS_root.after(next_call, update)
            # RPS_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPS_root.mainloop()

    def RPShtrace_monitor():
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)
        MonitorDuration = float(input("Monitor duration (s): "))
        MonitorPeriod = float(input("Monitor period (s): "))
        NumMonitor = int(MonitorDuration / MonitorPeriod)
        IndexMonitor = 0
        MonitorSaveDir = r'C:\Yichi Zhang\Quantum Hybrid Networks\SystemControl\TxController_v32_Tx5Rdev4\RPSdata.txt'

        # Create tkinter window
        RPS_root = Tk()
        RPS_root.state('zoomed')
        RPS_root.wm_title("RPS trace")
        # Configure row and column weights
        RPS_root.grid_rowconfigure(0, weight=0)
        RPS_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPS_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPS_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPS_root.grid_columnconfigure(1, weight=1)
        RPS_root.grid_columnconfigure(2, weight=1)
        RPS_root.grid_columnconfigure(3, weight=1)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPS_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(8, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        Monitor = zeros(shape=(9, NumMonitor), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPS_root.quit()
            RPS_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPS_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / H', color='cornflowerblue')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / H', color='darkorange')
        line3, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / D', color='limegreen')
        line4, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / D', color='crimson')
        line5, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / H (target)', linestyle="--", color='cornflowerblue')
        line6, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / H (target)', linestyle="--", color='darkorange')
        line7, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / D (target)', linestyle="--", color='limegreen')
        line8, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / D (target)', linestyle="--", color='crimson')
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        ax.set_ylim(-1.1, 1.1)

        t1 = time.time()
        t_idx = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx, Monitor, IndexMonitor
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("h")
            while True:
                output = read()
                if output == "RPS.\r\n":
                    break

            V_PStop2 = float(read())
            V_MZMtop2 = float(read())
            V_PStop1 = float(read())
            V_PStop2_str.set(str(round(V_PStop2, 3)))
            V_MZMtop2_str.set(str(round(V_MZMtop2, 3)))
            V_PStop1_str.set(str(round(V_PStop1, 3)))
            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())
            Sig[2, -1] = float(read())
            Sig[3, -1] = float(read())
            Sig[4, -1] = RPS_dS12
            Sig[5, -1] = RPS_dS34
            Sig[6, -1] = RPSY_dS12
            Sig[7, -1] = RPSY_dS34

            if dt >= IndexMonitor * MonitorPeriod and IndexMonitor < NumMonitor:
                write("e")
                while True:
                    output = read()
                    if output == "RPS.\r\n":
                        break
                Monitor[0, IndexMonitor] = time.time() - t1
                Monitor[1, IndexMonitor] = RPS_dS12
                Monitor[2, IndexMonitor] = float(read())
                Monitor[3, IndexMonitor] = RPS_dS34
                Monitor[4, IndexMonitor] = float(read())
                Monitor[5, IndexMonitor] = RPSY_dS12
                Monitor[6, IndexMonitor] = float(read())
                Monitor[7, IndexMonitor] = RPSY_dS34
                Monitor[8, IndexMonitor] = float(read())
                IndexMonitor = IndexMonitor + 1
                if IndexMonitor >= NumMonitor:
                    file = open(MonitorSaveDir, 'w')
                    for k in range(NumMonitor):
                        for j in range(9):
                            file.write(str(Monitor[j, k]))
                            if j != 8: file.write(',')
                            elif j == 8: file.write('\n')
                    file.write('END')
                    file.close()

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            line3.set_data(Time[-NumBins_real:-1], Sig[2, -NumBins_real:-1])
            line4.set_data(Time[-NumBins_real:-1], Sig[3, -NumBins_real:-1])
            line5.set_data(Time[-NumBins_real:-1], Sig[4, -NumBins_real:-1])
            line6.set_data(Time[-NumBins_real:-1], Sig[5, -NumBins_real:-1])
            line7.set_data(Time[-NumBins_real:-1], Sig[6, -NumBins_real:-1])
            line8.set_data(Time[-NumBins_real:-1], Sig[7, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPS_root.after(next_call, update)
            # RPS_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPS_root.mainloop()

    def RPShtrace_monitor2():
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)
        MonitorDuration = float(input("Monitor duration (s): "))
        MonitorPeriod = float(input("Monitor period (s): "))
        NumMonitor = int(MonitorDuration / MonitorPeriod)
        IndexMonitor = 0
        MonitorSaveDir = r'C:\Yichi Zhang\Quantum Hybrid Networks\SystemControl\TxController_v32_Tx5Rdev4\RPSdata2.txt'

        # Create tkinter window
        RPS_root = Tk()
        RPS_root.state('zoomed')
        RPS_root.wm_title("RPS trace")
        # Configure row and column weights
        RPS_root.grid_rowconfigure(0, weight=0)
        RPS_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPS_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPS_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPS_root.grid_columnconfigure(1, weight=1)
        RPS_root.grid_columnconfigure(2, weight=1)
        RPS_root.grid_columnconfigure(3, weight=1)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPS_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(8, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        Monitor = zeros(shape=(9, NumMonitor), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPS_root.quit()
            RPS_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPS_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / H', color='cornflowerblue')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / H', color='darkorange')
        line3, = ax.plot([], [], linewidth=1.5, label='PD1-PD2 / D', color='limegreen')
        line4, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / D', color='crimson')
        line5, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / H (target)', linestyle="--", color='cornflowerblue')
        line6, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / H (target)', linestyle="--", color='darkorange')
        line7, = ax.plot([], [], linewidth=1.0, label='PD1-PD2 / D (target)', linestyle="--", color='limegreen')
        line8, = ax.plot([], [], linewidth=1.0, label='PD3-PD4 / D (target)', linestyle="--", color='crimson')
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        ax.set_ylim(-1.1, 1.1)

        t1 = time.time()
        t_idx = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx, Monitor, IndexMonitor
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("e")
            while True:
                output = read()
                if output == "RPS.\r\n":
                    break

            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())
            Sig[2, -1] = float(read())
            Sig[3, -1] = float(read())
            Sig[4, -1] = RPS_dS12
            Sig[5, -1] = RPS_dS34
            Sig[6, -1] = RPSY_dS12
            Sig[7, -1] = RPSY_dS34

            if dt >= IndexMonitor * MonitorPeriod and IndexMonitor < NumMonitor:
                write("e")
                while True:
                    output = read()
                    if output == "RPS.\r\n":
                        break
                Monitor[0, IndexMonitor] = time.time() - t1
                Monitor[1, IndexMonitor] = RPS_dS12
                Monitor[2, IndexMonitor] = float(read())
                Monitor[3, IndexMonitor] = RPS_dS34
                Monitor[4, IndexMonitor] = float(read())
                Monitor[5, IndexMonitor] = RPSY_dS12
                Monitor[6, IndexMonitor] = float(read())
                Monitor[7, IndexMonitor] = RPSY_dS34
                Monitor[8, IndexMonitor] = float(read())
                IndexMonitor = IndexMonitor + 1
                if IndexMonitor >= NumMonitor:
                    file = open(MonitorSaveDir, 'w')
                    for k in range(NumMonitor):
                        for j in range(9):
                            file.write(str(Monitor[j, k]))
                            if j != 8: file.write(',')
                            elif j == 8: file.write('\n')
                    file.write('END')
                    file.close()

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            line3.set_data(Time[-NumBins_real:-1], Sig[2, -NumBins_real:-1])
            line4.set_data(Time[-NumBins_real:-1], Sig[3, -NumBins_real:-1])
            line5.set_data(Time[-NumBins_real:-1], Sig[4, -NumBins_real:-1])
            line6.set_data(Time[-NumBins_real:-1], Sig[5, -NumBins_real:-1])
            line7.set_data(Time[-NumBins_real:-1], Sig[6, -NumBins_real:-1])
            line8.set_data(Time[-NumBins_real:-1], Sig[7, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPS_root.after(next_call, update)
            # RPS_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPS_root.mainloop()

    def RPShtrace_monitor3():
        # Only record intensity
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)
        MonitorDuration = float(input("Monitor duration (s): "))
        MonitorPeriod = float(input("Monitor period (s): "))
        NumMonitor = int(MonitorDuration / MonitorPeriod)
        IndexMonitor = 0
        MonitorSaveDir = r'C:\Yichi Zhang\Quantum Hybrid Networks\SystemControl\TxController_v33_Tx5Rdev4\RPSdata3.txt'

        # Create tkinter window
        RPS_root = Tk()
        RPS_root.state('zoomed')
        RPS_root.wm_title("RPS trace")
        # Configure row and column weights
        RPS_root.grid_rowconfigure(0, weight=0)
        RPS_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPS_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPS_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPS_root.grid_columnconfigure(1, weight=1)
        RPS_root.grid_columnconfigure(2, weight=1)
        RPS_root.grid_columnconfigure(3, weight=1)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPS_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(2, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        Monitor = zeros(shape=(3, NumMonitor), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPS_root.quit()
            RPS_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPS_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD3+PD4 / H', color='cornflowerblue')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / H', color='darkorange')
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        # ax.set_ylim(-1.1, 1.1)
        ax.set_ylim(0, 4100)

        t1 = time.time()
        t_idx = 0
        Sig00 = 0
        Sig01 = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx, Monitor, IndexMonitor, Sig00, Sig01
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("f")
            while True:
                output = read()
                if output == "RPS.\r\n":
                    break

            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())

            if dt >= IndexMonitor * MonitorPeriod and IndexMonitor < NumMonitor:
                write("f")
                while True:
                    output = read()
                    if output == "RPS.\r\n":
                        break
                Monitor[0, IndexMonitor] = time.time() - t1
                Sig3 = float(read())
                Sig4 = float(read())
                if IndexMonitor == 0:
                    Sig00 = Sig3
                    Sig01 = Sig4
                Monitor[1, IndexMonitor] = (Sig3 + Sig4) / (Sig00 + Sig01)
                Monitor[2, IndexMonitor] = abs(Sig3 - Sig4) / (Sig3 + Sig4)
                IndexMonitor = IndexMonitor + 1
                if IndexMonitor >= NumMonitor:
                    file = open(MonitorSaveDir, 'w')
                    for k in range(NumMonitor):
                        for j in range(3):
                            file.write(str(Monitor[j, k]))
                            if j != 2: file.write(',')
                            elif j == 2: file.write('\n')
                    file.write('END')
                    file.close()

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPS_root.after(next_call, update)
            # RPS_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPS_root.mainloop()

    def RPShtrace_monitor4():
        # Only record intensity
        HistorySize = 180
        RepeatRate = 0.1
        NumBins = int(HistorySize / RepeatRate)
        MonitorDuration = float(input("Monitor duration (s): "))
        MonitorPeriod = float(input("Monitor period (s): "))
        NumMonitor = int(MonitorDuration / MonitorPeriod)
        IndexMonitor = 0
        MonitorSaveDir = r'C:\Yichi Zhang\Quantum Hybrid Networks\SystemControl\TxController_v33_Tx5Rdev4\RPSdata4.txt'

        # Create tkinter window
        RPS_root = Tk()
        RPS_root.state('zoomed')
        RPS_root.wm_title("RPS trace")
        # Configure row and column weights
        RPS_root.grid_rowconfigure(0, weight=0)
        RPS_root.grid_rowconfigure(1, weight=1)  # Make the canvas row resizable
        RPS_root.grid_rowconfigure(2, weight=0)  # Toolbar row (fixed size)
        RPS_root.grid_columnconfigure(0, weight=1)  # Allow all columns to resize equally
        RPS_root.grid_columnconfigure(1, weight=1)
        RPS_root.grid_columnconfigure(2, weight=1)
        RPS_root.grid_columnconfigure(3, weight=1)

        # Create canvas
        fig = Figure(figsize=(12, 5), dpi=100)
        canvas = FigureCanvasTkAgg(fig, master=RPS_root)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Create array
        # global Sig, Time, NumBins_real
        Sig = zeros(shape=(2, NumBins), dtype=float)
        Time = zeros(shape=(NumBins), dtype=float)
        Monitor = zeros(shape=(3, NumMonitor), dtype=float)
        NumBins_real = 0

        # Define button functions
        def on_key_press(event):
            print("you pressed {}".format(event.key))
            key_press_handler(event, canvas, toolbar)

        def _quit():
            RPS_root.quit()
            RPS_root.destroy()

        # Create toolbar
        toolbar_frame = Frame(RPS_root)
        toolbar_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        canvas.mpl_connect("key_press_event", on_key_press)

        QuitButton = Button(master=toolbar_frame, text="Quit", command=_quit, font=('Arial', 12))
        QuitButton.pack(side="right", padx=5)
        ax = fig.add_subplot(111)
        line1, = ax.plot([], [], linewidth=1.5, label='PD3+PD4 / H', color='cornflowerblue')
        line2, = ax.plot([], [], linewidth=1.5, label='PD3-PD4 / H', color='darkorange')
        ax.set(xlabel='time(s)', ylabel='Signal')
        ax.legend(loc='lower left')
        ax.set_ylim(-1.1, 1.1)
        # ax.set_ylim(0, 4100)

        t1 = time.time()
        t_idx = 0

        # Create update function
        def update():
            # tag1 = time.time()
            nonlocal Sig, Time, NumBins_real, t1, t_idx, Monitor, IndexMonitor
            Sig = roll(Sig, shift=-1, axis=1)
            Time = roll(Time, shift=-1)
            t2 = time.time()
            dt = t2 - t1
            t_idx = t_idx + 1

            write("g")
            while True:
                output = read()
                if output == "RPS.\r\n":
                    break

            Sig[0, -1] = float(read())
            Sig[1, -1] = float(read())

            if dt >= IndexMonitor * MonitorPeriod and IndexMonitor < NumMonitor:
                write("g")
                while True:
                    output = read()
                    if output == "RPS.\r\n":
                        break
                Monitor[0, IndexMonitor] = time.time() - t1
                Monitor[1, IndexMonitor] = float(read())
                Monitor[2, IndexMonitor] = float(read())
                IndexMonitor = IndexMonitor + 1
                if IndexMonitor >= NumMonitor:
                    file = open(MonitorSaveDir, 'w')
                    for k in range(NumMonitor):
                        for j in range(3):
                            file.write(str(Monitor[j, k]))
                            if j != 2: file.write(',')
                            elif j == 2: file.write('\n')
                    file.write('END')
                    file.close()

            Time[-1] = float(dt)
            if NumBins_real < NumBins:
                NumBins_real = NumBins_real + 1

            line1.set_data(Time[-NumBins_real:-1], Sig[0, -NumBins_real:-1])
            line2.set_data(Time[-NumBins_real:-1], Sig[1, -NumBins_real:-1])
            ax.relim()
            ax.autoscale_view()
            canvas.draw()
            toolbar.update()

            # Self Update Instruction
            # Detuning_correction()
            next_call = t1 + t_idx * RepeatRate - time.time()
            next_call = max(array([1, int(next_call * 1000)]))  # Convert to ms, ensure non-negative
            RPS_root.after(next_call, update)
            # RPS_root.after(int(RepeatRate*1000), update)
            # print(time.time() - tag1)

        # Formally start the counting process
        update()
        RPS_root.mainloop()

    def PC_calib():
        write("2")
        while True:
            output = read()
            if output == "Finished.\r\n":
                break

        Fig_idx = 0
        while Fig_idx < 2:
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N1 = int(read())
            N2 = int(read())
            V_alpha = zeros(N1 * N2, dtype=float)
            V_beta = zeros(N1 * N2, dtype=float)
            S = zeros(N1 * N2, dtype=float)
            for i in range(N1):
                for j in range(N2):
                    V_alpha[i * N2 + j] = float(read())
                    V_beta[i * N2 + j] = float(read())
                    S[i * N2 + j] = float(read())

            V_alpha = reshape(V_alpha, [N1, N2])
            V_beta = reshape(V_beta, [N1, N2])
            S = reshape(S, [N1, N2])
            fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
            surf = ax.plot_surface(V_alpha, V_beta, S, cmap=cm.coolwarm, linewidth=0, antialiased=False)

            if Fig_idx != 2:
                ax.set_zlim(-1.01, 1.01)
            ax.zaxis.set_major_locator(LinearLocator(10))
            # A StrMethodFormatter is used automatically
            ax.zaxis.set_major_formatter('{x:.02f}')

            # Add a color bar which maps values to colors.
            fig.colorbar(surf, shrink=0.5, aspect=5)
            Fig_idx = Fig_idx + 1

        plt.show()

    def LUT_calib_test1():
    #     # V_ref = V_MZMtop2
    #     # V_ref = V_PStop1
    #     # V_ref = V_PStop2
    #     V_ref = V_MZMbot
    #     # V_ref = V_PSbot
    #     # V_arr = sqrt(arange(start=100, stop=900, step=LUT_calib_dV2))
    #     # V_arr = sqrt(arange(start=100, stop=1000, step=200))
    #     V_arr = sqrt(arange(start=V_ref ** 2 - 400, stop=V_ref ** 2 + 600, step=200))
    #     N_arr = len(V_arr)
    #     PZ_top_arr = zeros(N_arr)
    #     PZ_bot_arr = zeros(N_arr)
    #
    #     vmb = VmbSystem.get_instance()
    #     ExposureTime = input("Exposure time: ")
    #     print("--- Connecting ---")
    #
    #     with vmb:
    #         cams = vmb.get_all_cameras()
    #         with cams[0] as cam:
    #             cam.set_pixel_format(PixelFormat.Mono14)
    #             cam.ExposureTime.set(ExposureTime)
    #             with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
    #                 print("--- Instruments Connected ---")
    #
    #                 for i in range(N_arr):
    #                     print(V_arr[i] ** 2)
    #                     # V_PStop2_str.set(str(round(V_arr[i], 3)))
    #                     # V_PStop1_str.set(str(round(V_arr[i], 3)))
    #                     # V_PSbot_str.set(str(round(V_arr[i], 3)))
    #                     # V_MZMtop2_str.set(str(round(V_arr[i], 3)))
    #                     V_MZMbot_str.set(str(round(V_arr[i], 3)))
    #                     update_voltage()
    #                     tk_root.update()
    #                     time.sleep(0.1)
    #                     [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY_LUT(dlc, cam, dlc_range=25,
    #                                                                                        dlc_step=0.25)
    #                     dlc.laser1.dl.pc.voltage_set.set(70)
    #                     time.sleep(0.1)
    #                     PZ_top_arr[i] = PZ_top
    #                     PZ_bot_arr[i] = PZ_bot
    #                 # V_PStop2_str.set(str(round(V_ref, 3)))
    #                 # V_PStop1_str.set(str(round(V_ref, 3)))
    #                 # V_PSbot_str.set(str(round(V_ref, 3)))
    #                 # V_MZMtop2_str.set(str(round(V_ref, 3)))
    #                 V_MZMbot_str.set(str(round(V_ref, 3)))
    #                 update_voltage()
    #                 time.sleep(0.1)
    #
    #                 plt.figure(0)
    #                 plt.grid(True)
    #                 plt.plot(V_arr, PZ_top_arr, label="TOP")
    #                 plt.plot(V_arr, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #
    #                 plt.figure(1)
    #                 plt.grid(True)
    #                 plt.plot(V_arr ** 2, PZ_top_arr, label="TOP")
    #                 plt.plot(V_arr ** 2, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #                 plt.show()
    #
    #                 # print(V_arr)
    #                 # print(PZ_top_arr)
    #                 # print(PZ_bot_arr)
    #
    # def LUT_calib_test2():
    #     V_ref1 = V_MZMtop2
    #     V_ref2 = V_MZMbot
    #     V1_arr = sqrt(arange(start=V_ref1 ** 2 - 400, stop=V_ref1 ** 2 + 600, step=200))
    #     V2_arr = sqrt(arange(start=V_ref2 ** 2 - 400, stop=V_ref2 ** 2 + 600, step=200))
    #     N_arr = len(V1_arr)
    #     PZ_top_arr = zeros(N_arr)
    #     PZ_bot_arr = zeros(N_arr)
    #
    #     vmb = VmbSystem.get_instance()
    #     ExposureTime = input("Exposure time: ")
    #     print("--- Connecting ---")
    #
    #     with vmb:
    #         cams = vmb.get_all_cameras()
    #         with cams[0] as cam:
    #             cam.set_pixel_format(PixelFormat.Mono14)
    #             cam.ExposureTime.set(ExposureTime)
    #             with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
    #                 print("--- Instruments Connected ---")
    #
    #                 for i in range(N_arr):
    #                     V_MZMtop2_str.set(str(round(V1_arr[i], 3)))
    #                     # V_PSbot_str.set(str(round(V1_arr[i], 3)))
    #                     V_MZMbot_str.set(str(round(V2_arr[i], 3)))
    #                     update_voltage()
    #                     time.sleep(0.1)
    #                     # [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY(dlc, cam, dlc_range=20,
    #                     #                                                                dlc_step=0.2)
    #                     [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY_LUT(dlc, cam, dlc_range=30,
    #                                                                                        dlc_step=0.3)
    #                     dlc.laser1.dl.pc.voltage_set.set(70)
    #                     time.sleep(0.2)
    #                     tk_root.update()
    #                     PZ_top_arr[i] = PZ_top
    #                     PZ_bot_arr[i] = PZ_bot
    #                 V_MZMtop2_str.set(str(round(V_ref1, 3)))
    #                 # V_PSbot_str.set(str(round(V_ref1, 3)))
    #                 V_MZMbot_str.set(str(round(V_ref2, 3)))
    #                 update_voltage()
    #                 time.sleep(0.1)
    #
    #                 plt.figure(0)
    #                 plt.grid(True)
    #                 plt.plot(V1_arr, PZ_top_arr, label="TOP")
    #                 plt.plot(V1_arr, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #
    #                 plt.figure(1)
    #                 plt.grid(True)
    #                 plt.plot(V1_arr ** 2, PZ_top_arr, label="TOP")
    #                 plt.plot(V1_arr ** 2, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #                 plt.show()
    #
        pass

    def LUT_calib_test3():
    #     V_ref1 = V_MZMtop2
    #     V_ref2 = V_PStop2
    #     V_ref3 = V_PStop1
    #     # V_arr = sqrt(arange(start=100, stop=900, step=LUT_calib_dV2))
    #     V_arr = sqrt(arange(start=100, stop=1000, step=100))
    #     N_arr = len(V_arr)
    #     PZ_top_arr = zeros(N_arr)
    #     PZ_bot_arr = zeros(N_arr)
    #
    #     vmb = VmbSystem.get_instance()
    #     ExposureTime = input("Exposure time: ")
    #     print("--- Connecting ---")
    #
    #     with vmb:
    #         cams = vmb.get_all_cameras()
    #         with cams[0] as cam:
    #             cam.set_pixel_format(PixelFormat.Mono14)
    #             cam.ExposureTime.set(ExposureTime)
    #             with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
    #                 print("--- Instruments Connected ---")
    #
    #                 for i in range(N_arr):
    #                     V_MZMtop2_str.set(str(round(V_arr[i], 3)))
    #                     V_PStop2_str.set(str(round(V_arr[i], 3)))
    #                     V_PStop1_str.set(str(round(V_arr[i], 3)))
    #                     # V_PSbot_str.set(str(round(V_arr[i], 3)))
    #                     # V_MZMbot_str.set(str(round(V_arr[i], 3)))
    #                     update_voltage()
    #                     tk_root.update()
    #                     time.sleep(0.1)
    #                     # [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY(dlc, cam, dlc_range=20,
    #                     #                                                                dlc_step=0.2)
    #                     [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY_LUT(dlc, cam, dlc_range=25,
    #                                                                                        dlc_step=0.25)
    #                     dlc.laser1.dl.pc.voltage_set.set(70)
    #                     time.sleep(0.1)
    #                     tk_root.update()
    #                     PZ_top_arr[i] = PZ_top
    #                     PZ_bot_arr[i] = PZ_bot
    #                 V_MZMtop2_str.set(str(round(V_ref1, 3)))
    #                 V_PStop2_str.set(str(round(V_ref2, 3)))
    #                 V_PStop1_str.set(str(round(V_ref3, 3)))
    #                 update_voltage()
    #                 time.sleep(0.1)
    #
    #                 plt.figure(0)
    #                 plt.grid(True)
    #                 plt.plot(V_arr, PZ_top_arr, label="TOP")
    #                 plt.plot(V_arr, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #
    #                 plt.figure(1)
    #                 plt.grid(True)
    #                 plt.plot(V_arr ** 2, PZ_top_arr, label="TOP")
    #                 plt.plot(V_arr ** 2, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #                 plt.show()
    #
    #                 print(V_arr)
    #                 print(PZ_top_arr - 70)
    #                 print(PZ_bot_arr - 70)
    #
    # def LUT_calib_test4():
    #     V_ref1 = V_MZMtop2
    #     V_ref2 = V_PStop2
    #     V_ref3 = V_PStop1
    #     V_ref4 = V_MZMbot
    #     # V_arr = sqrt(arange(start=100, stop=900, step=LUT_calib_dV2))
    #     V_arr = sqrt(arange(start=100, stop=1000, step=100))
    #     N_arr = len(V_arr)
    #     PZ_top_arr = zeros(N_arr)
    #     PZ_bot_arr = zeros(N_arr)
    #
    #     vmb = VmbSystem.get_instance()
    #     ExposureTime = input("Exposure time: ")
    #     print("--- Connecting ---")
    #
    #     with vmb:
    #         cams = vmb.get_all_cameras()
    #         with cams[0] as cam:
    #             cam.set_pixel_format(PixelFormat.Mono14)
    #             cam.ExposureTime.set(ExposureTime)
    #             with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
    #                 print("--- Instruments Connected ---")
    #
    #                 for i in range(N_arr):
    #                     V_MZMtop2_str.set(str(round(V_arr[i], 3)))
    #                     V_PStop2_str.set(str(round(V_arr[i], 3)))
    #                     V_PStop1_str.set(str(round(V_arr[i], 3)))
    #                     V_MZMbot_str.set(str(round(V_arr[i], 3)))
    #                     # V_MZMbot_str.set(str(round(V_arr[i], 3)))
    #                     update_voltage()
    #                     tk_root.update()
    #                     time.sleep(0.1)
    #                     # [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY(dlc, cam, dlc_range=20,
    #                     #                                                                dlc_step=0.2)
    #                     [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY_LUT(dlc, cam, dlc_range=25,
    #                                                                                        dlc_step=0.25)
    #                     dlc.laser1.dl.pc.voltage_set.set(70)
    #                     time.sleep(0.1)
    #                     tk_root.update()
    #                     PZ_top_arr[i] = PZ_top
    #                     PZ_bot_arr[i] = PZ_bot
    #                 V_MZMtop2_str.set(str(round(V_ref1, 3)))
    #                 V_PStop2_str.set(str(round(V_ref2, 3)))
    #                 V_PStop1_str.set(str(round(V_ref3, 3)))
    #                 V_MZMbot_str.set(str(round(V_ref4, 3)))
    #                 update_voltage()
    #                 time.sleep(0.1)
    #
    #                 plt.figure(0)
    #                 plt.grid(True)
    #                 plt.plot(V_arr, PZ_top_arr, label="TOP")
    #                 plt.plot(V_arr, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #
    #                 plt.figure(1)
    #                 plt.grid(True)
    #                 plt.plot(V_arr ** 2, PZ_top_arr, label="TOP")
    #                 plt.plot(V_arr ** 2, PZ_bot_arr, label="BOT")
    #                 ax = plt.gca()
    #                 ax.legend(loc='upper left')
    #                 plt.show()
    #
    #                 print(V_arr)
    #                 print(PZ_top_arr - 70)
    #                 print(PZ_bot_arr - 70)
        pass

    def LUT_SingleScan():
        vmb = VmbSystem.get_instance()
        ExposureTime = 30
        # ExposureTime = input("Exposure time: ")
        print("--- Connecting ---")

        with vmb:
            cams = vmb.get_all_cameras()
            with cams[0] as cam:
                cam.set_pixel_format(PixelFormat.Mono14)
                cam.ExposureTime.set(ExposureTime)
                with DLCpro(NetworkConnection("192.168.1.200")) as dlc:
                    print("--- Instruments Connected ---")
                    tk_root.update()
                    time.sleep(0.1)
                    [_, _, _, _, PZ_top, PZ_bot] = DLC_Scan_From2Centers_noDISPLAY_LUT(dlc, cam, dlc_range=25,
                                                                                       dlc_step=0.25)
                    dlc.laser1.dl.pc.voltage_set.set(70)
                    time.sleep(0.1)

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
    Quit2Button = Button(master=tk_root, text="Quit2", command=_quit2, font=('Arial', 18))
    Quit2Button.pack(side=RIGHT, fill='both', expand=False)
    UpdateButton = Button(master=tk_root, text="Update", command=update_voltage, font=('Arial', 18))
    UpdateButton.pack(side=RIGHT, fill='both', expand=False)
    StbStartButton = Button(master=tk_root, text="Lock", command=stabilize_laser3_revision1, font=('Arial', 18))
    StbStartButton.pack(side=LEFT, fill='both', expand=False)
    # RPCVButton = Button(master=tk_root, text="RPCV", command=RPC_voltage, font=('Arial', 18))
    # RPCVButton.pack(side=LEFT, fill='both', expand=False)
    # RPSVButton = Button(master=tk_root, text="RPSV", command=RPS_voltage, font=('Arial', 18))
    # RPSVButton.pack(side=LEFT, fill='both', expand=False)
    # StbStartButton = Button(master=tk_root, text="Align", command=stabilize_laser_align, font=('Arial', 18))
    # StbStartButton.pack(side=LEFT, fill='both', expand=False)
    StbEndButton = Button(master=tk_root, text="UnLock", command=stabilize_end, font=('Arial', 18))
    StbEndButton.pack(side=LEFT, fill='both', expand=False)
    # CalibRingButton = Button(master=tk_root, text="EPScalib", command=calibration_ring, font=('Arial', 18))
    # CalibRingButton.pack(side=LEFT, fill='both', expand=False)
    # CTxButton = Button(master=tk_root, text="SendMsg", command=Classic_Tx, font=('Arial', 18))
    # CTxButton.pack(side=LEFT, fill='both', expand=False)
    # GPCMButton = Button(master=tk_root, text="GPCM", command=GPCM, font=('Arial', 18))
    # GPCMButton.pack(side=LEFT, fill='both', expand=False)
    # GPC0Button = Button(master=tk_root, text="GPC0", command=GPC0, font=('Arial', 18))
    # GPC0Button.pack(side=LEFT, fill='both', expand=False)
    # GPC1Button = Button(master=tk_root, text="GPC1", command=GPC1, font=('Arial', 18))
    # GPC1Button.pack(side=LEFT, fill='both', expand=False)
    # GPC3Button = Button(master=tk_root, text="GPC3", command=GPC3, font=('Arial', 18))
    # GPC3Button.pack(side=LEFT, fill='both', expand=False)
    # GPCcButton = Button(master=tk_root, text="GPCc", command=GPCc, font=('Arial', 18))
    # GPCcButton.pack(side=LEFT, fill='both', expand=False)
    # RPCButton = Button(master=tk_root, text="RPC", command=RPC, font=('Arial', 18))
    # RPCButton.pack(side=LEFT, fill='both', expand=False)
    # RPCtraceButton = Button(master=tk_root, text="RPCt", command=RPCtrace, font=('Arial', 18))
    # RPCtraceButton.pack(side=LEFT, fill='both', expand=False)
    # RPCtrace2Button = Button(master=tk_root, text="RPCt", command=RPCtrace, font=('Arial', 18))
    # RPCtrace2Button.pack(side=LEFT, fill='both', expand=False)
    # RPSpreButton = Button(master=tk_root, text="RPSp", command=RPSpre, font=('Arial', 18))
    # RPSpreButton.pack(side=LEFT, fill='both', expand=False)
    # RPStraceButton = Button(master=tk_root, text="RPSt", command=RPStrace, font=('Arial', 18))
    # RPStraceButton.pack(side=LEFT, fill='both', expand=False)
    # RPShtraceButton = Button(master=tk_root, text="RPSht", command=RPShtrace, font=('Arial', 18))
    # RPShtraceButton.pack(side=LEFT, fill='both', expand=False)
    # RPShtmButton = Button(master=tk_root, text="RPSm", command=RPShtrace_monitor, font=('Arial', 18))
    # RPShtmButton.pack(side=LEFT, fill='both', expand=False)
    # RPShtm2Button = Button(master=tk_root, text="RPSm2", command=RPShtrace_monitor2, font=('Arial', 18))
    # RPShtm2Button.pack(side=LEFT, fill='both', expand=False)
    # RPShtm3Button = Button(master=tk_root, text="RPSm3", command=RPShtrace_monitor3, font=('Arial', 18))
    # RPShtm3Button.pack(side=LEFT, fill='both', expand=False)
    # RPShtm4Button = Button(master=tk_root, text="RPSm4", command=RPShtrace_monitor4, font=('Arial', 18))
    # RPShtm4Button.pack(side=LEFT, fill='both', expand=False)
    # QWNButton = Button(master=tk_root, text="QWN", command=QWN, font=('Arial', 18))
    # QWNButton.pack(side=LEFT, fill='both', expand=False)
    # QWNCButton = Button(master=tk_root, text="QWNC", command=QWNC_period, font=('Arial', 18))
    # QWNCButton.pack(side=LEFT, fill='both', expand=False)
    QWNCButton = Button(master=tk_root, text="QWNC", command=QWNC_direct, font=('Arial', 18))
    QWNCButton.pack(side=LEFT, fill='both', expand=False)
    QWNQ12Button = Button(master=tk_root, text="QWNQ12", command=QWNQ_period_1x2, font=('Arial', 18))
    QWNQ12Button.pack(side=LEFT, fill='both', expand=False)
    QWNQ22Button = Button(master=tk_root, text="QWNQ22", command=QWNQ_period_2x2, font=('Arial', 18))
    QWNQ22Button.pack(side=LEFT, fill='both', expand=False)
    # QWNQ12Button = Button(master=tk_root, text="QWNQ12", command=QWNQ_period_1x2, font=('Arial', 18))
    # QWNQ12Button.pack(side=LEFT, fill='both', expand=False)
    QWNQ223Button = Button(master=tk_root, text="QWNQ223", command=QWNQ_period_2x2x3, font=('Arial', 18))
    QWNQ223Button.pack(side=LEFT, fill='both', expand=False)
    # testButton = Button(master=tk_root, text="test", command=QWNQ_test, font=('Arial', 18))
    # testButton.pack(side=LEFT, fill='both', expand=False)
    # PCCalibButton = Button(master=tk_root, text="PCcalib", command=PC_calib, font=('Arial', 18))
    # PCCalibButton.pack(side=LEFT, fill='both', expand=False)
    # LUT1CalibButton = Button(master=tk_root, text="LUT1", command=LUT_calib_test1, font=('Arial', 18))
    # LUT1CalibButton.pack(side=LEFT, fill='both', expand=False)
    # LUT2CalibButton = Button(master=tk_root, text="LUT2", command=LUT_calib_test2, font=('Arial', 18))
    # LUT2CalibButton.pack(side=LEFT, fill='both', expand=False)
    # LUT = Button(master=tk_root, text="LUT", command=LUT_SingleScan, font=('Arial', 18))
    # LUT.pack(side=LEFT, fill='both', expand=False)
    # DetuningCorrectButton = Button(master=tk_root, text="DC", command=Detuning_correction, font=('Arial', 18))
    # DetuningCorrectButton.pack(side=LEFT, fill='both', expand=False)
    # tdtButton = Button(master=tk_root, text="TDT", command=TimeDelayTest, font=('Arial', 18))
    # tdtButton.pack(side=LEFT, fill='both', expand=False)

    # StbPauseButton = Button(master=tk_root, text="StbPause", command=stabilize_pause, font=('Arial', 18))
    # StbPauseButton.pack(side=LEFT, fill='both', expand=False)
    # endregion
    # endregion

    mainloop()


if __name__ == '__main__':

    Fig_idx = 0
    step = 10

    # region Step 1
    if step == 1:
        time.sleep(1)
        write("1")

        input("Tune to QF4 resonance.")
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break
        time.sleep(1)
        # output = read()
        # print(output)

        while (Fig_idx < 4):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [0, 1, 2, 3]:
                try:
                    params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                    Sf = MZM(V, *params)
                    if params[0] > 0:
                        phi0 = 0
                    if params[0] < 0:
                        phi0 = pi
                    V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                    V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                    V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                    print("Fitting results:", params)
                    print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                    plt.plot(V, Sf, label="fit")
                except RuntimeError:
                    print("Fitting failed.")
            Fig_idx = Fig_idx + 1
        plt.show()
        step = 2
        Fig_idx = 0
    # endregion

    # region Step 2
    if step == 2:
        time.sleep(1)
        write("2")

        input("Tune to QF2top resonance.")
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 1):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [0]:
                try:
                    params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                    Sf = MZM(V, *params)
                    if params[0] > 0:
                        phi0 = 0
                    if params[0] < 0:
                        phi0 = pi
                    V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                    V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                    V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                    print("Fitting results:", params)
                    print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                    plt.plot(V, Sf, label="fit")
                except RuntimeError:
                    print("Fitting failed.")
            Fig_idx = Fig_idx + 1
        plt.show()
        step = 3
        Fig_idx = 0
    # endregion

    # region Step 3
    if step == 3:
        time.sleep(1)
        write("3")

        input("Tune to QF3top resonance.")
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 1):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [0]:
                try:
                    params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                    Sf = MZM(V, *params)
                    if params[0] > 0:
                        phi0 = 0
                    if params[0] < 0:
                        phi0 = pi
                    V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                    V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                    V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                    print("Fitting results:", params)
                    print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                    plt.plot(V, Sf, label="fit")
                except RuntimeError:
                    print("Fitting failed.")
            Fig_idx = Fig_idx + 1
        plt.show()
        step = 4
        Fig_idx = 0
    # endregion

    # region Step 7
    if step == 7:

        # region ITU P
        time.sleep(1)
        write("7")

        input("Tune to ITU P.")
        time.sleep(2)
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 2):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [99]:
                params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                Sf = MZM(V, *params)
                if params[0] > 0:
                    phi0 = 0
                if params[0] < 0:
                    phi0 = pi
                V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                print("Fitting results:", params)
                print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                plt.plot(V, Sf, label="fit")
            Fig_idx = Fig_idx + 1
        plt.show()
        Fig_idx = 0
        # endregion

        # region ITU I
        input("Tune to ITU I.")
        time.sleep(2)
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 2):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [99]:
                params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                Sf = MZM(V, *params)
                if params[0] > 0:
                    phi0 = 0
                if params[0] < 0:
                    phi0 = pi
                V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                print("Fitting results:", params)
                print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                plt.plot(V, Sf, label="fit")
            Fig_idx = Fig_idx + 1
        plt.show()
        Fig_idx = 0
        # endregion

        # region ITU S
        input("Tune to ITU S.")
        time.sleep(2)
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 2):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [99]:
                params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                Sf = MZM(V, *params)
                if params[0] > 0:
                    phi0 = 0
                if params[0] < 0:
                    phi0 = pi
                V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                print("Fitting results:", params)
                print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                plt.plot(V, Sf, label="fit")
            Fig_idx = Fig_idx + 1
        plt.show()
        Fig_idx = 0
        # endregion

    # endregion

    # region Step 8
    if step == 8:

        # region ITU P
        time.sleep(1)
        write("8")

        input("Tune to ITU P.")
        time.sleep(2)
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 2):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [99]:
                params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                Sf = MZM(V, *params)
                if params[0] > 0:
                    phi0 = 0
                if params[0] < 0:
                    phi0 = pi
                V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                print("Fitting results:", params)
                print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                plt.plot(V, Sf, label="fit")
            Fig_idx = Fig_idx + 1
        plt.show()
        Fig_idx = 0
        # endregion

        # region ITU I
        input("Tune to ITU I.")
        time.sleep(2)
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 2):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [99]:
                params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                Sf = MZM(V, *params)
                if params[0] > 0:
                    phi0 = 0
                if params[0] < 0:
                    phi0 = pi
                V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                print("Fitting results:", params)
                print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                plt.plot(V, Sf, label="fit")
            Fig_idx = Fig_idx + 1
        plt.show()
        Fig_idx = 0
        # endregion

        # region ITU S
        input("Tune to ITU S.")
        time.sleep(2)
        write("0")
        while True:
            output = read()
            if output == "Done.\r\n":
                break

        while (Fig_idx < 2):
            write("1")
            time.sleep(1)
            print(Fig_idx)
            N = int(read())
            print(float(read()), float(read()), float(read()), sep=", ")
            V = zeros(N, dtype=float)
            S = zeros(N, dtype=float)
            for i in range(N):
                V[i] = float(read())
                S[i] = float(read())
            plt.figure(Fig_idx)
            plt.plot(V, S, label="sweep")
            plt.grid(True)
            if Fig_idx in [99]:
                params, params_covariance = optimize.curve_fit(MZM, V, S, p0=[1600, 0.0075, 0, 800])
                Sf = MZM(V, *params)
                if params[0] > 0:
                    phi0 = 0
                if params[0] < 0:
                    phi0 = pi
                V_maxf = sqrt(1 / params[1] * ((pi / 2 - params[2] - phi0) % (2 * pi)))
                V_minf = sqrt(1 / params[1] * ((3 * pi / 2 - params[2] - phi0) % (2 * pi)))
                V_medf = sqrt(1 / params[1] * ((0 - params[2] - phi0) % pi))
                print("Fitting results:", params)
                print("Fitted extremes", round(V_maxf, 3), round(V_minf, 3), round(V_medf, 3), sep=", ")
                plt.plot(V, Sf, label="fit")
            Fig_idx = Fig_idx + 1
        plt.show()
        Fig_idx = 0
        # endregion

    # endregion

    # region Step 10 - Hybrid Transmitter
    if step == 10:
        time.sleep(1)
        write("9")
        while True:
            output = read()
            if output == "Ready.\r\n":
                break

        Hybrid_Control()
