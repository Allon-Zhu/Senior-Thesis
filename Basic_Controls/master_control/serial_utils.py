import time

import serial


def open_arduino(port, baud=115200, timeout=0.2):
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def readline_str(ser):
    try:
        return ser.readline().decode("utf-8", errors="ignore")
    except Exception:
        return ""


def write_line(ser, value):
    if not value.endswith("\n"):
        value += "\n"
    ser.write(value.encode("utf-8"))


def write_float_line(ser, value):
    ser.write(f"{float(value)}\r\n".encode("utf-8"))
    time.sleep(0.02)


def wait_for_line(ser, expected=None, timeout_s=10.0):
    start = time.time()
    while time.time() - start <= timeout_s:
        line = readline_str(ser).strip()
        if not line:
            continue
        if expected is None or line == expected:
            return line
    raise TimeoutError(f"Timed out waiting for {expected or 'serial line'}")


def read_exact(ser, n_bytes, timeout_s=30.0):
    start = time.time()
    data = bytearray()
    while len(data) < n_bytes:
        if time.time() - start > timeout_s:
            raise TimeoutError(f"Timeout while reading {n_bytes} bytes")
        chunk = ser.read(n_bytes - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)
