import time

from serial_utils import open_arduino, readline_str, wait_for_line, write_line


class RxController:
    def __init__(self, ser, config):
        self.ser = ser
        self.config = config

    def sync_board_parameters(self):
        desired = self.config.rx_board_params()
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

    def stream_start(self):
        write_line(self.ser, "STREAM_START")

    def stream_stop(self):
        write_line(self.ser, "STREAM_STOP")

    def decode_mode(self):
        write_line(self.ser, "DECODE_MODE")

    def reconnect(self):
        connection = self.config.connection()
        try:
            self.ser.close()
        except Exception:
            pass
        time.sleep(0.5)
        self.ser = open_arduino(connection["rx_port"], connection["baudrate"], connection["timeout"])
        wait_for_line(self.ser, "RX Ready.", timeout_s=10.0)
        return self.ser
