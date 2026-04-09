import json
from pathlib import Path


class ConfigManager:
    def __init__(self, path=None):
        self.path = Path(path or Path(__file__).with_name("config").joinpath("device_config.json"))
        self.data = self.load()

    def load(self):
        with self.path.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def reload(self):
        self.data = self.load()
        return self.data

    def save(self):
        with self.path.open("w", encoding="utf-8") as handle:
            json.dump(self.data, handle, indent=2, sort_keys=True)
            handle.write("\n")

    def connection(self):
        return self.data["connection"]

    def tx_channels(self):
        return list(self.data["tx"]["channels"])

    def tx_board_params(self):
        return dict(self.data["tx"].get("board_params", {}))

    def rx_board_params(self):
        return dict(self.data["rx"].get("board_params", {}))

    def tx_gui_ranges(self):
        return dict(self.data["tx"].get("gui_ranges", {}))

    def update_tx_voltage(self, name, voltage):
        for channel in self.data["tx"]["channels"]:
            if channel["name"] == name:
                channel["voltage"] = float(voltage)
                self.save()
                return
        raise KeyError(f"Unknown TX channel: {name}")

    def set_board_param(self, board, key, value):
        self.data[board].setdefault("board_params", {})[key] = value
        self.save()

    def delete_board_param(self, board, key):
        params = self.data[board].setdefault("board_params", {})
        if key in params:
            del params[key]
            self.save()
