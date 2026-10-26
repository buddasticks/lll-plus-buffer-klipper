# ~/klipper/klippy/extras/lll_autoload.py
# Sends auto-load settings from printer.cfg to an LLL Buffer Plus MCU.
# Uses add_config_cmd, so the settings are sent during MCU startup,
# before the printer is ready. No gcode runs at boot.

class LLLAutoLoad:
    def __init__(self, config):
        self.printer = config.get_printer()
        mcu_name = config.get('mcu')
        self.enable = config.getboolean('auto_load', False)
        self.length = config.getint('load_length', 100, minval=1, maxval=500)
        self.speed_x10 = int(round(
            config.getfloat('load_speed', 33.1, above=0.0) * 10.0))
        mcu = self.printer.lookup_object('mcu ' + mcu_name)
        mcu.add_config_cmd(
            "lll_autoload enable=%d length_mm=%d speed_x10=%d"
            % (1 if self.enable else 0, self.length, self.speed_x10))

def load_config_prefix(config):
    return LLLAutoLoad(config)
