# Positioning & Localization Driver
# Responsible for reading position sensor inputs and translating to meaningfull representation
from tools.io_base import io_base


class position_sensor(io_base):
    def get_position(self) -> tuple:
        # TODO: Read from self._pin
        #       Translate & output (x,y) tuple
        pass