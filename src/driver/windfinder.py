# Wind Direction Sesnor Driver
# Responsible for reading wind sensor inputs and translating to meaningfull representation
from tools.io_base import io_base


class wind_sensor(io_base):
    def get_angle(self) -> float:
        # TODO: Read from self.__pin
        #       Translate & output wind angle (relative to boat centerline)
        pass