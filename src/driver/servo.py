# Servo Driver
# Responsible for driving the rudder and sail servos
from tools.io_base import io_base


class servo(io_base):
    def __init__(self, pin:int) -> None:
        super(servo, self).__init__(pin)
        self._current_angle = None

    @property
    def current_angle(self) -> int:
        return self._current_angle

    def set_angle(self, angle:int) -> None:
        # TODO: Write angle to self._pin (0 is centerline, -90 is port, 90 is starboard)
        self._current_angle = angle
        print(f'Setting angle to {angle}')
        pass


class rudder(servo):
    def scull(self, center_angle:float, cycles:int=5, scull_angle:float=15) -> None:
        self.set_angle(center_angle)
        for i in range(cycles):
            if i % 2:
                angle = center_angle + scull_angle
            else:
                angle = center_angle - scull_angle

            self.set_angle(angle)

            # TODO: Add some delay?
        self.set_angle(center_angle)


class sail(servo):
    def trim(self, wind_angle:float) -> None:
        sail_angle = wind_angle/2.0  # Just a guess for now
        self.set_angle(sail_angle)

    def backwind(self, port_starboard_en:bool) -> None:
        if port_starboard_en:
            self.set_angle(-90)
        else:
            self.set_angle(90)

