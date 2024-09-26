# Yachtonomous Control Algorithm
# Control algo ensures boat follows the path to reach the destination
from algo.navigation import navigation
from driver.servo import sail, rudder
from driver.ui import ui


class sailboat:
    def __init__(self, boat_sensor_pin:int, dest_sensor_pin:int, wind_sensor_pin:int, 
                 sail_servo_pin:int, rudder_servo_pin:int, switch_pin:int, *led_pins:int) -> None:
        self.__nav    = navigation(boat_sensor_pin, dest_sensor_pin, wind_sensor_pin)
        self.__sail   = sail(sail_servo_pin)
        self.__rudder = rudder(rudder_servo_pin)
        self.__ui     = ui(switch_pin, *led_pins)

    @property
    def nav(self) -> navigation:
        return self.__nav
    
    @property
    def sail(self) -> sail:
        return self.__sail
    
    @property
    def rudder(self) -> rudder:
        return self.__rudder
    
    @property
    def ui(self) -> ui:
        return self.__ui