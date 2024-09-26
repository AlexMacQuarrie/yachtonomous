# User Interface Driver
# Responsible for reading switch inputs and driving LEDs
from tools.io_base import io_base


class led(io_base):
    def turn_on(self) -> None:
        # TODO: Write self._pin high
        pass

    def turn_off(self) -> None:
        # TODO: Write self._pin low
        pass


class switch(io_base):
    def is_on(self) -> bool:
        # TODO: return self._pin voltage/status
        pass


class ui:
    def __init__(self, switch_pin:int, *led_pins:int) -> None:
        self.__switch = switch(switch_pin)
        self.__leds   = [led(led_pin) for led_pin in led_pins]
        self.__max_led_index = len(self.__leds)-1

    def switch_is_on(self) -> bool:
        return self.__switch.is_on()
    
    def turn_on_led(self, led_idx:int) -> None:
        if led_idx > self.__max_led_index or led_idx < 0:
            raise ValueError('Invalid LED index')
        self.__leds[led_idx].turn_on()

    def turn_off_led(self, led_idx:int) -> None:
        if led_idx > self.__max_led_index or led_idx < 0:
            raise ValueError('Invalid LED index')
        self.__leds[led_idx].turn_off()