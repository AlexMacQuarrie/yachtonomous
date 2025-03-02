# External
from machine import Pin, time_pulse_us
from micropython import const
import time
import math


# Consts
_SAIL_PIN        = const(14)
_SAIL_OFFSET_DEG = const(0.0)
_WIND_PIN        = const(12)
_WIND_OFFSET_DEG = const(0.0)
pi, tau          = math.pi, 2.0*math.pi


def _wrap_to_pi(angle:float):
    ''' Wrap angles to [-pi, pi] '''
    return (angle + pi) % (tau) - pi


class rotation_sensor:
    def __init__(self, pin:int, offset_deg:float):
        ''' Instantiate PWM pin and offset '''
        self.__pwm_pin    = Pin(pin, Pin.IN)
        self.__offset_rad = _wrap_to_pi(math.radians(offset_deg))

    def read_angle(self) -> float:
        ''' Read angle from a rotation sensor PWM signal'''
        # Time duty cycle
        high_time  = time_pulse_us(self.__pwm_pin, 1)
        low_time   = time_pulse_us(self.__pwm_pin, 0)
        duty_cycle = high_time/(high_time+low_time)

        # Convert to angle in radians
        angle_rad = tau*duty_cycle

        # Subtract offset and wrap
        angle_rad = _wrap_to_pi(angle_rad-self.__offset_rad)

        return angle_rad


class rotation_sensors:
    def __init__(self, sail_pin:int, sail_offset_deg:float, 
                       wind_pin:int, wind_offset_deg:float):
        ''' Instantiate roation sensor objects '''
        self.__sail_sensor = rotation_sensor(pin=sail_pin, offset_deg=sail_offset_deg)
        self.__wind_sensor = rotation_sensor(pin=wind_pin, offset_deg=wind_offset_deg)

    def read_angles(self) -> float:
        ''' Read wind and sail angles. Wind angle is relative to sail angle '''
        sail_angle = self.__sail_sensor.read_angle()
        wind_angle = _wrap_to_pi(self.__wind_sensor.read_angle()-sail_angle)
        return sail_angle, wind_angle

    def read_avg_angles(self, num_samples:int, delay_us:int) -> float:
        ''' Get angles averages over a number of samples '''
        sail_angle, wind_angle = 0, 0

        for _ in range(num_samples):
            new_sail_angle, new_wind_angle = self.read_angles()
            sail_angle += new_sail_angle
            wind_angle += new_wind_angle
            time.sleep_us(delay_us)

        sail_angle /= num_samples
        wind_angle /= num_samples
        return sail_angle, wind_angle
    

# Global object for sensors
wind_and_sail_sensors = rotation_sensors(sail_pin=_SAIL_PIN, sail_offset_deg=_SAIL_OFFSET_DEG,
                                         wind_pin=_WIND_PIN, wind_offset_deg=_WIND_OFFSET_DEG)
