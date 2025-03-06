# External
from machine import Pin, PWM
from micropython import const
import math


# Consts
_RUDDER_PIN = const(10)
_SAIL_PIN   = const(11)
_PWM_FREQ   = const(50)
_MIN_DUTY   = const(1000)  #   0 deg
_MID_DUTY   = const(4550)  #  90 deg
_MAX_DUTY   = const(7500)  # 180 deg
_90_DEG     = const(90)


class servo:
    def __init__(self, pin:int, pwm_freq:int):
        ''' Instantiate PWM pin '''
        self.__pwm_pin = PWM(Pin(pin), freq=pwm_freq)

    def set_angle(self, angle_radians:float) -> None:
        ''' Set the angle for a servo, input should be [-pi/2, pi/2] '''
        # Map [-pi/2, pi/2] -> [0, 180]
        mapped_angle = math.degrees(angle_radians) + _90_DEG

        # Linearly interpolate duty cycle over 2 windows for greater accuracy (0-90 and 90-180)
        if mapped_angle <= _90_DEG:
            duty = int(_MIN_DUTY+(mapped_angle/_90_DEG)*(_MID_DUTY-_MIN_DUTY))
        else:
            duty = int(_MID_DUTY+((mapped_angle-_90_DEG)/_90_DEG)*(_MAX_DUTY-_MID_DUTY))

        # Set duty cycle
        self.__pwm_pin.duty_u16(duty)


class servos:
    def __init__(self, rudder_pin:int, sail_pin:int, pwm_freq:int):
        ''' Instantiate servo objects '''
        self.__sail_servo   = servo(sail_pin  , pwm_freq)
        self.__rudder_servo = servo(rudder_pin, pwm_freq)

    def actuate_servos(self, eta_rad:float, phi_rad:float) -> None:
        ''' Actuate both servos given desired angles '''
        self.__sail_servo.set_angle(eta_rad)
        self.__rudder_servo.set_angle(phi_rad)


sail_and_rudder_servos = servos(rudder_pin=_RUDDER_PIN, sail_pin=_SAIL_PIN, pwm_freq=_PWM_FREQ)
