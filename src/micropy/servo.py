# External
from machine import Pin, PWM
from micropython import const
import time
import math


# Consts
_RUDDER_PIN = const(10)
_SAIL_PIN   = const(11)
_PWM_FREQ   = const(50)
_MIN_DUTY   = const(1000)  #   0 deg
_MID_DUTY   = const(4550)  #  90 deg
_MAX_DUTY   = const(7500)  # 180 deg
_90_DEG     = const(90)


# Set up rudder and sail servo PWM pins
_rudder_pwm = PWM(Pin(_RUDDER_PIN), freq=_PWM_FREQ)
_sail_pwm   = PWM(Pin(_SAIL_PIN)  , freq=_PWM_FREQ)


def actuate_servos(eta:float, phi:float) -> None:
    ''' Actuate both servos given desired angles '''
    _set_servo_angle(_sail_pwm  , eta)
    _set_servo_angle(_rudder_pwm, phi)


def _set_servo_angle(pwm_servo:PWM, angle_radians:float) -> None:
    ''' Set the angle for a servo, input should be [-pi/2, pi/2] '''
    # Map [-pi/2, pi/2] -> [0, 180]
    mapped_angle = math.degrees(angle_radians) + _90_DEG

    # Linearly interpolate duty cycle over 2 windows for greater accuracy (0-90 and 90-180)
    if mapped_angle <= _90_DEG:
        duty = int(_MIN_DUTY+(mapped_angle/_90_DEG)*(_MID_DUTY-_MIN_DUTY))
    else:
        duty = int(_MID_DUTY+((mapped_angle-_90_DEG)/_90_DEG)*(_MAX_DUTY-_MID_DUTY))

    # Set duty cycle
    pwm_servo.duty_u16(duty)
