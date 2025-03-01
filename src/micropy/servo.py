from time import sleep
from machine import Pin, PWM

pwm = PWM(Pin(11))
pwm.freq(50)

while True:
    for pos in range(1000,9000,100):
        pwm.duty_u16(pos)
        sleep(0.05)
    for pos in range(9000,1000,-200):
        pwm.duty_u16(pos)
        sleep(0.01)

