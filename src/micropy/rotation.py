from machine import Pin, time_pulse_us
import time

PWM_PIN = 11  # Change this to the actual pin you are using
pwm_pin = Pin(PWM_PIN, Pin.IN)

NUM_SAMPLES = 10  # Number of samples to average

while True:
    total_high = 0
    total_pwm = 0

    for _ in range(NUM_SAMPLES):
        # Measure HIGH time in microseconds
        T_HIGH = time_pulse_us(pwm_pin, 1)

        # Measure LOW time in microseconds
        T_LOW = time_pulse_us(pwm_pin, 0)

        # Total PWM period
        T_PWM = T_HIGH + T_LOW

        # Only add valid readings
        if T_PWM > 0:
            total_high += T_HIGH
            total_pwm += T_PWM

        time.sleep_ms(10)  # Short delay between samples to avoid flooding

    # Calculate the average angle
    if total_pwm > 0:
        avg_high = total_high / NUM_SAMPLES
        avg_pwm = total_pwm / NUM_SAMPLES
        angle = (avg_high / avg_pwm) * 360
    else:
        angle = 0

    print("Angle:", angle)

    time.sleep_ms(10)  # Delay between batches of samples

