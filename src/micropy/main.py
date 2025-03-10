# External
import time
from machine import Pin
from micropython import const
# Internal
from rpi_comms import udp_socket
from sensor import estimate_initial_gamma, get_measurements, rssi_manager
from servo import sail_and_rudder_servos


# Consts
NUM_BLINK         = const(5)
BLINK_ITERS       = const(2*NUM_BLINK)
BLINK_TIME_S      = const(0.25)
LOG_EN            = const(False)
NUM_WIND_SAMPLES  = const(10)
WIND_AVG_DELAY_US = const(1)


def main() -> None:
    # LED blink to show Pico is connected & restarting
    led = Pin('LED', Pin.OUT)
    for i in range(BLINK_ITERS):
        led.value(i%2)
        time.sleep(BLINK_TIME_S)
    
    # Set up UDP socket
    pico_socket = udp_socket(LOG_EN)

    while True:
        # Receive request
        command, request_json = pico_socket.get_request()
        
        # Send data back or use actuator values      
        if   command == 'SEND_INIT_WIND':
            initial_gamma = estimate_initial_gamma(num_samples = NUM_WIND_SAMPLES, 
                                                   delay_us    = WIND_AVG_DELAY_US)
            pico_socket.send_init_gamma(initial_gamma)
        elif command == 'SEND_DATA':
            pico_socket.send_sensor_readings(get_measurements())
        elif command == 'RECV_DATA':
            sail_and_rudder_servos.actuate_servos(request_json['eta'], request_json['phi'])
        elif command == 'ESP32':
            rssi_manager.update(request_json['number'])
        elif command == 'END_COMMS':
            if LOG_EN:
                print('Ending program')
            break
        else:
            raise Exception(f'Unkown command from client: {command}')

        # Blink to show loop working
        led.toggle()
        
    # Turn LED off at end of program
    led.value(0)
        
        
# main.py runs automatically on startup
main()
