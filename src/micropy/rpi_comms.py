# External
from micropython import const
import socket
import network
import time
import json


# Consts
_SSID        = const('PicoW_Network')
_PASSWORD    = const('micropython123')
_UPD_PORT    = const(12345)
_BUF_SIZE    = const(1024)
_ACT_DELAY_S = const(0.1)


class udp_socket:
    def __init__(self, log_en:bool):
        ''' Set up access point and UDP server '''
        # Set up Pico W as access point
        ap = network.WLAN(network.AP_IF)
        ap.config(essid=_SSID, password=_PASSWORD)
        ap.active(True)
        while not ap.active():
            time.sleep(_ACT_DELAY_S)

        # Set up UDP server
        self.log_en         = log_en
        self.buf_size       = _BUF_SIZE
        self.client_address = None   
        self.server_socket  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((ap.ifconfig()[0], _UPD_PORT))
        if self.log_en:
            print('Server setup complete')

    def get_request(self) -> dict:
        ''' Receive request from client (Hangs until we get something) '''
        if self.log_en:
            print('\nWaiting for a request from the client...')
        request, self.client_address = self.server_socket.recvfrom(self.buf_size)
        req_json = json.loads(request.decode())
        return req_json['command'], req_json
    
    def send_init_gamma(self, gamma:float) -> None:
        ''' Send initial relative wind angle to client '''
        if self.log_en:
            print('sending gamma')
        self.__send_response({'gamma': gamma})
    
    def send_sensor_readings(self, readings:list) -> None:
        ''' Send all sensor readings to client '''
        if self.log_en:
            print('sending sensor readings')
        self.__send_response({'sensor_values': readings})

    def __send_response(self, response_data:dict) -> None:
        ''' Send response to client '''
        if self.client_address is None:
            raise Exception('Cannot send response before receving a request, aborting')
        self.server_socket.sendto(json.dumps(response_data).encode(), self.client_address)
