# External
import socket
import json
import numpy as np
# Internal
from tools import arr


class udp_socket:
    def __init__(self, log_en:bool, timeout:float):
        ''' Set up UDP socket and default parms'''
        self.__client_socket  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__server_address = ('192.168.4.1', 12345)  # Pico W AP default IP and port
        self.__buf_size       = 1024
        self.__log_en         = log_en
        self.__client_socket.settimeout(timeout)

    def get_init_wind(self) -> float:
        ''' Send req for initial gamma, and return result from pico '''
        return self.__send_and_recv({'command':'SEND_INIT_WIND'}, 'gamma')

    def get_measurements(self, theta:float, num_features:int, T:float) -> arr:
        ''' Send req for all sensor readings, and return result from pico '''
        z = np.asarray(self.__send_and_recv({'command':'SEND_DATA'}, 'sensor_values'))
        z[num_features] = theta + z[num_features]*T  # integrate imu to get theta
        return z

    def send_actuator_inputs(self, u:arr) -> None:
        ''' Send actuator inputs to pico '''
        if self.__log_en:
            print('Sending actuator inputs')
        self.__send_request({'command':'RECV_DATA', 'eta':u[0], 'phi':u[1]})

    def close(self) -> None:
        if self.__log_en:
            print('Closing socket')
        self.__send_request({'command':'END_COMMS'})

    def __send_and_recv(self, request_data:dict, key:str) -> dict:
        ''' Send request and return results '''
        if self.__log_en:
            print('\nSending request for measurements')

        # Keep trying to send and recv if we timeout
        sent = False
        while not sent:
            try:
                self.__send_request(request_data)
                data = self.__recv_response(key)
                sent = True
            except socket.timeout:
                if self.__log_en:
                    print("Connection timed out. Trying again")
        return data

    def __send_request(self, request_data:dict) -> None:
        ''' Send request to pico '''
        self.__client_socket.sendto(json.dumps(request_data).encode(), self.__server_address)

    def __recv_response(self, key:str):
        ''' Read response from pico '''
        if self.__log_en:
            print('Waiting for response...')
        response, _ = self.__client_socket.recvfrom(self.__buf_size)
        return json.loads(response.decode())[key]
