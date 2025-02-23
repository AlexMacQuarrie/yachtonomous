# External
import numpy as np
# Internal
from tools import arr


__SENSOR_READING_TODO__ = 0.1


def _parse_serialized_rssi(serialized_rssi:int) -> int:
    cli_id = serialized_rssi//1000
    rssi   = serialized_rssi %1000  # Assumes positive, if negative, use -1000
    return cli_id, rssi


def _get_updated_rssi_values(num_features:int) -> arr:
    rssi_list = np.zeros(num_features)
    # Keep reading RSSI values until we get a value from each client
    while not np.all(rssi_list != 0):
        serialized_rssi   = __SENSOR_READING_TODO__
        cli_id, rssi      = _parse_serialized_rssi(serialized_rssi)
        rssi_list[cli_id] = rssi

    return rssi_list


def range_sensor(num_features:int) -> arr:
    ''' 
        Exponential range sensor function.
        Gets ranges as RSSI values, which we assume
        are of the form a*e^(b*r) + c, where r is range.
    '''
    return _get_updated_rssi_values(num_features)


def rotation_sensor(x_hat:arr, T:float) -> float:
    ''' IMU angle measurement using integration '''
    return x_hat[2] + (__SENSOR_READING_TODO__)*T


def wind_sensor() -> float:
    ''' Relative wind angle measurement '''
    return __SENSOR_READING_TODO__


def sail_sensor() -> float:
    ''' Sail/mast angle measurement '''
    return __SENSOR_READING_TODO__


def get_measurements(x_hat:arr, num_features:int, T:float) -> arr:
    ''' Get all sensor measurements '''
    z = np.zeros(num_features+3)
    z[0:num_features] = range_sensor(num_features)  # x, y
    z[num_features]   = rotation_sensor(x_hat, T)   # theta
    z[num_features+1] = wind_sensor()               # gamma
    z[num_features+2] = sail_sensor()               # eta
    return z 


def get_distance(x:arr, f_map:arr, i:int) -> float:
    ''' Compute euclidean distance '''
    return np.sqrt((x[0]-f_map[0, i])**2 + (x[1]-f_map[1, i])**2)
