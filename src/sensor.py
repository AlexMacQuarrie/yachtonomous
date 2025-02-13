# External
import numpy as np
# Internal
from tools import arr


__SENSOR_READING_TODO__ = 0.1


def range_sensor(num_features:int) -> arr:
    ''' Exponential range sensor function '''
    z = np.zeros(num_features)
    for j in range(num_features):
        z[j] = __SENSOR_READING_TODO__
    return z


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
