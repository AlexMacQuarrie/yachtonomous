# External
import numpy as np
from typing import Callable
# Internal
from tools import arr
    

def range_sensor(x:arr, exp_parms:list, sigma_w:float, f_map:arr) -> arr:
    ''' Exponential range sensor function '''
    # Compute the measured range to each feature from the current robot position
    num_features = f_map.shape[1]
    z = np.zeros(num_features)
    for j in range(num_features):
        r     = get_distance(x, f_map, j)
        z[j]  = exp_parms[0]*np.exp(-exp_parms[1]*r)
        z[j] += sigma_w*np.random.randn()
    # Return the array of noisy measurements
    return z


def rotation_sensor(x_hat:arr, sigma_w:float, f:arr, T:float) -> float:
    ''' Simulate IMU angle measurement using integration '''
    return x_hat[2] + (f[2]+sigma_w*np.random.randn())*T


def wind_sensor(x:arr, sigma_w:float):
    ''' 
        Simulate relative wind angle measurement.
        Technically this is done using 2 rotation sensors,
        so noise is doubled
    '''
    return x[3] + sigma_w*np.random.randn()


def sail_sensor(x:arr, sigma_w:float) -> float:
    ''' Simulate sail/mast angle measurement '''
    return x[5] + sigma_w*np.random.randn()


def get_measurements(x:arr, x_hat:arr, u:arr, f:Callable, exp_parms:list, 
                     sigma_w:float, f_map:arr, T:float) -> arr:
    ''' Get all sensor measurements '''
    num_features = f_map.shape[1]
    z = np.zeros(num_features+3)
    z[0:num_features] = range_sensor(x, exp_parms, sigma_w[0], f_map)   # x, y
    z[num_features]   = rotation_sensor(x_hat, sigma_w[1], f(x, u), T)  # theta
    z[num_features+1] = wind_sensor(x, sigma_w[2])                      # gamma
    z[num_features+2] = sail_sensor(x, sigma_w[3])                      # eta
    return z 


def get_distance(x:arr, f_map:arr, i:int) -> float:
    ''' Compute euclidean distance '''
    return np.sqrt((x[0]-f_map[0, i])**2 + (x[1]-f_map[1, i])**2)
