# External
import numpy as np
# Internal
from tools import arr


class WindSensor:
    def __init__(self, N:int, use_avg:bool=True):
        self.use_avg = use_avg
        if self.use_avg:
            self.abs_wind_est = np.zeros(N)

    def read(self, k:int, rel_wind_angle, x_hat:arr, sigma_wind:float, num_samples:int=5):
        rel_wind_est = np.mean([rel_wind_angle + sigma_wind*np.random.randn() for _ in range(num_samples)])

        # Use moving average of most recent 5 estimates to reduce noise
        # Only use this if wind direction is constant
        if self.use_avg:
            abs_wind_est = rel_wind_est + x_hat[2]
            self.abs_wind_est[k] = abs_wind_est
            left_idx = max(0, k-4)
            self.abs_wind_est[k] = np.mean(self.abs_wind_est[left_idx:k+1])
            rel_wind_est = self.abs_wind_est[k] - x_hat[2]

        return rel_wind_est
    

def estimate_initial_wind(rel_wind_angle, sigma_wind:float, num_samples:int=5) -> float:
    return np.mean([rel_wind_angle + sigma_wind*np.random.randn() for _ in range(num_samples)])
    

def range_sensor(x:arr, exp_parms:list, sigma_w:float, f_map:arr) -> arr:
    # Compute the measured range to each feature from the current robot position
    num_features = f_map.shape[1]
    z = np.zeros(num_features)
    for j in range(num_features):
        r     = get_distance(x, f_map, j)
        z[j]  = exp_parms[0]*np.exp(-exp_parms[1]*r)
        z[j] += sigma_w*np.random.randn()
    # Return the array of noisy measurements
    return z


def rotation_sensor(x:arr, sigma_w:float) -> float:
    # NOTE: Actual IMU probably needs to integrate?
    # Compute vehicle angle with noise
    return x[2] + sigma_w*np.random.randn()


def get_measurements(x:arr, exp_parms:list, sigma_w:float, f_map:arr) -> arr:
    num_features = f_map.shape[1]
    z = np.zeros(num_features+1)
    z[0:num_features] = range_sensor(x, exp_parms, sigma_w[0], f_map)
    z[num_features]   = rotation_sensor(x, sigma_w[1])
    return z 


def get_distance(x:arr, f_map:arr, i:int) -> float:
    # Compute euclidean distance
    return np.sqrt((x[0] - f_map[0, i])**2 + (x[1] - f_map[1, i])**2)
