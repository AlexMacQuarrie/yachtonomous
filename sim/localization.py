# External
import numpy as np
from typing import Tuple
# Internal
from boat_model import boat
from sensor import range_sensor, get_distance
from tools import arr


def ekf(sailboat:boat, exp_parms:list, T:float, x_hat:arr, P:arr, 
        u:float, z:arr, Q:arr, R:arr, f_map:arr) -> Tuple[arr, arr]:
    ''' Extended Kalman Filter for localization '''
    # Compute the Jacobian matrices (linearize about current estimate)
    num_states = len(x_hat)
    F = sailboat.F(T, x_hat)
    G = sailboat.G(T)

    # Compute the a priori estimate
    P_new = F @ P @ F.T + G @ Q @ G.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help the covariance matrix stay symmetric
    x_new = x_hat + T*sailboat.f(x_hat, u)

    # Linearize measurement model
    # Compute the Jacobian matrices (linearize about current estimate)
    num_features = f_map.shape[1]
    num_states   = len(x_hat)
    exp_measures = range_sensor(x_hat, exp_parms, 0, f_map)
    H = np.zeros((num_features+2, num_states))
    for j in range(0, num_features):
        distance    = get_distance(x_hat, f_map, j)
        H[j, :] = np.array(
            [
                -exp_parms[1]*exp_measures[j]*(x_hat[0] - f_map[0, j])/distance,
                -exp_parms[1]*exp_measures[j]*(x_hat[1] - f_map[1, j])/distance,
                0,
                0,
                0,
            ]
        )
    # Add a measurement for theta and gamma (IMU & wind sensor)
    H[num_features  , :] = np.array([0, 0, 1, 0, 0])
    H[num_features+1, :] = np.array([0, 0, 0, 1, 0])

    # Check the observability of this system
    observability_matrix = H
    for j in range(1, num_states):
        observability_matrix = np.concatenate((observability_matrix, 
                                               H @ np.linalg.matrix_power(F, j)), 
                                               axis=0)
    if np.linalg.matrix_rank(observability_matrix) < num_states:
        raise ValueError('System is not observable!')

    # Compute the Kalman gain
    K = P_new @ H.T @ np.linalg.inv(H @ P_new @ H.T + R)

    # Compute a posteriori state estimate
    z_hat = np.zeros(num_features+2)
    z_hat[0:num_features] = range_sensor(x_new, exp_parms, 0, f_map)
    z_hat[num_features]   = x_new[2]
    z_hat[num_features+1] = x_new[3]
    x_new = x_new + K @ (z - z_hat)

    # Compute a posteriori covariance
    P_new = (np.eye(num_states) - K @ H) @ P_new @ (np.eye(num_states) - K @ H).T + K @ R @ K.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help the covariance matrix stay symmetric

    # Return the estimated state
    return x_new, P_new
