# External
import numpy as np
import scipy
from typing import Tuple
# Internal
from boat_model import boat
from sensor import range_sensor, get_distance
from tools import arr


def ekf(sailboat:boat, log_parms:list, T:float, x_hat:arr, P:arr, 
        u:arr, z:arr, Q:arr, R:arr, f_map:arr) -> Tuple[arr, arr]:
    ''' Extended Kalman Filter for localization '''
    # Compute the Jacobian matrices
    F = sailboat.F(T, x_hat)
    G = sailboat.G(T)

    # Compute the a priori estimate (dead reckoning)
    P_new = F @ P @ F.T + G @ Q @ G.T
    P_new = np.tril(P_new) + np.triu(P_new.T, 1)  # Symmetry
    x_new = x_hat + T*sailboat.f(x_hat, u)

    # Linearize measurement model, compute the Jacobian
    num_features = f_map.shape[1]
    log_measures = range_sensor(x_hat, log_parms, 0, f_map)
    H = np.empty((num_features+3, sailboat.num_states))
    for j in range(0, num_features):
        sq_distance = get_distance(x_hat, f_map, j)**2
        H[j, :] = np.array(
            [
                log_parms[1]*(x_hat[0] - f_map[0, j])/sq_distance,
                log_parms[1]*(x_hat[1] - f_map[1, j])/sq_distance,
                0,
                0,
                0,
                0,
            ]
        )
    # Add a measurement for theta, gamma, eta (IMU, wind sensor, rotation sensor)
    H[num_features  , :] = np.array([0, 0, 1, 0, 0, 0])
    H[num_features+1, :] = np.array([0, 0, 0, 1, 0, 0])
    H[num_features+2, :] = np.array([0, 0, 0, 0, 0, 1])

    # Check the observability of this system (We don't do this on the boat to save time)
    observability_matrix = H
    for j in range(1, sailboat.num_states):
        observability_matrix = np.concatenate((observability_matrix, 
                                               H @ np.linalg.matrix_power(F, j)), 
                                               axis=0)
    if np.linalg.matrix_rank(observability_matrix) < sailboat.num_states:
        raise ValueError('System is not observable!')

    # Compute the Kalman gain
    K = scipy.linalg.solve(H @ P_new @ H.T + R, H @ P_new).T

    # Compute a posteriori state estimate
    z_hat = np.zeros(num_features+3)
    z_hat[0:num_features] = log_measures  # x, y
    z_hat[num_features]   = x_new[2]      # theta
    z_hat[num_features+1] = x_new[3]      # gamma
    z_hat[num_features+2] = x_new[5]      # eta
    x_new += K @ (z - z_hat)

    # Compute a posteriori covariance
    I_KH  = np.eye(sailboat.num_states) - K @ H
    P_new = I_KH @ P_new @ I_KH.T + K @ R @ K.T
    P_new = np.tril(P_new) + np.triu(P_new.T, 1)  # Symmetry

    # Return the estimated state and covariance
    return x_new, P_new
