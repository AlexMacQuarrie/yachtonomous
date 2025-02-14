# External
import numpy as np
# Internal
from boat_model import boat
from sensor import range_sensor, get_distance
from tools import arr


def ekf(sailboat:boat, exp_parms:list, T:float, x_hat:arr, P:arr, 
        u:arr, z:arr, Q:arr, R:arr, f_map:arr) -> arr:
    ''' Extended Kalman Filter for localization '''
    # Compute the Jacobian matrices
    F = sailboat.F(T, x_hat)
    G = sailboat.G(T)

    # Compute the a priori estimate (dead reckoning)
    P_new = F @ P @ F.T + G @ Q @ G.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help symmetry
    x_new = x_hat + T*sailboat.f(x_hat, u)

    # Linearize measurement model - Compute the Jacobian matrices
    num_features = f_map.shape[1]
    exp_measures = range_sensor(num_features)
    H = np.zeros((num_features+3, sailboat.num_states))
    for j in range(num_features):
        distance = get_distance(x_hat, f_map, j)
        H[j, :] = np.array(
            [
                -exp_parms[1]*exp_measures[j]*(x_hat[0] - f_map[0, j])/distance,
                -exp_parms[1]*exp_measures[j]*(x_hat[1] - f_map[1, j])/distance,
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

    # Compute the Kalman gain
    K = P_new @ H.T @ np.linalg.inv(H @ P_new @ H.T + R)

    # Compute a posteriori state estimate
    z_hat = np.zeros(num_features+3)
    z_hat[0:num_features] = exp_measures  # x, y
    z_hat[num_features]   = x_new[2]      # theta
    z_hat[num_features+1] = x_new[3]      # gamma
    z_hat[num_features+2] = x_new[5]      # eta
    x_new += K @ (z - z_hat)

    # Compute a posteriori covariance
    I_KH  = np.eye(sailboat.num_states) - K @ H
    P_new = I_KH @ P_new @ I_KH.T + K @ R @ K.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help symmetry

    # Return the estimated state and covariance
    return x_new, P_new
