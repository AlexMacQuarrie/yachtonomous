import numpy as np
from typing import Tuple
from boat import boat
from sensor import range_sensor, get_distance
from tools import arr


def ekf(vehicle:boat, exp_parms:list, T:float, q:arr, P:arr, 
        u:arr, z:arr, Q:arr, R:arr, f_map:arr) -> Tuple[arr, arr]:
    # Compute the Jacobian matrices (linearize about current estimate)
    num_states = len(q)
    F = vehicle.F(T, u[0], q[2], q[3])
    G = vehicle.G(T, q[2], q[3])

    # Compute the a priori estimate
    P_new = F @ P @ F.T + G @ Q @ G.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help the covariance matrix stay symmetric
    q_new = q + T*vehicle.f(q, u)

    # Linearize measurement model
    # Compute the Jacobian matrices (linearize about current estimate)
    num_features = f_map.shape[1]
    num_states = len(q)
    exp_measures = range_sensor(q, exp_parms, 0, f_map)
    H = np.zeros((num_features+1, num_states))
    for j in range(0, num_features):
        exp_measure = exp_measures[j]
        distance    = get_distance(q, f_map, j)
        H[j, :] = np.array(
            [
                -exp_parms[1]*exp_measure*(q[0] - f_map[0, j])/distance,
                -exp_parms[1]*exp_measure*(q[1] - f_map[1, j])/distance,
                0,
                0,
            ]
        )
    # Add a measurement for theta (Like from IMU, after integration)
    H[num_features, :] = np.array([0, 0, 1, 0])

    # Check the observability of this system
    observability_matrix = H
    for j in range(1, num_states):
        observability_matrix = np.concatenate((observability_matrix, 
                                               H @ np.linalg.matrix_power(F, j)), 
                                               axis=0)
    if np.linalg.matrix_rank(observability_matrix) < num_states:
        raise ValueError("System is not observable!")

    # Compute the Kalman gain
    K = P_new @ H.T @ np.linalg.inv(H @ P_new @ H.T + R)

    # Compute a posteriori state estimate
    z_hat = np.zeros(num_features+1)
    z_hat[0:num_features] = range_sensor(q_new, exp_parms, 0, f_map)
    z_hat[num_features] = q_new[2]
    q_new = q_new + K @ (z - z_hat)

    # Compute a posteriori covariance
    P_new = (np.eye(num_states) - K @ H) @ P_new @ (np.eye(num_states) - K @ H).T + K @ R @ K.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help the covariance matrix stay symmetric

    # Return the estimated state
    return q_new, P_new
