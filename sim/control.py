# External
import numpy as np
# Internal
from boat_model import boat
from tools import arr, wrap_to_pi, saturate
from config import settings


def mpc(sailboat:boat, control_config:settings, T:float, 
        N:int, k:int, x_d:arr, x_hat:arr) -> float:
    ''' Model Predictive Control for feedback control '''
    # Prediction horizon
    p = min(control_config.max_pred_horz, N-k)

    # Initialize state error and control effort weight matrices
    Q = np.kron(np.eye(p), np.diag(control_config.state_weights))
    R = np.kron(np.eye(p), np.diag([control_config.input_weight]))

    # Empty L & M matrices
    n, m = sailboat.num_states, sailboat.num_inputs
    L = np.zeros((n*p, n))
    M = np.zeros((n*p, m*p))
    xi_d = np.zeros(n*p)

    # Fill L & M
    for i in range(p):
        # Compute the approximate linearization
        F = sailboat.F(T, x_d[2, k+i-1], x_d[3, k+i-1])
        G = sailboat.G(T)

        # Compute L and M
        L[n*i:n*i+n, 0:n] = np.linalg.matrix_power(F, i+1)
        for j in range(p-i):
            M[n*(p-i)-n:n*(p-i), m*j:m*(j+1)] = np.linalg.matrix_power(F, p-i-j-1) @ G

        xi_d[n*i:n*i+n] = x_d[:, k+i]

    # Compute control inputs and take first
    K = np.linalg.inv(M.T @ Q @ M + R) @ M.T @ Q 
    u = K @ (xi_d - L @ x_hat[:, k-1])
    return saturate(wrap_to_pi(u[0]), control_config.input_saturation)
