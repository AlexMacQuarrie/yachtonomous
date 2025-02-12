# External
import numpy as np
# Internal
from boat_model import boat
from tools import arr, saturate
from config import settings


def mpc(sailboat:boat, control_config:settings, T:float, x_d:arr, x_hat:arr) -> float:
    ''' Model Predictive Control for feedback control '''
    # Prediction horizon
    p = min(control_config.max_pred_horz, x_d.shape[1])

    # Initialize state error and control effort weight matrices
    Q = np.kron(np.eye(p), np.diag(control_config.state_weights))
    R = np.kron(np.eye(p), np.diag(control_config.input_weights))

    # Empty L & M matrices
    n, m = sailboat.num_states, sailboat.num_inputs
    L = np.zeros((n*p, n))
    M = np.zeros((n*p, m*p))
    x_d_cat = np.zeros(n*p)

    # Fill L & M
    for i in range(p):
        # Compute the approximate linearizations
        F = sailboat.F(T, x_d[:, i])
        G = sailboat.G(T)

        # Compute L and M
        L[n*i:n*i+n, 0:n] = np.linalg.matrix_power(F, i+1)
        for j in range(p-i):
            M[n*(p-i)-n:n*(p-i), m*j:m*(j+1)] = np.linalg.matrix_power(F, p-i-j-1) @ G

        # Concatenated desired state
        x_d_cat[n*i:n*i+n] = x_d[:, i]

    # Compute optimal control inputs
    K = np.linalg.inv(M.T @ Q @ M + R) @ M.T @ Q 
    u = K @ (x_d_cat - L @ x_hat)

    # Take first inputs
    u = u[:m]
    for i in range(m):    
        u[i] = saturate(u[i], control_config.input_saturation[i])
    
    return u
