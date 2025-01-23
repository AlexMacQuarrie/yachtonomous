import numpy as np
from scipy import signal
from boat import boat
from tools import arr
from config import settings


def _saturate(u:arr, sat_limits:list) -> arr:
    return np.asarray([max(inp, -lim) if inp < 0 else min(inp, lim) for inp, lim in zip(u, sat_limits)])


def mpc(vehicle:boat, control_config:settings, T:float, 
        N:int, k:int, x_d:arr, u_d:arr, x_hat:arr) -> arr:
    # Prediction horizon
    p = min(control_config.max_pred_horz, N-k)

    # Initialize state error and control effort weight matrices
    Q = np.kron(np.eye(p), np.diag(control_config.state_weights))
    R = np.kron(np.eye(p), np.diag(control_config.input_wieghts))

    # Empty L & M matrices
    n, m = 4, 2
    L = np.zeros((n*p, n))
    M = np.zeros((n*p, m*p))
    xi_d = np.zeros(n*p)

    # Fill L & M
    for i in range(p):
        # Compute the approximate linearization
        F = vehicle.F(T, u_d[0, k+i-1], x_d[2, k+i-1], x_d[3, k+i-1])
        G = vehicle.G(T, x_d[2, k+i-1], x_d[3, k+i-1])

        # Compute L and M
        L[n*i:n*i+n, 0:n] = np.linalg.matrix_power(F, i+1)
        for j in range(p-i):
            M[n*(p-i)-n:n*(p-i), m*j:m*(j+1)] = np.linalg.matrix_power(F, p-i-j-1) @ G

        xi_d[n*i:n*i+n] = x_d[:, k+i]

    # Compute control inputs and take first
    K = np.linalg.inv(M.T @ Q @ M + R) @ M.T @ Q 
    u = K @ (xi_d - L @ x_hat[:, k-1])
    return _saturate(u[:2], control_config.input_saturation)


def fsf(vehicle:boat, control_config:settings, 
        x_d:arr, u_d:arr, x_hat:arr) -> arr:
    # Compute the gain matrix to place poles of (A-BK) at p
    A = vehicle.A(u_d[0], x_d[2], x_d[3])
    B = vehicle.B(x_d[2], x_d[3])
    p = np.array(control_config.poles)
    K = signal.place_poles(A, B, p)

    # Compute the controls (v, omega)
    return _saturate(K.gain_matrix @ (x_d - x_hat) + u_d, control_config.input_saturation)
