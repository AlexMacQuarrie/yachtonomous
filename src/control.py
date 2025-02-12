# External
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
# Internal
from boat_model import boat
from tools import arr, saturate, kron, matrix_power
from config import settings


def mpc(sailboat:boat, control_config:settings, T:float, x_d:arr, x_hat:arr) -> float:
    ''' Model Predictive Control for feedback control '''
    # Prediction horizon
    p = min(control_config.max_pred_horz, x_d.shape[1])

    # Initialize state error and control effort weight matrices
    Q = kron(np.eye(p, dtype=float), np.diag(control_config.state_weights))
    R = kron(np.eye(p, dtype=float), np.diag(control_config.input_weights))

    # Empty L & M matrices
    n, m    = sailboat.num_states, sailboat.num_inputs
    L       = np.zeros((n*p, n)  , dtype=float)
    M       = np.zeros((n*p, m*p), dtype=float)
    x_d_cat = np.zeros(n*p       , dtype=float)

    # Fill L & M
    for i in range(p):
        # Compute the approximate linearizations
        F = sailboat.F(T, x_d[:, i])
        G = sailboat.G(T)

        # Compute L and M
        L[n*i:n*i+n, 0:n] = matrix_power(F, i+1)
        for j in range(p-i):
            M[n*(p-i)-n:n*(p-i), m*j:m*(j+1)] = np.dot(matrix_power(F, p-i-j-1), G)

        # Concatenated desired state
        x_d_cat[n*i:n*i+n] = x_d[:, i]

    # Compute optimal control inputs
    K = np.dot(np.dot(np.linalg.inv(np.dot(np.dot(M.T, Q), M) + R), M.T), Q) 
    u = np.dot(K, (x_d_cat - np.dot(L, x_hat)))

    # Take first inputs
    u = u[:m]
    for i in range(m):    
        u[i] = saturate(u[i], control_config.input_saturation[i])
    
    return u
