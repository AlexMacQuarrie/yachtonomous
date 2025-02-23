# External
import numpy as np
import scipy
from functools import reduce
# Internal
from boat_model import boat
from tools import arr
from config import settings


def mpc(sailboat:boat, control_config:settings, T:float, x_d:arr, x_hat:arr) -> arr:
    ''' Model Predictive Control for feedback control '''
    # Prediction horizon
    p    = min(control_config.max_pred_horz, x_d.shape[1])
    n, m = sailboat.num_states, sailboat.num_inputs

    # Initialize state error and control effort weight matrices
    Q = scipy.linalg.block_diag(*[np.diag(control_config.state_weights)]*p)
    R = scipy.linalg.block_diag(*[np.diag(control_config.input_weights)]*p)

    # Compute all discrete time approximate linearizations
    F_stack = [sailboat.F(T, x_d[:, i]) for i in range(p)]
    G       =  sailboat.G(T)

    # Compute L
    L = np.vstack([np.linalg.matrix_power(F_stack[i], i+1) for i in range(p)])

    # Compute powers of F for M sequentially
    F_powers = np.array([reduce(np.matmul, F_stack[:i], np.eye(n)) for i in range(p)])    

    # Compute M
    M = np.zeros((n*p, m*p))
    for i, j in zip(*np.tril_indices(p)):
        M[n*i:n*(i+1), m*j:m*(j+1)] = F_powers[p-i-1-j] @ G

    # Compute optimal control inputs
    K = scipy.linalg.solve(M.T @ Q @ M + R, M.T @ Q, assume_a='pos')
    u = K @ (x_d[:, :p].ravel(order='F') - L @ x_hat)

    # Take first inputs & clip
    return np.clip(u[:m], -control_config.input_saturation, control_config.input_saturation)
