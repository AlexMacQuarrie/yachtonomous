# External
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from scipy.stats import chi2
from scipy import signal
from typing import Tuple
# Internal
from robot_tools import rk_four, trim_sail
from robot_tools import Tricycle
from tools import Angle


class arr(np.ndarray):
    # Alias for more concise type hinting
    pass


def wrap_to_pi(angle:float):
    """Wrap angles to the range [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def sec2(x:float) -> float:
    # Compute sec^2(x) -> sec^2(x) = tan^2(x) + 1
    return np.tan(x)**2 + 1


def minmax_norm(x:list) -> arr:
    x = np.asarray(x)
    return (x-np.min(x))/(np.max(x)-np.min(x))


class WindSensor:
    def __init__(self, N:int, use_avg:bool=True):
        self.use_avg = use_avg
        if self.use_avg:
            self.abs_wind_est = np.zeros(N)

    def wind_sensor(self, k:int, rel_wind_angle, x_hat:arr, sigma_wind:float, num_samples:int=5):
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
    

def get_distance(x:arr, f_map:arr, i:int) -> float:
    # Compute euclidean distance
    return np.sqrt((x[0] - f_map[0, i])**2 + (x[1] - f_map[1, i])**2)


def range_sensor(x:arr, sigma_w:float, f_map:arr) -> arr:
    # TODO: Made these up!
    a = 5.0
    b = 2.0
    # Compute the measured range to each feature from the current robot position
    num_features = f_map.shape[1]
    z = np.zeros(num_features)
    for j in range(num_features):
        r     = get_distance(x, f_map, j)
        z[j]  = a*np.exp(-b*r)
        z[j] += sigma_w*np.random.randn()
    # Return the array of noisy measurements
    return z


def rotation_sensor(x:arr, sigma_w:float) -> float:
    # NOTE: Actual IMU probably needs to integrate
    # Compute vehicle angle with noise
    return x[2] + sigma_w*np.random.randn()


def get_measurements(x:arr, sigma_w:float, f_map:arr) -> arr:
    num_features = f_map.shape[1]
    z = np.zeros(num_features+1)
    z[0:num_features] = range_sensor(x, sigma_w[0], f_map)
    z[num_features]   = rotation_sensor(x, sigma_w[1])
    return z 


def ekf(vehicle:Tricycle, T:float, q:arr, P:arr, u:arr, z:arr, Q:arr, R:arr, f_map:arr) -> Tuple[arr, arr]:
    # Compute the Jacobian matrices (linearize about current estimate)
    num_states = len(q)
    F = np.eye(num_states) + u[0]*T*np.array(
        [
            [0, 0, -np.sin(q[2]),  0                       ],
            [0, 0,  np.cos(q[2]),  0                       ],
            [0, 0,  0           , -sec2(q[3])/vehicle.ell_W],
            [0, 0,  0           ,  0                       ],
        ]
    )
    G = T*np.array(
        [
            [ np.cos(q[2])              , 0],
            [ np.sin(q[2])              , 0],
            [-np.tan(q[3])/vehicle.ell_W, 0],
            [ 0                         , 1],
        ]
    )

    # Compute the a priori estimate
    P_new = F @ P @ F.T + G @ Q @ G.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help the covariance matrix stay symmetric
    q_new = q + T*vehicle.f(q, u)

    # Linearize measurement model
    # Compute the Jacobian matrices (linearize about current estimate)
    num_features = f_map.shape[1]
    num_states = len(q)
    exp_measures = range_sensor(q, 0, f_map)
    H = np.zeros((num_features+1, num_states))
    for j in range(0, num_features):
        exp_measure = exp_measures[j]
        distance    = get_distance(q, f_map, j)
        H[j, :] = np.array(
            [
                # TODO: Need to grab b (2.0) from somewhere
                -2.0*exp_measure*(q[0] - f_map[0, j])/distance,
                -2.0*exp_measure*(q[1] - f_map[1, j])/distance,
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
    z_hat[0:num_features] = range_sensor(q_new, 0, f_map)
    z_hat[num_features] = q_new[2]
    q_new = q_new + K @ (z - z_hat)

    # Compute a posteriori covariance
    P_new = (np.eye(num_states) - K @ H) @ P_new @ (np.eye(num_states) - K @ H).T + K @ R @ K.T
    P_new = 0.5*(P_new + P_new.T)  # Numerically help the covariance matrix stay symmetric

    # Return the estimated state
    return q_new, P_new


def mpc(vehicle:Tricycle, T:float, N:int, k:int, x_d:arr, u_d:arr, x_hat:arr) -> arr:
    # Prediction horizon
    p = min(50, N-k)

    # Initialize state error and control effort weight matrices
    # TODO: I made these up might be kinda shit
    w = [10.0, 10.0, 5.0, 1.0, 5.0, 1.0]
    # w = minmax_norm(w)
    small_Q = np.diag(w[:4])
    small_R = np.diag(w[4:])
    Q = np.kron(np.eye(p), small_Q)
    R = np.kron(np.eye(p), small_R)

    # Empty L & M matrices
    n = 4
    m = 2
    L = np.zeros((n*p, n))
    M = np.zeros((n*p, m*p))
    xi_d = np.zeros(n*p)

    # Fill L & M
    for i in range(p):
        # Compute the approximate linearization
        F = np.eye(4) + u_d[0, k+i-1]*T*np.array(
            [
                [0, 0, -np.sin(x_d[2, k+i-1]), 0                                ],
                [0, 0,  np.cos(x_d[2, k+i-1]), 0                                ],
                [0, 0,  0,                    -sec2(x_d[3, k+i-1])/vehicle.ell_W],
                [0, 0,  0,                     0                                ],
            ]
        )
        G = T*np.array(
            [
                [ np.cos(x_d[2, k+i-1]),               0],
                [ np.sin(x_d[2, k+i-1]),               0],
                [-np.tan(x_d[3, k+i-1])/vehicle.ell_W, 0],
                [ 0,                                   1],
            ]
        )

        L[n*i:n*i+n, 0:n] = np.linalg.matrix_power(F, i+1)
        for j in range(p-i):
            M[n*(p-i)-n:n*(p-i), m*j:m*(j+1)] = np.linalg.matrix_power(F, p-i-j-1) @ G

        xi_d[n*i:n*i+n] = x_d[:, k+i]

    # Compute control inputs and take first
    K = np.linalg.inv(M.T @ Q @ M + R) @ M.T @ Q 
    u = K @ (xi_d - L @ x_hat[:, k-1])
    # I have no idea if + u_d[:, k-1] should be here really hard to tell
    return u[:2]


def fsf(vehicle:Tricycle, x_d:arr, u_d:arr, x_hat:arr) -> arr:
    # Compute the approximate linearization
    A = u_d[0]*np.array(
        [
            [0, 0, -np.sin(x_d[2]), 0                         ],
            [0, 0,  np.cos(x_d[2]), 0                         ],
            [0, 0,  0,             -sec2(x_d[3])/vehicle.ell_W],
            [0, 0,  0,              0                         ],
        ]
    )
    B = np.array(
        [
            [ np.cos(x_d[2]),               0],
            [ np.sin(x_d[2]),               0],
            [-np.tan(x_d[3])/vehicle.ell_W, 0],
            [ 0,                            1],
        ]
    )

    # Compute the gain matrix to place poles of (A-BK) at p
    p = np.array([-0.5, -1.0, -1.5, -2.0])
    K = signal.place_poles(A, B, p)

    # Compute the controls (v, omega)
    return K.gain_matrix @ (x_d - x_hat) + u_d 


def plot_results(vehicle:Tricycle, t:arr, N:int, T:float, f_map:arr, 
                 x:arr, x_hat:arr, x_d:arr, u:arr, u_d:arr, 
                 w:arr, w_hat:arr, s_d:arr, s:arr, sigma_wind:arr, 
                 P_hat:arr, map_size:arr, dest_pos:arr) -> None:
    # Change some plot settings (optional)
    plt.rc("savefig", format="pdf")
    plt.rc("savefig", bbox="tight")

    # Find the scaling factor for plotting covariance bounds
    # 0.01 -> 99% Confidence
    ALPHA = 0.01
    s1 = chi2.isf(ALPHA, 1)

    # Wind arrow plotting stuff
    radius         = np.sqrt(2.0)*map_size/2.0
    arrow_len      = map_size*0.1
    abs_wind_angle = w[0] + x[2, 0]
    wind_arrow     = arrow_len*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle)))
    wa_base        = radius*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle))) + (map_size/2.0,map_size/2.0)
    min_wind_angle = np.min(w_hat + x_hat[2, :])
    min_wind_arrow = arrow_len*np.array((np.cos(min_wind_angle), np.sin(min_wind_angle)))
    min_wa_base    = radius*np.array((np.cos(min_wind_angle), np.sin(min_wind_angle))) + (map_size/2.0,map_size/2.0)
    max_wind_angle = np.max(w_hat + x_hat[2, :])
    max_wind_arrow = arrow_len*np.array((np.cos(max_wind_angle), np.sin(max_wind_angle)))
    max_wa_base    = radius*np.array((np.cos(max_wind_angle), np.sin(max_wind_angle))) + (map_size/2.0,map_size/2.0)    

    # Sail Plotting
    abs_sail_angle = s[0] + x[2, 0]
    sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
    plt.arrow(x[0, 0], x[1, 0], sail_arrow[0], sail_arrow[1], head_width=0, color='k')
    abs_sail_angle = s[-1] + x[2, -1]
    sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
    plt.arrow(x[0, -1], x[1, -1], sail_arrow[0], sail_arrow[1], head_width=0, color='k')

    # Plot the position of the vehicle in the plane
    fig1 = plt.figure(1)
    plt.arrow(min_wa_base[0]+min_wind_arrow[0], min_wa_base[1]+min_wind_arrow[1], -min_wind_arrow[0], -min_wind_arrow[1], head_width=0.05, color='C1', label='Min Wind Est.')
    plt.arrow(max_wa_base[0]+max_wind_arrow[0], max_wa_base[1]+max_wind_arrow[1], -max_wind_arrow[0], -max_wind_arrow[1], head_width=0.05, color='C1', label='Max Wind Est.')
    plt.arrow(wa_base[0]+wind_arrow[0], wa_base[1]+wind_arrow[1], -wind_arrow[0], -wind_arrow[1], head_width=0.05, color='b', label='Wind')
    plt.plot(f_map[0, :], f_map[1, :], "C4*", label="Feature")
    plt.plot(dest_pos[0], dest_pos[1], "C3D", label="Destination")
    plt.plot(x_hat[0, 0], x_hat[1, 0], "C5D", label="Est. Start")
    plt.gca().add_patch(patches.Rectangle((0, 0), map_size, map_size, edgecolor='k', fill=False))
    plt.plot(x[0, :], x[1, :], "C0", label="Actual")
    plt.plot(x_hat[0, :], x_hat[1, :], "C1--", label="Estimated")
    plt.plot(x_d[0, :], x_d[1, :], "C2--", label="Desired")
    plt.axis("equal")
    X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B = vehicle.draw(*x[:, 0])
    plt.fill(X_L, Y_L, "k")
    plt.fill(X_R, Y_R, "k")
    plt.fill(X_F, Y_F, "k")
    plt.fill(X_B, Y_B, "C2", alpha=0.5, label="Start")
    X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B = vehicle.draw(*x[:, N-1])
    plt.fill(X_L, Y_L, "k")
    plt.fill(X_R, Y_R, "k")
    plt.fill(X_F, Y_F, "k")
    plt.fill(X_B, Y_B, "C3", alpha=0.5, label="End")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.savefig("src_outputs/boat_drawing.pdf")

    # Plot the states and inputs as a function of time
    fig2 = plt.figure(2)
    fig2.set_figheight(10.0)
    ax2a = plt.subplot(811)
    plt.plot(t, x[0, :], "C0", label="Actual")
    plt.plot(t, x_hat[0, :], "C1--", label="Estimated")
    plt.plot(t, x_d[0, :], "C2--", label="Desired")
    plt.grid(color="0.95")
    plt.ylabel("x [m]")
    plt.setp(ax2a, xticklabels=[])
    plt.legend()
    ax2b = plt.subplot(812)
    plt.plot(t, x[1, :], "C0", label="Actual")
    plt.plot(t, x_hat[1, :], "C1--", label="Estimated")
    plt.plot(t, x_d[1, :], "C2--", label="Desired")
    plt.grid(color="0.95")
    plt.ylabel("y [m]")
    plt.setp(ax2b, xticklabels=[])
    ax2c = plt.subplot(813)
    plt.plot(t, wrap_to_pi(x[2, :]) * 180.0 / np.pi, "C0", label="Actual")
    plt.plot(t, wrap_to_pi(x_hat[2, :]) * 180.0 / np.pi, "C1--", label="Estimated")
    plt.plot(t, wrap_to_pi(x_d[2, :]) * 180.0 / np.pi, "C2--", label="Desired")
    plt.ylabel("theta [deg]")
    plt.grid(color="0.95")
    plt.setp(ax2c, xticklabels=[])
    ax2d = plt.subplot(814)
    plt.plot(t, wrap_to_pi(x[3, :]) * 180.0 / np.pi, "C0", label="Actual")
    plt.plot(t, wrap_to_pi(x_hat[3, :]) * 180.0 / np.pi, "C1--", label="Estimated")
    plt.plot(t, wrap_to_pi(x_d[3, :]) * 180.0 / np.pi, "C2--", label="Desired")
    plt.ylabel("phi [deg]")
    plt.grid(color="0.95")
    plt.setp(ax2d, xticklabels=[])
    ax2e = plt.subplot(815)
    plt.step(t, wrap_to_pi(u[0, :]) * 180.0 / np.pi, "C0", label="Actual")
    plt.plot(t, wrap_to_pi(u_d[0, :]) * 180.0 / np.pi, "C2--", label="Desired")
    plt.ylabel("v [m/s]")
    plt.grid(color="0.95")
    plt.setp(ax2e, xticklabels=[])
    ax2f = plt.subplot(816)
    plt.step(t, u[1, :] * 180.0 / np.pi, "C0", label="Actual")
    plt.plot(t, u_d[1, :] * 180.0 / np.pi, "C2--", label="Desired")
    plt.ylabel("omega [rad/s]")
    plt.grid(color="0.95")
    plt.setp(ax2f, xticklabels=[])
    ax2g = plt.subplot(817)
    plt.plot(t, wrap_to_pi(w) * 180.0 / np.pi, "C0", label="Actual")
    plt.plot(t, wrap_to_pi(w_hat) * 180.0 / np.pi, "C1--", label="Estimated")
    plt.ylabel("wind [deg]")
    plt.grid(color="0.95")
    plt.setp(ax2g, xticklabels=[])
    ax2h = plt.subplot(818)
    plt.plot(t, wrap_to_pi(s_d) * 180.0 / np.pi, "C0", label="Actual")
    plt.plot(t, wrap_to_pi(s) * 180.0 / np.pi, "C1--", label="Estimated")
    plt.ylabel("sail [deg]")
    plt.grid(color="0.95")
    plt.xlabel("t [s]")
    plt.savefig("src_outputs/boat_state_and_inputs.pdf")

    # Plot the estimator errors as a function of time
    sigma = np.zeros((x.shape[0], N))
    fig3 = plt.figure(3)
    fig3.set_figheight(10.0)
    ax3a = plt.subplot(611)
    sigma[0, :] = np.sqrt(s1 * P_hat[0, 0, :])
    plt.fill_between(
        t,
        -sigma[0, :],
        sigma[0, :],
        color="C0",
        alpha=0.2,
        label=str(100 * (1 - ALPHA)) + " % Confidence",
    )
    plt.plot(t, x[0, :] - x_hat[0, :], "C0", label="Error")
    plt.grid(color="0.95")
    plt.ylabel("e_x [m]")
    plt.setp(ax3a, xticklabels=[])
    plt.legend()
    ax3b = plt.subplot(612)
    sigma[1, :] = np.sqrt(s1 * P_hat[1, 1, :])
    plt.fill_between(
        t,
        -sigma[1, :],
        sigma[1, :],
        color="C0",
        alpha=0.2,
        label=str(100 * (1 - ALPHA)) + " % Confidence",
    )
    plt.plot(t, x[1, :] - x_hat[1, :], "C0", label="Error")
    plt.grid(color="0.95")
    plt.ylabel("e_y [m]")
    plt.setp(ax3b, xticklabels=[])
    ax3c = plt.subplot(613)
    sigma[2, :] = np.sqrt(s1 * P_hat[2, 2, :])
    plt.fill_between(
        t,
        -sigma[2, :] * 180.0 / np.pi,
        sigma[2, :] * 180.0 / np.pi,
        color="C0",
        alpha=0.2,
        label=str(100 * (1 - ALPHA)) + " % Confidence",
    )
    plt.plot(t, (x[2, :] - x_hat[2, :]) * 180.0 / np.pi, "C0", label="Error")
    plt.ylabel("e_theta [deg]")
    plt.grid(color="0.95")
    plt.setp(ax3c, xticklabels=[])
    ax3d = plt.subplot(614)
    sigma[3, :] = np.sqrt(s1 * P_hat[3, 3, :])
    plt.fill_between(
        t,
        -sigma[3, :] * 180.0 / np.pi,
        sigma[3, :] * 180.0 / np.pi,
        color="C0",
        alpha=0.2,
        label=str(100 * (1 - ALPHA)) + " % Confidence",
    )
    plt.plot(t, (x[3, :] - x_hat[3, :]) * 180 / np.pi, "C0", label="Error")
    plt.ylabel("e_phi [deg]")
    plt.grid(color="0.95")
    plt.setp(ax3d, xticklabels=[])
    ax3d = plt.subplot(615)
    sigma = np.sqrt(s1 * sigma_wind**2)
    plt.fill_between(
        t,
        -sigma * 180.0 / np.pi,
        sigma * 180.0 / np.pi,
        color="C0",
        alpha=0.2,
        label=str(100 * (1 - ALPHA)) + " % Confidence",
    )
    plt.plot(t, (w - w_hat) * 180 / np.pi, "C0", label="Error")
    plt.ylabel("e_wind [deg]")
    plt.grid(color="0.95")
    plt.setp(ax3d, xticklabels=[])
    ax3e = plt.subplot(616)
    sigma = np.sqrt(s1 * sigma_wind**2)
    plt.fill_between(
        t,
        -sigma * 180.0 / np.pi,
        sigma * 180.0 / np.pi,
        color="C0",
        alpha=0.2,
        label=str(100 * (1 - ALPHA)) + " % Confidence",
    )
    plt.plot(t, (s_d - s) * 180 / np.pi, "C0", label="Error")
    plt.ylabel("e_sail [deg]")
    plt.grid(color="0.95")
    plt.xlabel("t [s]")
    plt.savefig("src_outputs/boat_state_errors.pdf")

    # Animate & Save Gif
    vehicle.animate(x, x_d, x_hat, w, w_hat, s, P_hat, ALPHA, T, map_size, f_map, 
                    dest_pos, save_ani=True, filename='src_outputs/boat_animation.gif')

    # Show all the plots to the screen
    plt.show()


def main() -> None:
    # Set the simulation time [s] and the sample period [s]
    sim_time = 20.0
    T = 0.05

    # Create an array of time values [s]
    t = np.arange(0.0, sim_time, T)
    N = np.size(t)

    # MOBILE ROBOT SETUP
    # Set the wheelbase and track of the vehicle [m]
    ell_w = 0.1
    ell_t = 0.1

    # Create vehicle
    vehicle = Tricycle(ell_w, ell_t)

    # Create wind sensor
    wind_sense = WindSensor(N)

    # CREATE A MAP OF FEATURES
    num_features = 4
    # Set the size [m] of a square map
    map_size = 2.0
    # Create a map of randomly placed feature locations
    f_map = np.zeros((2, num_features))
    f_map[:, 0] = (0.0     , 0.0     )
    f_map[:, 1] = (0.0     , map_size)
    f_map[:, 2] = (map_size, 0.0     )
    f_map[:, 3] = (map_size, map_size)
    # Add destination position
    dest_pos = (1.75, 1.75)

    # SET UP THE NOISE COVARIANCE MATRICES
    # Range sensor [m] and IMU [rad] noise standard deviations
    SIGMA_W = (0.1, 0.1)
    # Speed [m/s] and Steering Rate [rad/s] noise standard deviations
    SIGMA_U = (0.1, 0.1)
    # Wind angle [rad] noise standard deviation
    SIGMA_WIND = 0.1
    # Set constant wind angle
    ABS_WIND_ANLGE = 2.5*np.pi/4.0

    # Sensor noise covariance matrix
    R = np.zeros((num_features+1, num_features+1))
    R[0:num_features, 0:num_features] = np.diag([SIGMA_W[0]**2]*num_features)
    R[num_features, num_features]     = SIGMA_W[1]**2

    # Process noise covariance matrix
    Q = np.diag([SIGMA_U[0]**2, SIGMA_U[1]**2])

    # Initialize arrays that will be populated with our inputs and states
    num_states = 4
    num_inputs = 2
    x     = np.zeros((num_states, N))
    u     = np.zeros((num_inputs, N))
    x_hat = np.zeros((num_states, N))
    P_hat = np.zeros((num_states, num_states, N))
    w     = np.zeros(N)
    w_hat = np.zeros(N)
    s_d   = np.zeros(N)
    s = np.zeros(N)

    # Initialize the state estimate
    # Assume in the middle of the box with no knowledge of theta or phi
    x_hat[:, 0] = (0.25, 0.25, np.pi/4, 0.0)
    # Assume we are very sure it's in the box, very unsure of theta, phi within several degrees of 0
    P_hat[:, :, 0] = np.diag([0.5**2, 0.5**2, (np.pi/2.0)**2, 0.1**2])

    # Set the initial pose [m, m, rad, rad] -> NOTE: Assumes we try to place in the same pose every time
    x[:, 0] = x_hat[:, 0] + 0.1*np.random.randn()
    # Set the initial velocities [m/s, rad/s] -> Initial speed cannot be 0
    u[:, 0] = (0.1, 0.0)

    # Set the inital wind direction (relatve to boat)
    w[0] = ABS_WIND_ANLGE - x[2, 0]
    # Set the initial sail angle
    s_d[0] = trim_sail(Angle.exp(x[2, 0]), Angle.exp(w[0])).log
    # Get initial wind measurement estimate
    w_hat[0] = wind_sense.wind_sensor(0, w[0], x_hat[:, 0], SIGMA_WIND)
    # Get initial sail angle
    s[0] = trim_sail(Angle.exp(x_hat[2, 0]), Angle.exp(w_hat[0])).log

    # Constant desired speed and angle
    v_d = 0.1            # [m/s]
    theta_d = np.pi/8.0  # [rad]

    # Initial desired position, speed, and turning rate
    x_d = np.zeros((4, N))
    u_d = np.zeros((2, N))
    x_d[:, 0] = (x_hat[:, 0][0], x_hat[:, 0][1], theta_d, 0.0)
    u_d[:, 0] = (v_d, 0.0)

    # Pre-compute the desired trajectory and inputs
    for k in range(1, N):
        x_d[0, k] = x_d[0, 0] + t[k]*v_d*np.cos(theta_d)
        x_d[1, k] = x_d[1, 0] + t[k]*v_d*np.sin(theta_d)
        x_d[2, k] = theta_d
        x_d[3, k] = 0.0
        u_d[0, k] = v_d
        u_d[1, k] = 0.0

    # Run simulation
    for k in range(1, N):
        # Simulate the robot's motion
        x[:, k] = rk_four(vehicle.f, x[:, k-1], u[:, k-1], T)
        w[k]    = ABS_WIND_ANLGE - x[2, k-1]

        # Take measurements 
        z = get_measurements(x[:, k], SIGMA_W, f_map)

        # Use the measurements to estimate the robot's state
        x_hat[:, k], P_hat[:, :, k] = ekf(vehicle, T, x_hat[:, k-1], P_hat[:, :, k-1], u[:, k-1], z, Q, R, f_map)

        # Take wind measurement seperately
        w_hat[k] = wind_sense.wind_sensor(k, w[k-1], x_hat[:, k-1], SIGMA_WIND)

        # Compute the controls (v, omega)
        # u[:, k] = fsf(vehicle, x_d[:, k-1], u_d[:, k-1], x_hat[:, k-1])
        # TODO: Bool to select method
        u[:, k] = mpc(vehicle, T, N, k, x_d, u_d, x_hat)

        # Trim the sail
        s_d[k] = trim_sail(Angle.exp(x[2, k-1])    , Angle.exp(w[k-1])).log
        s[k]   = trim_sail(Angle.exp(x_hat[2, k-1]), Angle.exp(w_hat[k-1])).log
        

    # Plot the results
    plot_results(vehicle, t, N, T, f_map, x, x_hat, x_d, u, u_d, 
                 w, w_hat, s_d, s, SIGMA_WIND, P_hat, map_size, dest_pos)


if __name__ == '__main__':
    main()
