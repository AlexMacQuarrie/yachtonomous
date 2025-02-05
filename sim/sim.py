# External
import numpy as np
# Internal
from config import parse_config
from boat_model import boat, trim_sail
from control import mpc
from localization import ekf
from sensor import get_measurements, wind_sensor
from navigation import plot_course
from plot import plot_results
from tools import rk_four, Angle, PlotCtrl

'''
W5/6
- Better speed func -> speed(gamma, s)
'''

def simulate() -> None:
    ''' Simulation Entrypoint '''
    # Parse the simulation config
    control_config, boat_config, test_config, noise_config = parse_config()

    # Create sailboat
    sailboat = boat(boat_config, control_config)

    # Create the feature map of distance beacons
    num_features = 4
    f_map        = np.zeros((2, num_features))
    f_map[:, 0]  = (0.0                     , 0.0                     )
    f_map[:, 1]  = (0.0                     , test_config.pool_size[1])
    f_map[:, 2]  = (test_config.pool_size[0], 0.0                     )
    f_map[:, 3]  = (test_config.pool_size[0], test_config.pool_size[1])

    # Sensor and process noise covariance matrices
    R = np.zeros((num_features+2, num_features+2))
    R[0:num_features, 0:num_features] = np.diag([noise_config.sensor_noise[0]**2]*num_features)
    R[num_features  , num_features  ] = noise_config.sensor_noise[1]**2
    R[num_features+1, num_features+1] = noise_config.sensor_noise[2]**2
    Q = np.diag([noise_config.input_noise**2])

    # Get initial state estimate
    x_hat_init    = boat_config.est_start_pos
    x_hat_init[3] = test_config.abs_wind_angle - x_hat_init[2]
    x_hat_init[3] = wind_sensor(x_hat_init, noise_config.sensor_noise[2])

    # Get course to destination (desired state)
    x_d = plot_course(
        np.asarray(x_hat_init[:2]), 
        np.asarray(test_config.dest_pos),
        Angle.exp(x_hat_init[3]), 
        Angle.exp(x_hat_init[2]), 
        Angle.exp(boat_config.crit_wind_angle), 
        border_pad  = test_config.border_pad, 
        point_dist  = test_config.T*boat_config.speed, 
        dest_thresh = test_config.dest_thresh*sailboat.length, 
        max_length  = 5.0/(test_config.T*boat_config.speed), 
        plot_ctrl   = PlotCtrl.ALWAYS
    )

    # Create an array of time values
    N = x_d.shape[1]
    t = np.arange(0.0, N*test_config.T, test_config.T)[:N]

    # Initialize inputs (est, act) states (des, act)
    x, x_hat = np.zeros((sailboat.num_states, N)), np.zeros((sailboat.num_states, N))
    u, u_d, s, s_d = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N)
    P_hat = np.zeros((sailboat.num_states, sailboat.num_states, N))

    # Initialize state, state estimate, state uncertainty
    x_hat[:, 0]    = x_hat_init
    x[:, 0]        = x_hat[:, 0] + np.random.randn()*np.asarray(noise_config.start_noise)
    P_hat[:, :, 0] = np.diag(np.power(noise_config.state_noise, 2))

    # Run simulation
    for k in range(1, N):
        # Simulate the robot's motion
        x[:, k] = rk_four(sailboat.f, x[:, k-1], u[k-1], 
                          test_config.T, 
                          boat_config.max_rudder_angle)

        # Take measurements 
        z = get_measurements(x[:, k], test_config.exp_parms, noise_config.sensor_noise, f_map)

        # Use the measurements to estimate the robot's state
        x_hat[:, k], P_hat[:, :, k] = ekf(sailboat, test_config.exp_parms, test_config.T, 
                                          x_hat[:, k-1], P_hat[:, :, k-1], u[k-1], z, Q, R, f_map)

        # Feedback control (steering rate)
        u[k] = mpc(sailboat, control_config, test_config.T, N, k, x_d, x_hat)

        # Trim the sail
        s[k] = trim_sail(Angle.exp(x_hat[3, k-1]), Angle.exp(boat_config.crit_sail_angle)).log

        # Compute desired sail trim based on true relative wind angle
        s_d[k] = trim_sail(Angle.exp(x[3, k-1]), Angle.exp(boat_config.crit_sail_angle)).log

    # Plot the results
    plot_results(sailboat, t, N, test_config.T, f_map, x, x_hat, x_d, u, u_d, 
                 s_d, s, P_hat, test_config.pool_size, test_config.dest_pos)


if __name__ == '__main__':
    simulate()
