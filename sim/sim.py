# External
import numpy as np
# Internal
from config import parse_config
from boat_model import boat, integrate_inputs
from control import mpc
from localization import ekf
from sensor import get_measurements, wind_sensor
from navigation import plot_course
from plot import plot_results
from tools import Angle, PlotCtrl, rk_four

'''
If needed, try dynamics with 9 states (add speed for x, y, theta as states)
    - Sensors/ekf change a bit (add un-integrated IMU, integrated acceleration for x_dot and y_dot)
    - Technically current EKF is wrong because IMU measures theta_dot, which is not a state
        - Adding theta_dot as a state is ridiculously complicated in current form
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
    R = np.zeros((num_features+3, num_features+3))
    R[0:num_features, 0:num_features] = np.diag([noise_config.sensor_noise[0]**2]*num_features)
    for i in range(3):
        R[num_features+i, num_features+i] = noise_config.sensor_noise[i]**2
    Q = np.diag(np.power(noise_config.input_noise, 2))

    # Get initial theta estimate
    theta_dest = np.arctan2(test_config.dest_pos[1]-boat_config.est_start_pos[1], 
                            test_config.dest_pos[0]-boat_config.est_start_pos[0])
    angle_diff = np.abs(test_config.abs_wind_angle - theta_dest)
    if angle_diff < boat_config.crit_wind_angle:
        if test_config.abs_wind_angle < np.pi/4.0:
            init_theta_est = np.pi/2.0
        else:
            init_theta_est = 0.0
    else:
        init_theta_est = theta_dest

    # Get initial state estimate
    x_hat_init    = boat_config.est_start_pos
    x_hat_init[2] = init_theta_est
    x_hat_init[3] = test_config.abs_wind_angle - x_hat_init[2]
    x_hat_init[3] = wind_sensor(x_hat_init, noise_config.sensor_noise[2])

    # Compute desired point distance given est. max possible boat speed
    point_speed = test_config.point_speed_factor*sailboat.max_speed(
        x_hat_init[3], boat_config.crit_wind_angle
    )
    point_dist  = test_config.T*point_speed

    # Get course to destination (desired state and inputs)
    x_d, u_d = plot_course(
        np.asarray(x_hat_init[:2]), 
        np.asarray(test_config.dest_pos),
        Angle.exp(x_hat_init[3]), 
        Angle.exp(x_hat_init[2]), 
        Angle.exp(boat_config.crit_wind_angle), 
        border_pad  = test_config.border_pad, 
        point_dist  = point_dist, 
        dest_thresh = test_config.dest_thresh*sailboat.length, 
        max_length  = 5//point_dist, 
        plot_ctrl   = PlotCtrl.ALWAYS
    )

    # Create an array of time values
    N = x_d.shape[1]
    t = np.arange(0.0, N*test_config.T, test_config.T)[:N]

    # Initialize inputs (est, act) states (des, act)
    P_hat = np.zeros((sailboat.num_states, sailboat.num_states, N))
    x     = np.zeros((sailboat.num_states, N))
    x_hat = np.zeros((sailboat.num_states, N))
    u     = np.zeros((sailboat.num_inputs, N))
    u_act = np.zeros((sailboat.num_inputs, N))

    # Initialize state, inputs, state estimate, state uncertainty
    x_hat[:, 0]    = x_hat_init
    x[:, 0]        = x_hat[:, 0] + np.random.randn()*np.asarray(noise_config.start_noise)
    P_hat[:, :, 0] = np.diag(np.power(noise_config.state_noise, 2))
    u[:, 0]        = control_config.init_inputs
    u_act[:, 0]    = (x_hat[5, 0], x_hat[4, 0])

    # Run simulation
    for k in range(1, N):
        # Simulate the robot's motion
        x[:, k] = rk_four(sailboat.f, x[:, k-1], u[:, k-1], 
                          test_config.T, 
                          boat_config.max_phi,
                          boat_config.max_eta)

        # Take measurements 
        z = get_measurements(x[:, k], x_hat[:, k-1], u[:, k-1], sailboat.f, test_config.exp_parms, 
                             noise_config.sensor_noise, f_map, test_config.T)

        # Use the measurements to estimate the robot's state
        x_hat[:, k], P_hat[:, :, k] = ekf(sailboat, test_config.exp_parms, test_config.T, 
                                          x_hat[:, k-1], P_hat[:, :, k-1], u[:, k-1], z, Q, R, f_map)

        # Feedback control (servo rates)
        u[:, k] = mpc(sailboat, control_config, test_config.T, x_d[:, k:], x_hat[:, k])

        # Integrate to get actual servo inputs
        u_act[:, k] = integrate_inputs(u_act[:, k-1], u[:, k], 
                                       test_config.T, 
                                       boat_config.max_eta,
                                       boat_config.max_phi,
                                       noise_config.input_noise)

    # Plot the results
    plot_results(sailboat, t, N, test_config.T, f_map, x, x_hat, x_d, u, u_d, u_act, P_hat, 
                 test_config.pool_size, test_config.dest_pos)


if __name__ == '__main__':
    simulate()
