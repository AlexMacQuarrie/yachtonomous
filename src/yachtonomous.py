# External
import numpy as np
# Internal
from config import parse_config
from boat_model import boat, integrate_inputs
from control import mpc
from localization import ekf
from sensor import get_measurements, wind_sensor
from navigation import plot_course
from boat_logging import boat_logger
from tools import Angle


def run() -> None:
    ''' Simulation Entrypoint '''
    # Parse the simulation config
    control_config, boat_config, test_config, noise_config = parse_config()

    # Create sailboat & logger
    sailboat = boat(boat_config, control_config)
    logger   = boat_logger(test_config.log_en, test_config.log_file_name)

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
    Q = np.diag(noise_config.input_noise**2)

    # Get initial state estimate (except theta)
    x_hat_init = boat_config.est_start_pos
    for _ in range(test_config.num_wind_samples):
        x_hat_init[3] += wind_sensor()/float(test_config.num_wind_samples)

    # Get initial theta estimate
    if abs(x_hat_init[3]) > boat_config.crit_wind_angle:
        x_hat_init[2] = np.arctan2(test_config.dest_pos[1]-boat_config.est_start_pos[1], 
                                   test_config.dest_pos[0]-boat_config.est_start_pos[0])
    elif x_hat_init[3] < 0:
        x_hat_init[2] = np.pi/2.0
    else:
        x_hat_init[2] = 0.0

    # Compute desired point distance given est. max possible boat speed
    point_speed = test_config.point_speed_factor*sailboat.max_speed(
        x_hat_init[3], boat_config.crit_wind_angle
    )
    point_dist  = test_config.T*point_speed

    # Get course to destination (desired state and inputs)
    x_d = plot_course(
        np.asarray(x_hat_init[:2]), 
        np.asarray(test_config.dest_pos),
        Angle.exp(x_hat_init[3]), 
        Angle.exp(x_hat_init[2]), 
        Angle.exp(boat_config.crit_wind_angle), 
        border_pad  = test_config.border_pad, 
        point_dist  = point_dist, 
        dest_thresh = test_config.dest_thresh*sailboat.length, 
        max_length  = test_config.max_len_numerator//point_dist, 
    )
    
    # Initialize inputs & states
    P_hat = np.zeros((sailboat.num_states, sailboat.num_states))
    x_hat = np.zeros(sailboat.num_states)
    u     = np.zeros(sailboat.num_inputs)
    u_act = np.zeros(sailboat.num_inputs)

    # Initialize state, inputs, state estimate, state uncertainty
    x_hat = x_hat_init
    P_hat = np.diag(np.array(noise_config.state_noise)**2)
    u     = control_config.init_inputs
    u_act = [x_hat[5], x_hat[4]]

    # Log initial results
    logger.log_results(x_hat, x_d[:, 0], u, u_act, 0.0)

    # Run simulation
    for k in range(1, x_d.shape[1]):
        # Take measurements 
        z = get_measurements(x_hat, num_features, test_config.T)

        # Use the measurements to estimate the boat's state
        x_hat, P_hat = ekf(sailboat, test_config.exp_parms, test_config.T, x_hat, P_hat, u, z, Q, R, f_map)

        # Feedback control (servo rates)
        u = mpc(sailboat, control_config, test_config.T, x_d[:, k:], x_hat)

        # Integrate to get actual servo inputs (angles)
        u_act = integrate_inputs(u_act, u, test_config.T, boat_config.max_eta, boat_config.max_phi)

        # Log results for debug
        logger.log_results(x_hat, x_d[:, k], u, u_act, k*test_config.T)

    # Close logger
    logger.end()
