import numpy as np
from config import parse_config
from boat import boat, trim_sail
from control import mpc, fsf
from localization import ekf
from sensor import WindSensor, get_measurements, estimate_initial_wind
from navigation import plot_course
from plot import plot_results
from tools import Angle, rk_four, PlotCtrl

'''
TODO
W3 -> Navigation Integration
- Add in navigation stuff to the sketch
- Add in navigation stuff to the animation -> just constant on top?
- Make trajectory use actual sailing course
- Clean up plotting code

W4 -> Path Following
- Switch from trajectory tracking to path following (or some pseudo trajectory tracking)

W5 -> Model Updates
- Try adding drift from wind to model (i.e. x_dot = -wind_speed*np.cos(rel_wind_ang+x[2]) + u[0]*np.cos(x[2]))
- Switch to actual sailboat model somehow
'''

def main() -> None:
    # Parse the simulation config
    sim_config, control_config, boat_config, \
    test_config, initial_config, noise_config = parse_config()

    # Create vehicle
    vehicle = boat(boat_config)

    # Create the feature map
    num_features = 4
    f_map        = np.zeros((2, num_features))
    f_map[:, 0]  = (0.0                  , 0.0                  )
    f_map[:, 1]  = (0.0                  , test_config.pool_size)
    f_map[:, 2]  = (test_config.pool_size, 0.0                  )
    f_map[:, 3]  = (test_config.pool_size, test_config.pool_size)

    # Sensor and Process noise covariance matrices
    R = np.zeros((num_features+1, num_features+1))
    R[0:num_features, 0:num_features] = np.diag([noise_config.sensor_noise[0]**2]*num_features)
    R[num_features, num_features]     = noise_config.sensor_noise[1]**2
    Q = np.diag(np.power(noise_config.input_noise, 2))

    # Get course to destination
    init_wind_angle_est = test_config.wind_angle - noise_config.start_noise*initial_config.start_pos[2]
    init_wind_angle_est = estimate_initial_wind(init_wind_angle_est, noise_config.wind_noise)
    course, desired_state = plot_course(
        np.asarray(initial_config.start_pos[:2]), 
        np.asarray(test_config.dest_pos), 
        Angle.exp(init_wind_angle_est), Angle.exp(initial_config.start_pos[2]), 
        Angle.exp(boat_config.crit_wind_angle_deg, deg=True), 
        border_pad=test_config.border_pad, point_dist=0.005, 
        dest_thresh=test_config.dest_thresh*vehicle.ell_W, 
        max_length=1000, plot_ctrl=PlotCtrl.ALWAYS
    )
    desired_state = desired_state.T

    # Create an array of time values [s]
    N        = desired_state.shape[1]
    sim_time = (N-1)*sim_config.T
    t        = np.arange(0.0, sim_time, sim_config.T)

    # Create wind sensor
    wind_sensor = WindSensor(N)

    # Initialize arrays that will be populated with our inputs and states
    num_states = 4 ; num_inputs = 2
    x     = np.zeros((num_states, N))
    u     = np.zeros((num_inputs, N))
    x_hat = np.zeros((num_states, N))
    P_hat = np.zeros((num_states, num_states, N))
    w     = np.zeros(N)
    w_hat = np.zeros(N)
    s_d   = np.zeros(N)
    s     = np.zeros(N)

    # Initialize the state estimate and uncertainty
    x_hat[:, 0]    = initial_config.start_pos
    P_hat[:, :, 0] = np.diag(np.power(noise_config.state_noise, 2))

    # Set the initial pose [m, m, rad, rad] and inputs [m/s, rad/s]
    x[:, 0] = x_hat[:, 0] + noise_config.start_noise*np.random.randn()
    u[:, 0] = initial_config.init_inputs

    # Set the inital wind and sail angles (actual, estimated, desired)
    w[0]     = test_config.wind_angle - x[2, 0]
    w_hat[0] = wind_sensor.read(0, w[0], x_hat[:, 0], noise_config.wind_noise)
    s_d[0]   = trim_sail(Angle.exp(w[0]), Angle.exp(boat_config.crit_sail_angle_deg, deg=True)).log
    s[0]     = 0.0

    # Constant desired speed and angle
    v_d     = 0.1       # [m/s]
    theta_d = np.pi/8.0 # [rad]

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
        x[:, k] = rk_four(vehicle.f, x[:, k-1], u[:, k-1], sim_config.T)
        w[k]    = test_config.wind_angle - x[2, k-1]

        # Take measurements 
        z = get_measurements(x[:, k], test_config.exp_parms, noise_config.sensor_noise, f_map)

        # Use the measurements to estimate the robot's state
        x_hat[:, k], P_hat[:, :, k] = ekf(vehicle, test_config.exp_parms, sim_config.T, x_hat[:, k-1], 
                                          P_hat[:, :, k-1], u[:, k-1], z, Q, R, f_map)

        # Take wind measurement seperately
        w_hat[k] = wind_sensor.read(k, w[k-1], x_hat[:, k-1], noise_config.wind_noise)

        # Compute the controls
        if t[k] >= sim_config.skip_time:
            # Feedback control (speed, steering rate)
            if control_config.mpc_en:
                u[:, k] = mpc(vehicle, control_config, sim_config.T, N, k, x_d, u_d, x_hat)
            else:
                u[:, k] = fsf(vehicle, control_config, x_d[:, k-1], u_d[:, k-1], x_hat[:, k-1])

            # Trim the sail
            s[k] = trim_sail(Angle.exp(w_hat[k-1]), Angle.exp(boat_config.crit_sail_angle_deg, deg=True)).log
        else:
            # Zero-order hold to initial inputs if just localizing initially
            u[:, k] = u[:, k-1]
            s[k]    = s[k-1]

        # Compute desired sail trim based on true relative wind angle
        s_d[k] = trim_sail(Angle.exp(w[k-1]), Angle.exp(boat_config.crit_sail_angle_deg, deg=True)).log
        

    # Plot the results
    plot_results(vehicle, t, N, sim_config.T, f_map, x, x_hat, x_d, u, u_d, 
                 w, w_hat, s_d, s, noise_config.wind_noise, P_hat, 
                 test_config.pool_size, test_config.dest_pos)


if __name__ == '__main__':
    main()