# External
from time import time
from ulab import numpy as np
# Internal
from sensor import get_measurements
from servo import sail_and_rudder_servos

N = 100
vals_rad = np.zeros((4, N))
T = 0

for i in range(N):
    start = time()

    sail_and_rudder_servos.actuate_servos(0, 0)

    z = get_measurements()
    gyro_z_rps = z[4]
    angle_rad += gyro_z_rps*T
    wind_angle = z[5]
    sail_angle = z[6]

    vals_rad[i] = np.array([gyro_z_rps, angle_rad, wind_angle, sail_angle])

    T = time()-start

means, stds = np.mean(vals_rad, 1), np.std(vals_rad, 1)
print(means, stds)
print(f'gyro_mean_rad={means[0]}             \n theta_std_deg={stds[1]}            \n '
      f'gamma_mean_deg={np.degrees(means[2])}\n gamma_std_deg={np.degrees(stds[2])}\n '
      f'eta_mean_deg={np.degrees(means[3])}  \n eta_std_deg={np.degrees(stds[3])}  \n ')




