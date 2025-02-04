# External
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from scipy.stats import chi2
# Internal
from boat_model import boat
from tools import arr, wrap_to_pi


def plot_results(vehicle:boat, t:arr, N:int, T:float, f_map:arr, 
                 x:arr, x_hat:arr, x_d:arr, u:arr, u_d:arr, 
                 w:arr, w_hat:arr, s_d:arr, s:arr, sigma_wind:arr, 
                 P_hat:arr, map_size:arr, dest_pos:arr) -> None:
    ''' Plot simulation results '''
    # Plot settings
    plt.rc('savefig', format='pdf')
    plt.rc('savefig', bbox='tight')

    # Find the scaling factor for plotting covariance bounds. 0.01 -> 99% Confidence Region
    ALPHA = 0.01
    s1    = chi2.isf(ALPHA, 1)

    # Wind arrow plotting stuff
    radius         = np.sqrt(2.0)*np.mean(map_size)/2.0
    arrow_len      = np.mean(map_size)*0.1
    abs_wind_angle = w[0] + x[2, 0]
    wind_arrow     = arrow_len*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle)))
    wa_base        = radius*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle))) + \
                     (map_size[0]/2.0, map_size[1]/2.0)
    min_wind_angle = np.min(w_hat + x_hat[2, :])
    min_wind_arrow = arrow_len*np.array((np.cos(min_wind_angle), np.sin(min_wind_angle)))
    min_wa_base    = radius*np.array((np.cos(min_wind_angle), np.sin(min_wind_angle))) + \
                     (map_size[0]/2.0, map_size[1]/2.0)
    max_wind_angle = np.max(w_hat + x_hat[2, :])
    max_wind_arrow = arrow_len*np.array((np.cos(max_wind_angle), np.sin(max_wind_angle)))
    max_wa_base    = radius*np.array((np.cos(max_wind_angle), np.sin(max_wind_angle))) + \
                     (map_size[0]/2.0, map_size[1]/2.0)    

    # Sail Plotting
    abs_sail_angle = s[0] + x[2, 0]
    sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
    plt.arrow(x[0, 0], x[1, 0], sail_arrow[0], sail_arrow[1], head_width=0, color='k')
    abs_sail_angle = s[-1] + x[2, -1]
    sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
    plt.arrow(x[0, -1], x[1, -1], sail_arrow[0], sail_arrow[1], head_width=0, color='k')

    # Plot the position of the vehicle in the plane
    plt.arrow(min_wa_base[0]+min_wind_arrow[0], min_wa_base[1]+min_wind_arrow[1], 
              -min_wind_arrow[0], -min_wind_arrow[1], head_width=0.05, color='C1', label='Min Wind Est.')
    plt.arrow(max_wa_base[0]+max_wind_arrow[0], max_wa_base[1]+max_wind_arrow[1], 
              -max_wind_arrow[0], -max_wind_arrow[1], head_width=0.05, color='C1', label='Max Wind Est.')
    plt.arrow(wa_base[0]+wind_arrow[0], wa_base[1]+wind_arrow[1], -wind_arrow[0], 
              -wind_arrow[1], head_width=0.05, color='b', label='Wind')
    plt.plot(f_map[0, :], f_map[1, :], 'C4*', label='Feature')
    plt.plot(dest_pos[0], dest_pos[1], 'C3D', label='Destination')
    plt.plot(x_hat[0, 0], x_hat[1, 0], 'C5D', label='Est. Start')
    plt.gca().add_patch(patches.Rectangle((0, 0), map_size[0], map_size[1], edgecolor='k', fill=False))
    plt.plot(x[0, :]    , x[1, :]    , 'C0'  , label='Actual')
    plt.plot(x_hat[0, :], x_hat[1, :], 'C1--', label='Estimated')
    plt.plot(x_d[0, :]  , x_d[1, :]  , 'C2--', label='Desired')
    plt.axis('equal')
    X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B = vehicle.draw(*x[:, 0])
    plt.fill(X_L, Y_L, 'k')
    plt.fill(X_R, Y_R, 'k')
    plt.fill(X_F, Y_F, 'k')
    plt.fill(X_B, Y_B, 'C2', alpha=0.5, label='Start')
    X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B = vehicle.draw(*x[:, N-1])
    plt.fill(X_L, Y_L, 'k')
    plt.fill(X_R, Y_R, 'k')
    plt.fill(X_F, Y_F, 'k')
    plt.fill(X_B, Y_B, 'C3', alpha=0.5, label='End')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.savefig('outputs/boat_drawing.pdf')

    # Plot the states and inputs as a function of time
    fig2 = plt.figure(2)
    fig2.set_figheight(10.0)
    ax2a = plt.subplot(811)
    plt.plot(t, x[0, :]    , 'C0'  , label='Actual')
    plt.plot(t, x_hat[0, :], 'C1--', label='Estimated')
    plt.plot(t, x_d[0, :]  , 'C2--', label='Desired')
    plt.grid(color='0.95')
    plt.ylabel('x [m]')
    plt.setp(ax2a, xticklabels=[])
    plt.legend()
    ax2b = plt.subplot(812)
    plt.plot(t, x[1, :]    , 'C0'  , label='Actual')
    plt.plot(t, x_hat[1, :], 'C1--', label='Estimated')
    plt.plot(t, x_d[1, :]  , 'C2--', label='Desired')
    plt.grid(color='0.95')
    plt.ylabel('y [m]')
    plt.setp(ax2b, xticklabels=[])
    ax2c = plt.subplot(813)
    plt.plot(t, wrap_to_pi(x[2, :])    *180.0/np.pi, 'C0'  , label='Actual')
    plt.plot(t, wrap_to_pi(x_hat[2, :])*180.0/np.pi, 'C1--', label='Estimated')
    plt.plot(t, wrap_to_pi(x_d[2, :])  *180.0/np.pi, 'C2--', label='Desired')
    plt.ylabel('theta [deg]')
    plt.grid(color='0.95')
    plt.setp(ax2c, xticklabels=[])
    ax2d = plt.subplot(814)
    plt.plot(t, wrap_to_pi(x[3, :])    *180.0/np.pi, 'C0'  , label='Actual')
    plt.plot(t, wrap_to_pi(x_hat[3, :])*180.0/np.pi, 'C1--', label='Estimated')
    plt.plot(t, wrap_to_pi(x_d[3, :])  *180.0/np.pi, 'C2--', label='Desired')
    plt.ylabel('phi [deg]')
    plt.grid(color='0.95')
    plt.setp(ax2d, xticklabels=[])
    ax2e = plt.subplot(815)
    plt.step(t, u[0, :]  , 'C0'  , label='Actual')
    plt.plot(t, u_d[0, :], 'C2--', label='Desired')
    plt.ylabel('v [m/s]')
    plt.grid(color='0.95')
    plt.setp(ax2e, xticklabels=[])
    ax2f = plt.subplot(816)
    plt.step(t, wrap_to_pi(u[1, :])  *180.0/np.pi, 'C0'  , label='Actual')
    plt.plot(t, wrap_to_pi(u_d[1, :])*180.0/np.pi, 'C2--', label='Desired')
    plt.ylabel('omega [deg/s]')
    plt.grid(color='0.95')
    plt.setp(ax2f, xticklabels=[])
    ax2g = plt.subplot(817)
    plt.plot(t, wrap_to_pi(w)    *180.0/np.pi, 'C0'  , label='Actual')
    plt.plot(t, wrap_to_pi(w_hat)*180.0/np.pi, 'C1--', label='Estimated')
    plt.ylabel('wind [deg]')
    plt.grid(color='0.95')
    plt.setp(ax2g, xticklabels=[])
    ax2h = plt.subplot(818)
    plt.plot(t, wrap_to_pi(s)  *180.0/np.pi, 'C0'  , label='Actual')
    plt.plot(t, wrap_to_pi(s_d)*180.0/np.pi, 'C2--', label='Desired')
    plt.ylabel('sail [deg]')
    plt.grid(color='0.95')
    plt.xlabel('t [s]')
    plt.savefig('outputs/boat_state_and_inputs.pdf')

    # Plot the estimator errors as a function of time
    sigma = np.zeros((x.shape[0], N))
    fig3 = plt.figure(3)
    fig3.set_figheight(10.0)
    ax3a = plt.subplot(611)
    sigma[0, :] = np.sqrt(s1*P_hat[0, 0, :])
    plt.fill_between(
        t,
        -sigma[0, :],
         sigma[0, :],
        color = 'C0',
        alpha = 0.2,
        label = str(100*(1-ALPHA)) + ' % Confidence',
    )
    plt.plot(t, x[0, :]-x_hat[0, :], 'C0', label='Error')
    plt.grid(color='0.95')
    plt.ylabel('e_x [m]')
    plt.setp(ax3a, xticklabels=[])
    plt.legend()
    ax3b = plt.subplot(612)
    sigma[1, :] = np.sqrt(s1*P_hat[1, 1, :])
    plt.fill_between(
        t,
        -sigma[1, :],
         sigma[1, :],
        color = 'C0',
        alpha = 0.2,
        label = str(100*(1-ALPHA))+' % Confidence',
    )
    plt.plot(t, x[1, :] - x_hat[1, :], 'C0', label='Error')
    plt.grid(color='0.95')
    plt.ylabel('e_y [m]')
    plt.setp(ax3b, xticklabels=[])
    ax3c = plt.subplot(613)
    sigma[2, :] = np.sqrt(s1*P_hat[2, 2, :])
    plt.fill_between(
        t,
        -sigma[2, :]*180.0/np.pi,
         sigma[2, :]*180.0/np.pi,
        color = 'C0',
        alpha = 0.2,
        label = str(100*(1-ALPHA)) + ' % Confidence',
    )
    plt.plot(t, (x[2, :]-x_hat[2, :])*180.0/np.pi, 'C0', label='Error')
    plt.ylabel('e_theta [deg]')
    plt.grid(color='0.95')
    plt.setp(ax3c, xticklabels=[])
    ax3d = plt.subplot(614)
    sigma[3, :] = np.sqrt(s1*P_hat[3, 3, :])
    plt.fill_between(
        t,
        -sigma[3, :]*180.0/np.pi,
         sigma[3, :]*180.0/np.pi,
        color = 'C0',
        alpha = 0.2,
        label = str(100*(1-ALPHA)) + ' % Confidence',
    )
    plt.plot(t, (x[3, :]-x_hat[3, :])*180/np.pi, 'C0', label='Error')
    plt.ylabel('e_phi [deg]')
    plt.grid(color='0.95')
    plt.setp(ax3d, xticklabels=[])
    ax3e  = plt.subplot(615)
    sigma = np.sqrt(s1*sigma_wind**2)
    plt.fill_between(
        t,
        -sigma*180.0/np.pi,
         sigma*180.0/np.pi,
        color = 'C0',
        alpha = 0.2,
        label = str(100*(1-ALPHA)) + ' % Confidence',
    )
    plt.plot(t, (w-w_hat)*180/np.pi, 'C0', label='Error')
    plt.ylabel('e_wind [deg]')
    plt.grid(color='0.95')
    plt.setp(ax3e, xticklabels=[])
    ax3f  = plt.subplot(616)
    sigma = np.sqrt(s1*sigma_wind**2)
    plt.fill_between(
        t,
        -sigma*180.0/np.pi,
         sigma*180.0/np.pi,
        color = 'C0',
        alpha = 0.2,
        label = str(100*(1-ALPHA)) + ' % Confidence',
    )
    plt.plot(t, (s_d-s)*180/np.pi, 'C0', label='Error')
    plt.ylabel('e_sail [deg]')
    plt.grid(color='0.95')
    plt.xlabel('t [s]')
    plt.savefig('outputs/boat_state_errors.pdf')

    # Animate & Save Gif
    vehicle.animate(x, x_d, x_hat, w, w_hat, s, P_hat, ALPHA, T, map_size, f_map, 
                    dest_pos, save_ani=True, filename='outputs/boat_animation.gif')

    # Show all the plots to the screen
    plt.show()
