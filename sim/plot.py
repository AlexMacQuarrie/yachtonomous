# External
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from scipy.stats import chi2
# Internal
from boat_model import boat
from tools import arr


def plot_results(vehicle:boat, t:arr, N:int, T:float, f_map:arr, x:arr, x_hat:arr, 
                 x_d:arr, u:arr, u_d:arr, P_hat:arr, map_size:arr, dest_pos:arr) -> None:
    ''' Plot simulation results '''
    # Plot settings
    plt.rc('savefig', format='pdf')
    plt.rc('savefig', bbox='tight')

    # Find the scaling factor for plotting covariance bounds. 0.01 -> 99% Confidence Region
    ALPHA = 0.01
    s1    = chi2.isf(ALPHA, 1)

    # Wind arrow plotting
    radius         = np.sqrt(2.0)*np.mean(map_size)/2.0
    arrow_len      = np.mean(map_size)*0.1
    abs_wind_angle = x[2, 0] + x[3, 0]
    wind_arrow     = arrow_len*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle)))
    wa_base        = radius*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle))) + \
                     (map_size[0]/2.0, map_size[1]/2.0)   
    min_wind_angle = np.min(x_hat[2, :] + x_hat[3, :])
    min_wind_arrow = arrow_len*np.array((np.cos(min_wind_angle), np.sin(min_wind_angle)))
    min_wa_base    = radius*np.array((np.cos(min_wind_angle), np.sin(min_wind_angle))) + \
                     (map_size[0]/2.0, map_size[1]/2.0)
    max_wind_angle = np.max(x_hat[2, :] + x_hat[3, :])
    max_wind_arrow = arrow_len*np.array((np.cos(max_wind_angle), np.sin(max_wind_angle)))
    max_wa_base    = radius*np.array((np.cos(max_wind_angle), np.sin(max_wind_angle))) + \
                     (map_size[0]/2.0, map_size[1]/2.0)  

    # Sail Plotting
    abs_sail_angle = x[5, 0] + x[2, 0]
    sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
    plt.arrow(x[0, 0], x[1, 0], sail_arrow[0], sail_arrow[1], head_width=0, color='k')
    abs_sail_angle = x[5, -1] + x[2, -1]
    sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
    plt.arrow(x[0, -1], x[1, -1], sail_arrow[0], sail_arrow[1], head_width=0, color='k')

    # Plot the start/end position of the vehicle in the plane
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
    ylabels_states = ['x [m]', 'y [m]', 'theta [deg]', 
                      'gamma [deg]', 'phi [deg]', 'eta [deg]']
    ylabels_inputs = ['sigma [deg/s]', 'omega [deg/s]']
    for i in range(vehicle.num_states):
        ax2 = plt.subplot(int(str(vehicle.num_states+vehicle.num_inputs)+'1'+str(i+1)))
        if 'deg' in ylabels_states[i]:
            plt.plot(t, x[i, :]    *180.0/np.pi, 'C0'  , label='Actual')
            plt.plot(t, x_hat[i, :]*180.0/np.pi, 'C1--', label='Estimated')
            plt.plot(t, x_d[i, :]  *180.0/np.pi, 'C2--', label='Desired')
        else:
            plt.plot(t, x[i, :]    , 'C0'  , label='Actual')
            plt.plot(t, x_hat[i, :], 'C1--', label='Estimated')
            plt.plot(t, x_d[i, :]  , 'C2--', label='Desired')
        plt.grid(color='0.95')
        plt.ylabel(ylabels_states[i])
        plt.setp(ax2, xticklabels=[])
        if i == 0:
            plt.legend()
    for i in range(vehicle.num_inputs):
        ax2 = plt.subplot(int(str(vehicle.num_states+vehicle.num_inputs)+'1'+str(vehicle.num_states+i+1)))
        if 'deg' in ylabels_inputs[i]:
            plt.step(t, u[i, :]  *180.0/np.pi, 'C0'  , label='Actual')
            plt.plot(t, u_d[i, :]*180.0/np.pi, 'C2--', label='Desired')
        else:
            plt.step(t, u[i, :]  , 'C0'  , label='Actual')
            plt.plot(t, u_d[i, :], 'C2--', label='Desired')
        plt.ylabel(ylabels_inputs[i])
        plt.grid(color='0.95')
        if i != vehicle.num_inputs-1:
            plt.setp(ax2, xticklabels=[])  
    plt.xlabel('t [s]')
    plt.savefig('outputs/boat_state_and_inputs.pdf')

    # Plot the estimator errors as a function of time
    sigma = np.zeros((x.shape[0], N))
    fig3 = plt.figure(3)
    fig3.set_figheight(10.0)
    for i in range(vehicle.num_states):
        ax3 = plt.subplot(int(str(vehicle.num_states)+'1'+str(i+1)))
        sigma[i, :] = np.sqrt(s1*P_hat[i, i, :])
        if 'deg' in ylabels_states[i]:
            plt.fill_between(t, -sigma[i, :]*180.0/np.pi, sigma[i, :]*180.0/np.pi, color = 'C0',
                             alpha = 0.2, label = str(100*(1-ALPHA)) + ' % Confidence')
            plt.plot(t, (x[i, :]-x_hat[i, :])*180.0/np.pi, 'C0', label='Error')
        else:
            plt.fill_between(t, -sigma[i, :]            , sigma[i, :]            , color = 'C0',
                             alpha = 0.2, label = str(100*(1-ALPHA)) + ' % Confidence')
            plt.plot(t,  x[i, :]-x_hat[i, :]             , 'C0', label='Error')
        plt.grid(color='0.95')
        plt.ylabel('e_' + ylabels_states[i])
        if i != vehicle.num_states-1:
            plt.setp(ax3, xticklabels=[])
        if i == 0:
            plt.legend()
    plt.xlabel('t [s]')
    plt.savefig('outputs/boat_state_errors.pdf')

    # Animate & Save Gif
    vehicle.animate(x, x_d, x_hat, T, map_size, f_map, dest_pos, 
                    save_ani=True, filename='outputs/boat_animation.gif')

    # Show all the plots to the screen
    plt.show()
