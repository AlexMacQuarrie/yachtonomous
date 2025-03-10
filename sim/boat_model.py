# External
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches
from typing import Tuple
# Internal
from tools import arr, draw_rectangle, sec2
from config import settings


def integrate_inputs(prev:arr, rate:arr, T:float, 
                     max_eta:float, max_phi:float,
                     sigma_w:list) -> arr:
    ''' Integrate rate inputs to actual servo angle inputs, including noise '''
    u_act = prev + rate*T + sigma_w*np.random.randn()
    u_act[0] = np.clip(u_act[0], -max_eta, max_eta)
    u_act[1] = np.clip(u_act[1], -max_phi, max_phi)
    return u_act


class boat:
    ''' Object to model, draw, and animate the boat '''
    def __init__(self, boat_config:settings, control_config:settings):
        self.length      = boat_config.length
        self.width       = boat_config.width
        self.num_states  = boat_config.num_states_inputs[0]
        self.num_inputs  = boat_config.num_states_inputs[1]
        self.s           = boat_config.speed
        self.a           = boat_config.gamma_coeffs[0]
        self.b           = boat_config.gamma_coeffs[1]
        self.c           = boat_config.gamma_coeffs[2]
        self.max_eta_dot = control_config.input_saturation[0]
        self.max_phi_dot = control_config.input_saturation[1]

        # Caching some frequently created matrices
        self.eye   = np.eye(self.num_states)
        self.f_vec = np.empty(self.num_states)
        self.B_mat = np.array([[0, 0],[0, 0], [0, 0],[0, 0],[0, 1],[1, 0]])
        self.A_mat = np.zeros((self.num_states, self.num_states))

    def f(self, x:arr, u:arr) -> arr:
        ''' Kinematic model, x_dot = A(q) + B*u '''
        gamma_func = self.a*x[3]**4 + self.b*x[3]**2 + self.c
        sail_func  = np.cos(2*x[5]-x[3])
        boat_speed = self.s*sail_func*gamma_func

        self.f_vec[0] =  boat_speed*np.cos(x[2])                             # x
        self.f_vec[1] =  boat_speed*np.sin(x[2])                             # y
        self.f_vec[2] = -boat_speed*np.tan(x[4])/self.length                 # theta
        self.f_vec[3] = -self.f_vec[2]                                       # gamma
        self.f_vec[4] =  np.clip(u[1], -self.max_phi_dot, self.max_phi_dot)  # phi
        self.f_vec[5] =  np.clip(u[0], -self.max_eta_dot, self.max_eta_dot)  # eta
        return self.f_vec
    
    def A(self, x:arr) -> arr:
        ''' Linearization of model w.r.t. state '''
        # Cache multi-use functions
        theta, gamma, phi, eta = x[2], x[3], x[4], x[5]
        gamma_func = self.a*gamma**4 + self.b*gamma**2 + self.c
        d_gamma_f  = 4*self.a*gamma**3 + 2*self.b*gamma
        cos_theta  = np.cos(theta)
        sin_theta  = np.sin(theta)
        tan_phi_l  = np.tan(phi)/self.length
        sec2_phi_l = sec2(phi)/self.length
        cos_sail   = np.cos(2*eta-gamma)
        sin_sail   = np.sin(2*eta-gamma)
        prod_rule  = self.s*(sin_sail*gamma_func + cos_sail*d_gamma_f)
        cos_gamma  = self.s*cos_sail*gamma_func
        sin_gamma  = self.s*sin_sail*gamma_func

        # Jacobian w/ Partial derivatives
        self.A_mat[0, 2] = -cos_gamma*sin_theta
        self.A_mat[0, 3] =  prod_rule*cos_theta
        self.A_mat[0, 5] = -sin_gamma*cos_theta*2

        self.A_mat[1, 2] =  cos_gamma*cos_theta
        self.A_mat[1, 3] =  prod_rule*sin_theta
        self.A_mat[1, 5] = -sin_gamma*sin_theta*2

        self.A_mat[2, 3] =  prod_rule*tan_phi_l
        self.A_mat[2, 4] = -cos_gamma*sec2_phi_l
        self.A_mat[2, 5] =  sin_gamma*tan_phi_l*2

        self.A_mat[3, 3] = -self.A_mat[2, 3]
        self.A_mat[3, 4] = -self.A_mat[2, 4]
        self.A_mat[3, 5] = -self.A_mat[2, 5]

        return self.A_mat

    def B(self) -> arr:
        ''' Linearization of model w.r.t. inputs '''
        return self.B_mat
    
    def F(self, T:float, x:arr) -> arr:
        ''' Discretization of A via Euler integration '''
        return np.eye(self.num_states) + T*self.A(x)

    def G(self, T:float) -> arr:
        ''' Discretization of B via Euler integration '''
        return T*self.B()
    
    def max_speed(self, gamma:float, crit_angle:float) -> float:
        ''' Get max possible boat speed given rel wind angle '''
        gamma = max(abs(gamma), crit_angle)
        return self.s*(self.a*gamma**4 + self.b*gamma**2 + self.c)

    def draw(self, x:float, y:float, theta:float, 
             gamma:float, phi:float, eta:float) -> Tuple[float]:
        ''' Draw the boat using a series of points '''
        board_width = 0.15
        board_len   = 0.40
        # Rudders
        X_L, Y_L = draw_rectangle(
            x - 0.5*self.width*np.sin(theta) - self.length*np.cos(theta),
            y + 0.5*self.width*np.cos(theta) - self.length*np.sin(theta),
            board_len*self.width,
            board_width*self.width,
            theta + phi,
        )
        X_R, Y_R = draw_rectangle(
            x + 0.5*self.width*np.sin(theta) - self.length*np.cos(theta),
            y - 0.5*self.width*np.cos(theta) - self.length*np.sin(theta),
            board_len*self.width,
            board_width*self.width,
            theta + phi,
        )
        # Daggerboard
        X_F, Y_F = draw_rectangle(
            x,
            y,
            board_len*self.width,
            board_width*self.width,
            theta,
        )
        # Body
        X_BD, Y_BD = draw_rectangle(
            x + 0*self.length*np.cos(theta),
            y + 0*self.length*np.sin(theta),
            0.28,
            0.28,
            theta,
        )
        # Return the arrays of points
        return X_L, Y_L, X_R, Y_R, X_F, Y_F, X_BD, Y_BD

    def animate(self, x:arr, x_d:arr, x_hat:arr, T:float, 
                map_size:arr, f_map:arr, dest_pos:arr, 
                save_ani:bool, filename:str) -> animation.FuncAnimation:
        ''' Animation of the boat '''
        # Wind Arow Stuff
        radius         = np.sqrt(2.0)*np.mean(map_size)/2.0
        arrow_len      = np.mean(map_size)*0.05
        abs_wind_angle = x[2, 0] + x[3, 0]
        wind_arrow     = arrow_len*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle)))
        wa_base        = radius*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle))) + \
                         (map_size[0]/2.0,map_size[1]/2.0)
        # Sail Stuff
        fig, ax = plt.subplots()
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.axis('equal')
        ax.plot(f_map[0, :], f_map[1, :], 'C4*', label='Feature')
        plt.plot(dest_pos[0], dest_pos[1], 'C3D', label='Destination')
        plt.plot(x_hat[0, 0], x_hat[1, 0], 'C5D', label='Est. Start')
        plt.arrow(wa_base[0]+wind_arrow[0], wa_base[1]+wind_arrow[1], 
                  -wind_arrow[0], -wind_arrow[1], head_width=0.05, color='b', label='Wind')
        ax.add_patch(patches.Rectangle((0, 0), map_size[0], map_size[1], edgecolor='k', fill=False))
        (line,) = ax.plot([], [], 'C0')
        (estimated,) = ax.plot([], [], '--C1')
        (desired,) = ax.plot([], [], '--C2')
        (leftwheel,) = ax.fill([], [], color='k')
        (rightwheel,) = ax.fill([], [], color='k')
        (frontwheel,) = ax.fill([], [], color='k')
        (body,) = ax.fill([], [], color='C0', alpha=0.5)
        est_wind_arrow = ax.arrow([], [], [], [], head_width=0.05, color='C1')
        sail = ax.arrow([], [], [], [], head_width=0, color='k')
        time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

        def init() -> Tuple:
            ''' A function that initializes the animation '''
            line.set_data([], [])
            estimated.set_data([], [])
            desired.set_data([], [])
            leftwheel.set_xy(np.empty([5, 2]))
            rightwheel.set_xy(np.empty([5, 2]))
            frontwheel.set_xy(np.empty([5, 2]))
            body.set_xy(np.empty([5, 2]))
            est_wind_arrow.set_data()
            sail.set_data()
            time_text.set_text('')
            return line, estimated, desired, leftwheel, rightwheel, \
                   frontwheel, body, time_text, est_wind_arrow

        def movie(k:int) -> Tuple:
            ''' The function called at each step of the animation '''
            # Draw Estimated Wind Arrow
            est_wind_angle = x_hat[2, k] + x_hat[3, k]
            est_wind_vec   = arrow_len*np.array((np.cos(est_wind_angle), np.sin(est_wind_angle)))
            est_wa_base    = radius*np.array((np.cos(est_wind_angle), np.sin(est_wind_angle))) + \
                             (map_size[0]/2.0,map_size[1]/2.0)
            est_wind_arrow.set_data(x=est_wa_base[0]+est_wind_vec[0], 
                                    y=est_wa_base[1]+est_wind_vec[1], 
                                    dx=-est_wind_vec[0], dy=-est_wind_vec[1])
            # Draw Sail
            abs_sail_angle = x[5, k] + x[2, k]
            sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
            sail.set_data(x=x[0, k], y=x[1, k], dx=sail_arrow[0], dy=sail_arrow[1])
            # Draw the estimated trajectory
            estimated.set_data(x_hat[0, 0:k+1], x_hat[1, 0:k+1])
            # Draw the path followed by the vehicle
            line.set_data(x[0, 0:k+1], x[1, 0 : k+1])
            # Draw the desired trajectory
            desired.set_data(x_d[0, 0:k+1], x_d[1, 0:k+1])
            # Draw the Boat vehicle
            X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B = self.draw(*x[:, k])
            leftwheel.set_xy(np.transpose([X_L, Y_L]))
            rightwheel.set_xy(np.transpose([X_R, Y_R]))
            frontwheel.set_xy(np.transpose([X_F, Y_F]))
            body.set_xy(np.transpose([X_B, Y_B]))
            # Add the simulation time
            time_text.set_text('t = %.1f s'%(k*T))
            # Set the axis limits 
            ax.set_xbound(lower=-0.5, upper=map_size[0]+0.5)
            ax.set_ybound(lower=-0.5, upper=map_size[1]+0.5)
            ax.figure.canvas.draw()
            # Return the objects to animate
            return line, estimated, desired, leftwheel, rightwheel, \
                   frontwheel, body, time_text, est_wind_arrow

        # Create the animation
        ani = animation.FuncAnimation(
            fig, movie,
            np.arange(1, len(x[0, :]), max(1, int(1/T/10))),
            init_func =init,
            interval  = T*1000,
            blit      = True,
            repeat    = False,
        )
        if save_ani:
            ani.save(filename, fps=min(1/T, 10))
        # Return the figure object
        return ani
