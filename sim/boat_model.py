# External
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches
from typing import Tuple
# Internal
from tools import arr, draw_rectangle, sec2, saturate
from config import settings


def integrate_inputs(prev:arr, rate:arr, T:float, 
                     max_eta:float, max_phi:float,
                     sigma_w:list) -> arr:
    ''' Integrate rate inputs to actual servo angle inputs, including noise '''
    u_act = prev + (rate+np.asarray(sigma_w)*np.random.randn())*T
    u_act[0] = saturate(u_act[0], max_eta)
    u_act[1] = saturate(u_act[1], max_phi)
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

    def f(self, x:arr, u:arr) -> arr:
        ''' Kinematic model, x_dot = A(q) + B*u '''
        f = np.zeros(self.num_states)
        f[0] =  self.s*np.cos(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.cos(x[2])              # x
        f[1] =  self.s*np.cos(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.sin(x[2])              # y
        f[2] = -self.s*np.cos(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.tan(x[4])/self.length  # theta
        f[3] = -f[2]                                                                                            # gamma
        f[4] =  saturate(u[1], self.max_phi_dot)                                                                # phi
        f[5] =  saturate(u[0], self.max_eta_dot)                                                                # eta
        return f
    
    def A(self, x:arr) -> arr:
        ''' Linearization of model w.r.t. state '''
        dx_dt = -self.s* np.cos(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.sin(x[2])
        dx_dg =  self.s*(np.sin(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c) + np.cos(2*x[5]-x[3])*(4*self.a*x[3]**3 + 2*self.b*x[3]))*np.cos(x[2])
        dx_de = -self.s* np.sin(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.cos(x[2])*2

        dy_dt =  self.s* np.cos(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.cos(x[2])
        dy_dg =  self.s*(np.sin(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c) + np.cos(2*x[5]-x[3])*(4*self.a*x[3]**3 + 2*self.b*x[3]))*np.sin(x[2])
        dy_de = -self.s* np.sin(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.sin(x[2])*2

        dt_dg =  self.s*(np.sin(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c) + np.cos(2*x[5]-x[3])*(4*self.a*x[3]**3 + 2*self.b*x[3]))*np.tan(x[4])/self.length
        dt_dp = -self.s* np.cos(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*  sec2(x[4])/self.length
        dt_de =  self.s* np.sin(2*x[5]-x[3])*(self.a*x[3]**4 + self.b*x[3]**2 + self.c)*np.tan(x[4])*2/self.length

        dg_dg = -dt_dg
        dg_dp = -dt_dp
        dg_de = -dt_de

        return np.array(
            [
                [0, 0, dx_dt, dx_dg, 0,     dx_de],
                [0, 0, dy_dt, dy_dg, 0,     dy_de],
                [0, 0, 0,     dt_dg, dt_dp, dt_de],
                [0, 0, 0,     dg_dg, dg_dp, dg_de],
                [0, 0, 0,     0,     0,     0    ],
                [0, 0, 0,     0,     0,     0    ],
            ]
        )

    def B(self) -> arr:
        ''' Linearization of model w.r.t. inputs '''
        return np.array(
            [
                [0, 0],
                [0, 0], 
                [0, 0],
                [0, 0],
                [0, 1],
                [1, 0],
            ]
        )

    def F(self, T:float, x:arr) -> arr:
        ''' Discretization of A via Euler integration '''
        return np.eye(self.num_states) + T*self.A(x)

    def G(self, T:float) -> arr:
        ''' Discretization of B via Euler integration '''
        return T*self.B()
    
    def max_speed(self, gamma:float, crit_angle:float) -> float:
        ''' Get max possible boat speed given rel wind angle '''
        gamma = max(np.abs(gamma), crit_angle)
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
            ax.set_xbound(lower=-1, upper=3)
            ax.set_ybound(lower=-1, upper=3)
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
