# External
import numpy as np
# Internal
from tools import arr, sec2, saturate
from config import settings


def integrate_inputs(prev:arr, rate:arr, T:float, max_eta:float, max_phi:float) -> arr:
    ''' Integrate rate inputs to actual servo angle inputs, including noise '''
    u_act    = prev + rate*T
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
        gamma = max(abs(gamma), crit_angle)
        return self.s*(self.a*gamma**4 + self.b*gamma**2 + self.c)
