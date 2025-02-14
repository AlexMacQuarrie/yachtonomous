# External
import numpy as np
# Internal
from tools import arr, sec2
from config import settings


def integrate_inputs(prev:arr, rate:arr, T:float, max_eta:float, max_phi:float) -> arr:
    ''' Integrate rate inputs to actual servo angle inputs, including noise '''
    u_act    = prev + rate*T
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
