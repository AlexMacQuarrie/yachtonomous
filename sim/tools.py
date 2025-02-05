# External
import numpy as np
import math
from enum import Enum


class arr(np.ndarray):
    pass  # Alias for more concise type hinting


class PlotCtrl(Enum):
    ''' Enum for controlling when to plot figures '''
    ALWAYS  = 0,
    NEVER   = 1,
    ON_PASS = 2,
    ON_FAIL = 3


class Angle:
    ''' Object to help handle angle wrap-around '''
    def __init__(self, sin_theta:float, cos_theta:float) -> None:
        self.sin = sin_theta
        self.cos = cos_theta

    @staticmethod
    def exp(theta:float, deg:bool=False) -> 'Angle':
        theta = math.radians(theta) if deg else theta
        return Angle(math.sin(theta), math.cos(theta))

    @property
    def log(self) -> float:
        return math.atan2(self.sin, self.cos)

    def __mul__(self, other:'Angle') -> 'Angle':
        new_sin = math.sin(self.log + other.log)
        new_cos = math.cos(self.log + other.log)
        return Angle(new_sin, new_cos)

    def __truediv__(self, other:'Angle') -> 'Angle':
        new_sin = math.sin(self.log - other.log)
        new_cos = math.cos(self.log - other.log)
        return Angle(new_sin, new_cos)

    def __add__(self, other:'Angle') -> 'Angle':
        return (self * other)

    def __sub__(self, other:'Angle') -> 'Angle':
        return (self / other)

    def __repr__(self) -> str:
        return '%.2f'%(math.degrees(self.log))
    

def saturate(u:float, sat_limit:float) -> float:
    ''' Clamp inputs to within given limits '''
    return max(u, -sat_limit) if u < 0 else min(u, sat_limit)


def wrap_to_pi(angle:float):
    ''' Wrap angles to the range [-pi, pi]. '''
    return (angle+np.pi)%(2*np.pi)-np.pi


def sec2(x:float) -> float:
    ''' Compute sec^2(x) -> sec^2(x) = tan^2(x) + 1 '''
    return np.tan(x)**2 + 1


def rk_four(f, x, u, T, max_phi):
    ''' Perform fourth-order Runge-Kutta numerical integration '''
    k_1 = f(x, u)
    k_2 = f(x + T*k_1/2.0, u)
    k_3 = f(x + T*k_2/2.0, u)
    k_4 = f(x + T*k_3, u)
    x_new = x + T*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4)/6.0
    # Ensure rudder does not exceed max angle
    x_new[4] = saturate(x_new[4], max_phi)
    return x_new


def draw_rectangle(x, y, length, width, angle):
    ''' Finds points that draw a rectangle '''
    V = np.zeros((2, 5))
    l = 0.5 * length
    w = 0.5 * width
    V = np.array([[-l, -l, l, l, -l], [-w, w, w, -w, -w]])
    R = np.array([[np.cos(angle), np.sin(-angle)], [np.sin(angle), np.cos(angle)]])
    V = R @ V
    X = V[0, :] + x
    Y = V[1, :] + y
    return X, Y
