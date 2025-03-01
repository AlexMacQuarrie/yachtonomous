# External
import numpy as np
from enum import Enum
from typing import Callable, Tuple


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
        self.__sin = sin_theta
        self.__cos = cos_theta

    @staticmethod
    def exp(theta:float, deg:bool=False) -> 'Angle':
        theta = np.radians(theta) if deg else theta
        return Angle(np.sin(theta), np.cos(theta))
    
    @property
    def cos(self) -> float:
        return self.__cos
    
    @property
    def sin(self) -> float:
        return self.__sin

    @property
    def log(self) -> float:
        return np.arctan2(self.__sin, self.__cos)

    def __mul__(self, other:'Angle') -> 'Angle':
        new_sin = np.sin(self.log + other.log)
        new_cos = np.cos(self.log + other.log)
        return Angle(new_sin, new_cos)

    def __truediv__(self, other:'Angle') -> 'Angle':
        new_sin = np.sin(self.log - other.log)
        new_cos = np.cos(self.log - other.log)
        return Angle(new_sin, new_cos)

    def __add__(self, other:'Angle') -> 'Angle':
        return (self * other)

    def __sub__(self, other:'Angle') -> 'Angle':
        return (self / other)

    def __repr__(self) -> str:
        return '%.2f'%(np.degrees(self.log))


def sec2(x:float) -> float:
    ''' Compute sec^2(x) -> sec^2(x) = tan^2(x) + 1 '''
    return np.tan(x)**2 + 1


def rk_four(f:Callable, x:arr, u:arr, T:float, 
            max_phi:float, max_eta:float) -> arr:
    ''' Fourth-order Runge-Kutta numerical integration '''
    k_1 = f(x, u)
    k_2 = f(x + T*k_1/2.0, u)
    k_3 = f(x + T*k_2/2.0, u)
    k_4 = f(x + T*k_3, u)
    x_new = x + T*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4)/6.0
    # Ensure rudder/sail do not exceed max angle
    x_new[4] = np.clip(x_new[4], -max_phi, max_phi)
    x_new[5] = np.clip(x_new[5], -max_eta, max_eta)
    return x_new


def draw_rectangle(x:float, y:float, length:float, width:float, 
                   angle:float) -> Tuple[float, float]:
    ''' Finds points that draw a rectangle '''
    l = 0.5*length
    w = 0.5*width
    V = np.array([[-l, -l, l, l, -l], [-w, w, w, -w, -w]])
    R = np.array([[np.cos(angle), np.sin(-angle)], 
                  [np.sin(angle), np.cos(angle)]])
    V = R @ V
    X = V[0, :] + x
    Y = V[1, :] + y
    return X, Y
