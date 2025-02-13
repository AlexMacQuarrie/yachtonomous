# External
import numpy as np


class arr(np.ndarray):
    pass  # Alias for more concise type hinting


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
    

def saturate(u:float, sat_limit:float) -> float:
    ''' Clamp inputs to within given limits '''
    return max(u, -sat_limit) if u < 0 else min(u, sat_limit)


def sec2(x:float) -> float:
    ''' Compute sec^2(x) -> sec^2(x) = tan^2(x) + 1 '''
    return np.tan(x)**2 + 1
