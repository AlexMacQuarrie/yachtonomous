# Module for helper classes and function for handling angle/orientation
import math


class Angle:
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