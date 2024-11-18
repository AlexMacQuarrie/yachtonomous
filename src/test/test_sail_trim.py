# Unit tests for sail trim simulation module
from sim.sail_trim import trim_sail
from tools.angle import Angle
from tools.plotting import PlotCtrl


def test_sail_trim_individual() -> None:
    boat_angle     = Angle.exp(90, deg=True)
    rel_wind_angle = Angle.exp(-90, deg=True)

    trim_sail(boat_angle, rel_wind_angle, plot_ctrl=PlotCtrl.ALWAYS)
