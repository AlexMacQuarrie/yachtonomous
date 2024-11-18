# Module for sail trim simulation
import numpy as np
import matplotlib.pyplot as plt
from tools.angle import Angle
from tools.plotting import PlotCtrl


def plot_boat(center_pos:np.ndarray, boat_angle:Angle, 
              sail_angle:Angle, rel_wind_angle:Angle) -> None:
    # NOTE: Currently in box from [-1,-1] to [1,1] and scaled accordingly
    
    # Plot boat arrow
    boat_arrow = 0.5*np.array((boat_angle.cos, boat_angle.sin))
    plt.arrow(center_pos[0], center_pos[1], boat_arrow[0], boat_arrow[1], head_width=0.05, color='g', length_includes_head=True)
    # Plot wind arrow
    abs_wind_angle = boat_angle + rel_wind_angle
    wind_arrow = 0.5*np.array((abs_wind_angle.cos, abs_wind_angle.sin))
    plt.arrow(center_pos[0]+wind_arrow[0], center_pos[1]+wind_arrow[1], -wind_arrow[0], -wind_arrow[1], head_width=0.05, color='b', length_includes_head=True)
    # Plot sail arrow (line)
    abs_sail_angle = boat_angle + sail_angle
    sail_arrow = 0.3*np.array((-abs_sail_angle.cos, -abs_sail_angle.sin))
    plt.arrow(center_pos[0], center_pos[1], sail_arrow[0], sail_arrow[1], head_width=0, color='k', length_includes_head=True)
    # Draw boat
    rot_matrix = np.array(((boat_angle.cos, -boat_angle.sin), 
                           (boat_angle.sin,  boat_angle.cos))).T
    bow     = np.array(( 0.25,  0   )) @ rot_matrix + center_pos
    l_stern = np.array((-0.3 ,  0.15)) @ rot_matrix + center_pos
    r_stern = np.array((-0.3 , -0.15)) @ rot_matrix + center_pos
    plt.plot((l_stern[0], r_stern[0]), (l_stern[1], r_stern[1]), 'k')
    plt.plot((l_stern[0], bow[0]    ), (l_stern[1], bow[1]    ), 'k')
    plt.plot((bow[0]    , r_stern[0]), (bow[1]    , r_stern[1]), 'k')
    plt.fill((l_stern[0], r_stern[0], bow[0]), (l_stern[1], r_stern[1], bow[1]), 'k', alpha=0.1)
    # Square plot with equivalent axis to preserve geometry
    plt.xticks([-1, 1])
    plt.yticks([-1, 1])
    plt.gca().set_aspect('equal')
    plt.show()


def trim_sail(boat_angle:Angle, rel_wind_angle:Angle, crit_angle:Angle=Angle.exp(20, deg=True),
              plot_ctrl:PlotCtrl=PlotCtrl.ON_FAIL) -> Angle:
    # Since rel_wind_angle is [-180, 180], division is fine
    if rel_wind_angle.log >= 0:
        sail_angle = Angle.exp(max( crit_angle.log, rel_wind_angle.log/2.0))
    else:
        sail_angle = Angle.exp(min(-crit_angle.log, rel_wind_angle.log/2.0))

    # Only plot if desired
    if plot_ctrl == PlotCtrl.NEVER or plot_ctrl == PlotCtrl.ON_FAIL:
        return sail_angle
    
    # Plot the boat
    plot_boat(np.array((0, 0)), boat_angle, sail_angle, rel_wind_angle)

    return sail_angle
