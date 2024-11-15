# Module for navigation simulation
import math
import numpy as np
import matplotlib.pyplot as plt
from tools.angle import Angle
from tools.plotting import PlotCtrl

"""
TODO
- Always need X and Y limit (for high critical angles)
- Initial upwind vector should ensure it's always pointing in bounds to avoid tacking immediately
- Consider switching upwind vector to whichever is closest to boat vector instead of whichever is closes to direct path
"""

class NavigationError(Exception):
    pass


def is_upwind(path_angle:Angle, abs_wind_angle:Angle, crit_angle_wind:Angle) -> bool:
        anglediff = path_angle - abs_wind_angle
        return -crit_angle_wind.log < round(anglediff.log, 6) < crit_angle_wind.log


def plot_course(boat_pos:np.ndarray,
                dest_pos:np.ndarray,
                wind_angle:Angle,
                boat_angle:Angle,
                crit_angle_wind:Angle=Angle.exp(45.0, deg=True),
                min_window:float=1.0,
                point_dist:float=0.05,
                dest_thresh:float=0.1,
                max_length:int=300,
                plot_ctrl:PlotCtrl=PlotCtrl.ON_FAIL
                ) -> np.ndarray:
    # Check crit angle (relative to wind direction, which is relative to boat direction)
    if not (math.radians(0) < crit_angle_wind.log < math.radians(90)):
        raise NavigationError('Crit angle must be in (0, 90)')

    # Ensure point distance provides sufficient resoltion
    # Technically this is not actually sufficient but should be good enough in practice
    if point_dist > dest_thresh:
        raise NavigationError('Point distance must be small enough to ensure we do not miss the desintation')

    # Initial calculations
    course         = []
    path_vector    = dest_pos-boat_pos
    distance       = np.linalg.norm(path_vector)
    if distance < dest_thresh:
        raise NavigationError(f'Boat={boat_pos} and Destination={dest_pos} are too close')
    path_vector    = point_dist*path_vector/distance  # Scaled to step correct distance
    path_angle     = Angle.exp(np.arctan2(path_vector[1], path_vector[0]))
    abs_wind_angle = wind_angle + boat_angle
    crit_angle     = abs_wind_angle - crit_angle_wind
    left_crit      = crit_angle + crit_angle_wind + crit_angle_wind

    # Determine if boat is sailing generally more towards y-axis vs x-axis
    up_or_down = Angle.exp(45, deg=True).log <= abs(path_angle.log) <= Angle.exp(135, deg=True).log
    limit_idx = 0 if up_or_down else 1

    # Get x limits of course (between boat and dest)
    low_limit  = min(boat_pos[limit_idx], dest_pos[limit_idx])
    high_limit = max(boat_pos[limit_idx], dest_pos[limit_idx])
    window     = high_limit - low_limit

    # If x gap between boat and dest is too small, 
    # move limits to +/- x_min/2, centered about middle of boat and dest
    if window < min_window:
        mid         = (low_limit+high_limit)/2.0
        low_limit  = mid - min_window/2.0
        high_limit = mid + min_window/2.0

    # Assume plotting doesn't fail
    failed = False
    
    # Plot course upwind or directly
    if not is_upwind(path_angle, abs_wind_angle, crit_angle_wind):
        ### Direct trajectory
        num_points = math.ceil(distance/point_dist)
        for i in range(num_points):
            course.append(boat_pos + (i+1)*path_vector)
    else:
        ### Upwind trajectory
        # Rotation matrix for tacking
        # transpose == inverse, so can tack back using the transpose
        tack_angle = crit_angle_wind + crit_angle_wind
        tack_matrix = np.array(
            ((math.cos(tack_angle.log), -math.sin(tack_angle.log)), 
             (math.sin(tack_angle.log),  math.cos(tack_angle.log)))
        )
        
        # Get initial upwind path vector
        # Selects the tack that takes it closer to the mark
        boat_vec  = point_dist*np.array((left_crit.cos , left_crit.sin ))
        other_vec = point_dist*np.array((crit_angle.cos, crit_angle.sin))
        if np.dot(path_vector, other_vec) > np.dot(path_vector, boat_vec):
            boat_vec    = other_vec
            tack_matrix = tack_matrix.transpose()

        # Set initial upwind conditions
        past_layline = False
        course.append(boat_pos + boat_vec)

        # Plot upwind course
        while True:
            # Get current position info
            current_pos   = course[-1]
            vec_to_dest   = dest_pos-current_pos
            dist_to_dest  = np.linalg.norm(vec_to_dest)
            angle_to_dest = Angle.exp(np.arctan2(vec_to_dest[1], vec_to_dest[0]))

            # Near destination, stop
            if dist_to_dest < dest_thresh:
                break

            if not past_layline:
                # Going past layline, tack and sail to destination
                if not is_upwind(angle_to_dest, abs_wind_angle, crit_angle_wind):
                    past_layline = True
                    boat_vec     = boat_vec @ tack_matrix
                    tack_matrix  = tack_matrix.transpose()
                # Going out of set bounds, tack
                elif (current_pos[limit_idx] < low_limit  and boat_vec[limit_idx] < 0
                   or current_pos[limit_idx] > high_limit and boat_vec[limit_idx] > 0):
                    boat_vec    = boat_vec @ tack_matrix
                    tack_matrix = tack_matrix.transpose()

            # Error out if we somehow fail to each the destination in a reasonable time
            if len(course) > max_length:
                err = NavigationError(f'Course failed to reach the destination after {max_length} points')
                failed = True
                if plot_ctrl == PlotCtrl.NEVER or plot_ctrl == PlotCtrl.ON_PASS:
                    raise err
                else: break

            # Extend course
            course.append(current_pos + boat_vec)

    # Convert to np array now that size is fixed
    course = np.asarray(course)

    # Early exit if not plotting
    if (plot_ctrl == PlotCtrl.NEVER 
    or (    failed and plot_ctrl == PlotCtrl.ON_PASS) 
    or (not failed and plot_ctrl == PlotCtrl.ON_FAIL)):
        if failed: raise err
        return course

    # Get X & Y limits
    lowest  = min(boat_pos[1], dest_pos[1])
    highest = max(boat_pos[1], dest_pos[1])
    x_lim_left  = min(boat_pos[0], dest_pos[0])
    x_lim_right = max(boat_pos[0], dest_pos[0])

    # Get plot sizes
    y_len = highest - lowest
    x_len = x_lim_right - x_lim_left
    plot_size = max(x_len, y_len)
    prop_dist = 0.5*plot_size

    # Get values to center the course in the plot
    mid_x    = (x_lim_left+x_lim_right)/2.0
    leftmost = mid_x - plot_size/2.0
    mid_y      = (highest+lowest)/2.0
    bottommost = mid_y - plot_size/2.0
    x_ticks = np.linspace(leftmost  , leftmost+plot_size  , int(plot_size+1))
    y_ticks = np.linspace(bottommost, bottommost+plot_size, int(plot_size+1))

    # Get layline points
    r_ll_point = dest_pos + prop_dist*np.array((-crit_angle.cos, -crit_angle.sin))
    l_ll_point = dest_pos + prop_dist*np.array((-left_crit.cos, -left_crit.sin))

    # Plot boat and destination
    plt.plot(boat_pos[0], boat_pos[1], 'go')
    plt.plot(dest_pos[0], dest_pos[1], 'ro')
    # Plot course
    plt.scatter(course[:, 0], course[:, 1], s=10, c='b')
    # Plot x or y limits
    if limit_idx == 0:
        plt.plot((low_limit , low_limit  ), (lowest    , highest   ), 'k')
        plt.plot((high_limit, high_limit ), (lowest    , highest   ), 'k')
    else:
        plt.plot((x_lim_left, x_lim_right), (low_limit , low_limit ), 'k')
        plt.plot((x_lim_left, x_lim_right), (high_limit, high_limit), 'k')
    # Plot laylines
    plt.plot((dest_pos[0], r_ll_point[0]), (dest_pos[1], r_ll_point[1]), 'm')
    plt.plot((dest_pos[0], l_ll_point[0]), (dest_pos[1], l_ll_point[1]), 'm')
    # Add vectors for boat and wind angle
    boat_arrow = (prop_dist/5.0)*np.array((math.cos(boat_angle.log), math.sin(boat_angle.log)))
    plt.arrow(boat_pos[0], boat_pos[1], boat_arrow[0], boat_arrow[1], head_width=0.05, color='g')
    wind_arrow = (prop_dist/2.5)*np.array((-math.cos(abs_wind_angle.log), -math.sin(abs_wind_angle.log)))
    plt.arrow(dest_pos[0], dest_pos[1], wind_arrow[0], wind_arrow[1], head_width=0.05, color='b')
    # Resize to square plot with equivalent axis to preserve geometry
    # Content will be centered in the plot
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    plt.gca().set_aspect('equal')
    plt.show()

    if failed: raise err
    return course
