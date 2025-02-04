# External
import math
import numpy as np
import matplotlib.pyplot as plt
# Internal
from tools import Angle, PlotCtrl


class NavigationError(Exception):
    pass


def _is_upwind(path_angle:Angle, abs_wind_angle:Angle, crit_angle_wind:Angle) -> bool:
    ''' Check if boat moving upwind '''
    anglediff = path_angle - abs_wind_angle
    return -crit_angle_wind.log < round(anglediff.log, 6) < crit_angle_wind.log


def _is_out_of_bounds(current_pos:np.ndarray, boat_vec:np.ndarray,
                     x_lim_l:float, x_lim_h:float, 
                     y_lim_l:float, y_lim_h:float) -> bool:
    ''' Check if boat is out of bounds '''
    return (current_pos[0] < x_lim_l and boat_vec[0] < 0
         or current_pos[0] > x_lim_h and boat_vec[0] > 0
         or current_pos[1] < y_lim_l and boat_vec[1] < 0
         or current_pos[1] > y_lim_h and boat_vec[1] > 0)


def plot_course(boat_pos        :np.ndarray,
                dest_pos        :np.ndarray,
                wind_angle      :Angle,
                boat_angle      :Angle,
                crit_angle_wind :Angle,
                border_pad      :float      = 0.0,
                point_dist      :float      = 0.05,
                dest_thresh     :float      = 0.1,
                max_length      :int        = 300,
                plot_ctrl       :PlotCtrl   = PlotCtrl.ON_FAIL,
                true_start      :np.ndarray = None
                ) -> np.ndarray:
    ''' Plot course from start to destination '''
    # Check crit angle (relative to wind direction, which is relative to boat direction)
    if not (math.radians(0) < crit_angle_wind.log < math.radians(90)):
        raise NavigationError('Crit angle must be in (0, 90)')

    # Ensure point distance provides sufficient resoltion
    # Technically this is not actually sufficient but should be good enough in practice
    if point_dist > dest_thresh:
        raise NavigationError('Point distance must be small enough to ensure we do not miss the desintation')

    # Initial calculations
    course         = []
    state          = []
    path_vector    = dest_pos-boat_pos
    distance       = np.linalg.norm(path_vector)
    if distance < dest_thresh:
        raise NavigationError(f'Boat={boat_pos} and Destination={dest_pos} are too close')
    path_vector    = point_dist*path_vector/distance  # Scaled to step correct distance
    path_angle     = Angle.exp(np.arctan2(path_vector[1], path_vector[0]))
    abs_wind_angle = wind_angle + boat_angle
    crit_angle     = abs_wind_angle - crit_angle_wind
    left_crit      = crit_angle + crit_angle_wind + crit_angle_wind

    # Get x and y limits
    # Assumes boat is left and below destination
    if true_start is None:
        x_lim_l = boat_pos[0] - border_pad
        x_lim_h = dest_pos[0] + border_pad
        y_lim_l = boat_pos[1] - border_pad
        y_lim_h = dest_pos[1] + border_pad
    else:
        x_lim_l = true_start[0] - border_pad
        x_lim_h = dest_pos[0]   + border_pad
        y_lim_l = true_start[1] - border_pad
        y_lim_h = dest_pos[1]   + border_pad

    '''
        Old way of setting limits. More general, but not neccessary
    
        # Get x and y limits (min window default was 1.0)
        x_lim_l = min(boat_pos[0], dest_pos[0])
        x_lim_h = max(boat_pos[0], dest_pos[0])
        y_lim_l = min(boat_pos[1], dest_pos[1])
        y_lim_h = max(boat_pos[1], dest_pos[1])

        mid_x = (x_lim_l+x_lim_h)/2.0
        x_lim_l = min(x_lim_l, mid_x - min_window/2.0)
        x_lim_h = max(x_lim_h, mid_x + min_window/2.0)

        mid_y = (y_lim_l+y_lim_h)/2.0
        y_lim_l = min(y_lim_l, mid_y - min_window/2.0)
        y_lim_h = max(y_lim_h, mid_y + min_window/2.0)
    '''

    # Assume plotting doesn't fail
    failed = False
    
    # Plot course upwind or directly
    if not _is_upwind(path_angle, abs_wind_angle, crit_angle_wind):
        ### Direct trajectory
        num_points = math.ceil(distance/point_dist)
        for i in range(num_points):
            x_y = boat_pos + (i+1)*path_vector
            theta = Angle.exp(np.arctan2(path_vector[1], path_vector[0])).log
            course.append(x_y)
            state.append((x_y[0], x_y[1], theta, 0.0))
            if np.linalg.norm(dest_pos-course[-1]) < dest_thresh:
                break
    else:
        ### Upwind trajectory
        # Rotation matrix for tacking
        # transpose == inverse, so can tack back using the transpose
        tack_angle = crit_angle_wind + crit_angle_wind
        tack_matrix = np.array(
            ((tack_angle.cos, -tack_angle.sin), 
             (tack_angle.sin,  tack_angle.cos))
        )
        
        # Get initial upwind path vector
        # Selects the tack that is closer to the boat's initial heading
        boat_vec  = point_dist*np.array((left_crit.cos , left_crit.sin ))
        other_vec = point_dist*np.array((crit_angle.cos, crit_angle.sin))
        boat_angle_vec = np.array((boat_angle.cos, boat_angle.sin))
        if np.dot(boat_angle_vec, other_vec) > np.dot(boat_angle_vec, boat_vec):
            boat_vec, other_vec = other_vec, boat_vec
            tack_matrix         = tack_matrix.transpose()

        # If the vector we are using will immediately take boat out of bounds,
        # then use the other vector
        first_point = boat_pos + boat_vec
        if _is_out_of_bounds(first_point, boat_vec, x_lim_l, x_lim_h, y_lim_l, y_lim_h):
            boat_vec, other_vec = other_vec, boat_vec
            tack_matrix         = tack_matrix.transpose()

        # Set initial upwind conditions
        past_layline = False
        x_y   = boat_pos + boat_vec
        theta = Angle.exp(np.arctan2(boat_vec[1], boat_vec[0])).log
        course.append(x_y)
        state.append((x_y[0], x_y[1], theta, 0.0))

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
                if not _is_upwind(angle_to_dest, abs_wind_angle, crit_angle_wind):
                    past_layline = True
                    boat_vec     = boat_vec @ tack_matrix
                    tack_matrix  = tack_matrix.transpose()
                # Going out of set bounds, tack
                elif _is_out_of_bounds(current_pos, boat_vec, x_lim_l, x_lim_h, y_lim_l, y_lim_h):
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
            x_y   = current_pos + boat_vec
            theta = Angle.exp(np.arctan2(boat_vec[1], boat_vec[0])).log
            course.append(x_y)
            state.append((x_y[0], x_y[1], theta, 0.0))


    # Convert to np array now that size is fixed
    course = np.asarray(course)
    state = np.asarray(state)

    # Early exit if not plotting
    if (plot_ctrl == PlotCtrl.NEVER 
    or (    failed and plot_ctrl == PlotCtrl.ON_PASS) 
    or (not failed and plot_ctrl == PlotCtrl.ON_FAIL)):
        if failed: raise err
        return state.T
    
    # Set some tunable plot sizing multipliers
    # All of these will be multiplied by the plot size
    layline_len = 0.5
    arrow_width = 0.02
    boat_length = 0.1
    wind_length = 0.2
    dot_size    = 5

    # Get plot sizes
    plot_size = max(y_lim_h-y_lim_l, x_lim_h-x_lim_l)

    # Get values to center the course in the plot
    x_ticks = np.linspace(x_lim_l, x_lim_l+plot_size, int(plot_size+1))
    y_ticks = np.linspace(y_lim_l, y_lim_l+plot_size, int(plot_size+1))

    # Get layline points
    r_ll_point = dest_pos + (layline_len*plot_size)*np.array((-crit_angle.cos, -crit_angle.sin))
    l_ll_point = dest_pos + (layline_len*plot_size)*np.array((-left_crit.cos , -left_crit.sin ))

    # Plot boat and destination
    plt.plot(boat_pos[0], boat_pos[1], 'go')
    plt.plot(dest_pos[0], dest_pos[1], 'ro')
    # Plot course (points and line)
    plt.scatter(course[:, 0], course[:, 1], s=int(dot_size*plot_size), c='b')
    plt.plot(course[:, 0], course[:, 1], c='b')
    # Plot x or y limits
    plt.plot((x_lim_l, x_lim_l), (y_lim_l, y_lim_h), 'k')
    plt.plot((x_lim_h, x_lim_h), (y_lim_l, y_lim_h), 'k')
    plt.plot((x_lim_l, x_lim_h), (y_lim_l, y_lim_l), 'k')
    plt.plot((x_lim_l, x_lim_h), (y_lim_h, y_lim_h), 'k')
    # Plot laylines
    plt.plot((dest_pos[0], r_ll_point[0]), (dest_pos[1], r_ll_point[1]), 'm')
    plt.plot((dest_pos[0], l_ll_point[0]), (dest_pos[1], l_ll_point[1]), 'm')
    # Add vectors for boat and wind angle
    arrow_width = arrow_width*plot_size
    boat_arrow = (boat_length*plot_size)*np.array((boat_angle.cos, boat_angle.sin))
    plt.arrow(boat_pos[0], boat_pos[1], boat_arrow[0], boat_arrow[1], head_width=arrow_width, color='g')
    wind_arrow = (wind_length*plot_size)*np.array((-abs_wind_angle.cos, -abs_wind_angle.sin))
    plt.arrow(dest_pos[0], dest_pos[1], wind_arrow[0], wind_arrow[1], head_width=arrow_width, color='b')
    # Resize to square plot with equivalent axis to preserve geometry
    # Content will be centered in the plot
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    plt.gca().set_aspect('equal')
    plt.savefig('outputs/navigation_drawing.pdf')
    plt.show()

    if failed: raise err
    return state.T