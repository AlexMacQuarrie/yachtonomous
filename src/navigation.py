# External
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
# Internal
from tools import Angle, arr


class NavigationError(Exception):
    ''' Custom error to check in nav testing '''
    pass


def _is_upwind(path_angle:Angle, abs_wind_angle:Angle, 
               crit_angle_wind:Angle) -> bool:
    ''' Check if boat moving upwind '''
    anglediff = path_angle - abs_wind_angle
    return -crit_angle_wind.log < round(anglediff.log, 6) < crit_angle_wind.log


def _is_out_of_bounds(current_pos:arr, boat_vec:arr,
                      x_lim_l:float, x_lim_h:float, 
                      y_lim_l:float, y_lim_h:float) -> bool:
    ''' Check if boat is out of bounds '''
    return (current_pos[0] < x_lim_l and boat_vec[0] < 0
         or current_pos[0] > x_lim_h and boat_vec[0] > 0
         or current_pos[1] < y_lim_l and boat_vec[1] < 0
         or current_pos[1] > y_lim_h and boat_vec[1] > 0)


def plot_course(boat_pos        :arr,
                dest_pos        :arr,
                wind_angle      :Angle,
                boat_angle      :Angle,
                crit_angle_wind :Angle,
                border_pad      :float,
                point_dist      :float,
                dest_thresh     :float,
                max_length      :int
                ) -> arr:
    ''' Plot course from start to destination '''
    # Check crit angle (relative to wind direction, which is relative to boat direction)
    if not (0 < crit_angle_wind.log < np.pi/2):
        raise NavigationError('Crit angle must be in (0, 90)')

    # Ensure point distance provides sufficient resoltion
    # Technically this is not actually sufficient but should be good enough in practice
    if point_dist > dest_thresh:
        raise NavigationError('Point distance must be small enough to ensure we do not miss the desintation')

    # Initial calculations
    state          = []
    path_vector    = dest_pos-boat_pos
    distance       = np.linalg.norm(path_vector)
    path_vector    = point_dist*path_vector/distance  # Scaled to step correct distance
    path_angle     = Angle.exp(np.arctan2(path_vector[1], path_vector[0]))
    abs_wind_angle = wind_angle + boat_angle
    crit_angle     = abs_wind_angle - crit_angle_wind
    left_crit      = crit_angle + crit_angle_wind + crit_angle_wind

    # Check we aren't starting too close
    if distance < dest_thresh:
        raise NavigationError(f'Boat={boat_pos} and Destination={dest_pos} are too close')

    # Get x and y limits
    # Assumes boat is left and below destination
    x_lim_l = boat_pos[0] - border_pad
    x_lim_h = dest_pos[0] + border_pad
    y_lim_l = boat_pos[1] - border_pad
    y_lim_h = dest_pos[1] + border_pad
    
    # Plot course upwind or directly
    if not _is_upwind(path_angle, abs_wind_angle, crit_angle_wind):
        ### Direct trajectory
        num_points = np.ceil(distance/point_dist)
        for i in range(num_points):
            x_y   = boat_pos + (i+1)*path_vector
            theta = Angle.exp(np.arctan2(path_vector[1], path_vector[0]))
            gamma = abs_wind_angle - theta
            state.append((x_y[0], x_y[1], theta.log, gamma.log, 0.0, gamma.log/2.0))

            if np.linalg.norm(dest_pos-state[-1][:2]) < dest_thresh:
                break
    else:
        ### Upwind trajectory
        # Rotation matrix for tacking
        # transpose == inverse, so can tack back using the transpose
        tack_angle = crit_angle_wind + crit_angle_wind
        tack_matrix = np.array(
            ((tack_angle.cos, -tack_angle.sin), 
             (tack_angle.sin,  tack_angle.cos)),
            dtype=float
        )
        
        # Get initial upwind path vector
        # Selects the tack that is closer to the boat's initial heading
        boat_vec  = point_dist*np.array((left_crit.cos , left_crit.sin ), dtype=float)
        other_vec = point_dist*np.array((crit_angle.cos, crit_angle.sin), dtype=float)
        boat_angle_vec = np.array((boat_angle.cos, boat_angle.sin), dtype=float)
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
        theta = Angle.exp(np.arctan2(boat_vec[1], boat_vec[0]))
        gamma = abs_wind_angle - theta
        state.append((x_y[0], x_y[1], theta.log, gamma.log, 0.0, gamma.log/2.0))

        # Plot upwind course
        while True:
            # Get current position info
            current_pos   = state[-1][:2]
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
                    boat_vec     = np.dot(boat_vec, tack_matrix)
                    tack_matrix  = tack_matrix.transpose()
                # Going out of set bounds, tack
                elif _is_out_of_bounds(current_pos, boat_vec, x_lim_l, x_lim_h, y_lim_l, y_lim_h):
                    boat_vec    = np.dot(boat_vec, tack_matrix)
                    tack_matrix = tack_matrix.transpose()

            # Error out if we somehow fail to each the destination in a reasonable time
            if len(state) > max_length:
                raise NavigationError(f'Course failed to reach the destination after {max_length} points')

            # Extend course
            x_y   = current_pos + boat_vec
            theta = Angle.exp(np.arctan2(boat_vec[1], boat_vec[0]))
            gamma = abs_wind_angle - theta
            state.append((x_y[0], x_y[1], theta.log, gamma.log, 0.0, gamma.log/2.0))

    # Convert to np array now that size is fixed
    return np.asarray(state, dtype=float).T
