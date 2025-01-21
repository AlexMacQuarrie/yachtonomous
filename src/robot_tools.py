import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches
from scipy.stats import chi2
from tools import Angle


def trim_sail(rel_wind_angle:Angle, crit_angle:Angle=Angle.exp(20, deg=True)) -> Angle:
    # Since rel_wind_angle is [-180, 180], division is fine
    if rel_wind_angle.log >= 0:
        sail_angle = Angle.exp(max( crit_angle.log, rel_wind_angle.log/2.0))
    else:
        sail_angle = Angle.exp(min(-crit_angle.log, rel_wind_angle.log/2.0))

    return sail_angle


def rk_four(f, x, u, T):
    """
    Perform fourth-order Runge-Kutta numerical integration.

    The function to integrate is f(x, u, params), where the state variables are
    collected in the variable x, we assume a constant input vector u over time
    interval T > 0, and params is an array of the system's parameters.
    """
    k_1 = f(x, u)
    k_2 = f(x + T * k_1 / 2.0, u)
    k_3 = f(x + T * k_2 / 2.0, u)
    k_4 = f(x + T * k_3, u)
    x_new = x + T / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)
    return x_new


def draw_rectangle(x, y, length, width, angle):
    """Finds points that draw a rectangle.

    The rectangle has centre (x, y), a length, width, and angle [rad].
    """
    V = np.zeros((2, 5))
    l = 0.5 * length
    w = 0.5 * width
    V = np.array([[-l, -l, l, l, -l], [-w, w, w, -w, -w]])
    R = np.array([[np.cos(angle), np.sin(-angle)], [np.sin(angle), np.cos(angle)]])
    V = R @ V
    X = V[0, :] + x
    Y = V[1, :] + y
    return X, Y


class Tricycle:
    """
    Tricycle or planar bicycle vehicle class.

    Parameters
    ----------
    ell_W : float
        The wheelbase of the vehicle [m].
    ell_T : float
        The vehicle's track length [m].
    """

    def __init__(self, ell_W, ell_T):
        """Constructor method."""
        self.ell_W = ell_W
        self.ell_T = ell_T

    def f(self, x, u):
        """Tricycle or planar bicycle vehicle kinematic model.

        Parameters
        ----------
        x : ndarray of length 4
            The vehicle's state (x, y, theta, phi).
        u : ndarray of length 2

        Returns
        -------
        f : ndarray of length 4
            The rate of change of the vehicle states.
        """
        f = np.zeros(4)
        f[0] = u[0] * np.cos(x[2])
        f[1] = u[0] * np.sin(x[2])
        f[2] = -u[0] * 1.0 / self.ell_W * np.tan(x[3])
        f[3] = u[1]
        return f

    def draw(self, x, y, theta, phi):
        """Finds points that draw a tricycle vehicle.

        The centre of the rear wheel axle is (x, y), the body has orientation
        theta, steering angle phi, wheelbase ell_W and track length ell_T.

        Returns X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B, where L is for the left
        wheel, R is for the right wheel, F is for the single front wheel, and
        BD is for the vehicle's body.
        """
        # Left and right back wheels
        X_L, Y_L = draw_rectangle(
            x - 0.5 * self.ell_T * np.sin(theta) - self.ell_W * np.cos(theta),
            y + 0.5 * self.ell_T * np.cos(theta) - self.ell_W * np.sin(theta),
            0.5 * self.ell_T,
            0.25 * self.ell_T,
            theta + phi,
        )
        X_R, Y_R = draw_rectangle(
            x + 0.5 * self.ell_T * np.sin(theta) - self.ell_W * np.cos(theta),
            y - 0.5 * self.ell_T * np.cos(theta) - self.ell_W * np.sin(theta),
            0.5 * self.ell_T,
            0.25 * self.ell_T,
            theta + phi,
        )
        # Front wheel
        X_F, Y_F = draw_rectangle(
            x,
            y,
            0.5 * self.ell_T,
            0.25 * self.ell_T,
            theta,
        )
        # Body
        X_BD, Y_BD = draw_rectangle(
            x - self.ell_W / 2.0 * np.cos(theta),
            y - self.ell_W / 2.0 * np.sin(theta),
            2.0 * self.ell_W,
            2.0 * self.ell_T,
            theta,
        )
        # Return the arrays of points
        return X_L, Y_L, X_R, Y_R, X_F, Y_F, X_BD, Y_BD

    def animate(
        self,
        x,
        x_d,
        x_hat,
        w,
        w_hat,
        s,
        P_hat,
        alpha,
        T,
        map_size,
        f_map,
        dest_pos,
        save_ani=False,
        filename="animate_tricycle.gif",
        draw_ell=False
    ):
        """
        Create an animation of a tricycle vehicle.

        Returns animation object for array of vehicle positions x with time
        increments T [s], wheelbase ell_W [m], and track ell_T [m].

        To save the animation to a GIF file, set save_ani to True and give a
        filename (default 'animate_tricycle.gif').
        """
        # Wind Arow Stuff
        radius         = np.sqrt(2.0)*map_size/2.0
        arrow_len      = map_size*0.05
        abs_wind_angle = w[0] + x[2, 0]
        wind_arrow     = arrow_len*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle)))
        wa_base        = radius*np.array((np.cos(abs_wind_angle), np.sin(abs_wind_angle))) + (map_size/2.0,map_size/2.0)
        # Sail Stuff
        
        fig, ax = plt.subplots()
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        ax.plot(f_map[0, :], f_map[1, :], "C4*", label="Feature")
        plt.plot(dest_pos[0], dest_pos[1], "C3D", label="Destination")
        plt.plot(x_hat[0, 0], x_hat[1, 0], "C5D", label="Est. Start")
        plt.arrow(wa_base[0]+wind_arrow[0], wa_base[1]+wind_arrow[1], -wind_arrow[0], -wind_arrow[1], head_width=0.05, color='b', label='Wind')
        ax.add_patch(patches.Rectangle((0, 0), map_size, map_size, edgecolor='k', fill=False))
        (line,) = ax.plot([], [], "C0")
        (estimated,) = ax.plot([], [], "--C1")
        (desired,) = ax.plot([], [], "--C2")
        (leftwheel,) = ax.fill([], [], color="k")
        (rightwheel,) = ax.fill([], [], color="k")
        (frontwheel,) = ax.fill([], [], color="k")
        (body,) = ax.fill([], [], color="C0", alpha=0.5)
        est_wind_arrow = ax.arrow([], [], [], [], head_width=0.05, color='C1')
        sail = ax.arrow([], [], [], [], head_width=0, color='k')
        time_text = ax.text(0.05, 0.9, "", transform=ax.transAxes)
        s2 = chi2.isf(alpha, 2)

        def init():
            """A function that initializes the animation."""
            line.set_data([], [])
            estimated.set_data([], [])
            desired.set_data([], [])
            leftwheel.set_xy(np.empty([5, 2]))
            rightwheel.set_xy(np.empty([5, 2]))
            frontwheel.set_xy(np.empty([5, 2]))
            body.set_xy(np.empty([5, 2]))
            est_wind_arrow.set_data()
            sail.set_data()
            time_text.set_text("")
            return line, estimated, desired, leftwheel, rightwheel, frontwheel, body, time_text, est_wind_arrow

        def movie(k):
            """The function called at each step of the animation."""
            # Draw Estimated Wind Arrow
            est_wind_angle = w_hat[k] + x_hat[2, k]
            est_wind_vec   = arrow_len*np.array((np.cos(est_wind_angle), np.sin(est_wind_angle)))
            est_wa_base    = radius*np.array((np.cos(est_wind_angle), np.sin(est_wind_angle))) + (map_size/2.0,map_size/2.0)
            est_wind_arrow.set_data(x=est_wa_base[0]+est_wind_vec[0], 
                                    y=est_wa_base[1]+est_wind_vec[1], 
                                    dx=-est_wind_vec[0], dy=-est_wind_vec[1])
            # Draw Sail
            abs_sail_angle = s[k] + x[2, k]
            sail_arrow     = 0.15*np.array((-np.cos(abs_sail_angle), -np.sin(abs_sail_angle)))
            sail.set_data(x=x[0, k], y=x[1, k], dx=sail_arrow[0], dy=sail_arrow[1])
            # Draw the estimated trajectory
            estimated.set_data(x_hat[0, 0 : k + 1], x_hat[1, 0 : k + 1])
            # Draw the path followed by the vehicle
            line.set_data(x[0, 0 : k + 1], x[1, 0 : k + 1])
            # Draw the desired trajectory
            desired.set_data(x_d[0, 0 : k + 1], x_d[1, 0 : k + 1])
            # Draw the tricycle vehicle
            X_L, Y_L, X_R, Y_R, X_F, Y_F, X_B, Y_B = self.draw(
                x[0, k], x[1, k], x[2, k], x[3, k]
            )
            leftwheel.set_xy(np.transpose([X_L, Y_L]))
            rightwheel.set_xy(np.transpose([X_R, Y_R]))
            frontwheel.set_xy(np.transpose([X_F, Y_F]))
            body.set_xy(np.transpose([X_B, Y_B]))
            # Compute eigenvalues and eigenvectors to find axes for covariance ellipse
            W, V = np.linalg.eig(P_hat[0:2, 0:2, k])
            # Find the index of the largest and smallest eigenvalues
            j_max = np.argmax(W)
            j_min = np.argmin(W)
            ell = patches.Ellipse(
                (x_hat[0, k], x_hat[1, k]),
                2 * np.sqrt(s2 * W[j_max]),
                2 * np.sqrt(s2 * W[j_min]),
                angle=np.arctan2(V[j_max, 1], V[j_max, 0]) * 180 / np.pi,
                alpha=0.2,
                color="C1",
            )
            if draw_ell:
                ax.add_artist(ell)
            # Add the simulation time
            time_text.set_text("t = %.1f s" % (k * T))
            # Set the axis limits 
            ax.set_xbound(lower=-1, upper=3)
            ax.set_ybound(lower=-1, upper=3)
            ax.figure.canvas.draw()
            # Return the objects to animate
            return line, estimated, desired, leftwheel, rightwheel, frontwheel, body, time_text, est_wind_arrow

        # Create the animation
        ani = animation.FuncAnimation(
            fig,
            movie,
            np.arange(1, len(x[0, :]), max(1, int(1 / T / 10))),
            init_func=init,
            interval=T * 1000,
            blit=True,
            repeat=False,
        )
        if save_ani is True:
            ani.save(filename, fps=min(1 / T, 10))
        # Return the figure object
        return ani
