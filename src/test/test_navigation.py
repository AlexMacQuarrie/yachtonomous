# Unit tests for navigation simulation
import numpy as np
from time import time
from tools.angle import Angle
from tools.plotting import PlotCtrl
from sim.navigation import NavigationError, plot_course


def test_navigation_individual() -> None:
    boat = np.array((0, 0))
    dest = np.array((0.5, 0.5))

    boat_angle = Angle.exp(0 , deg=True)
    wind_angle = Angle.exp(30, deg=True)
    crit_angle = Angle.exp(60, deg=True)

    plot_course(boat, dest, wind_angle, boat_angle, crit_angle, plot_ctrl=PlotCtrl.ALWAYS)


def test_navigation() -> None:
    # Create permutations
    crits = [Angle.exp(i, deg=True) for i in range(30, 70, 10)]
    winds = [Angle.exp(i, deg=True) for i in range(-180, 180, 20)]
    boats = [np.array((i, j)) for i in range(-2, 3) for j in range(-2, 3)]
    dests = [np.array((i, j)) for i in range(-2, 3) for j in range(-3, 3)]
    boat_angles = [Angle.exp(i, deg=True) for i in range(-180, 180, 20)]

    # Get number of tests
    num_tests = len(crits)*len(winds)*len(boats)*len(dests)*len(boat_angles)-len(boats)
    print(f'Running {num_tests} tests...')

    # Run all tests
    start_time = time()
    for boat_angle in boat_angles:
        for boat in boats:
            for dest in dests:
                for crit in crits:
                    for wind in winds:
                        # Skip when boat == dest
                        if np.array_equal(boat, dest):
                            continue
                        # Run the test
                        try:
                            plot_course(boat, dest, wind, boat_angle, crit, plot_ctrl=PlotCtrl.ON_FAIL)
                        except NavigationError as err:
                            print('FAILURE!!!')
                            raise NavigationError(f'Nav error for Boat={boat}, Dest={dest}, Wind={wind}, Crit={crit}, BoatAngle={boat_angle} -> {err}')         
    print(f'All tests passed in {(time()-start_time)/60.0:.2f} minutes')
