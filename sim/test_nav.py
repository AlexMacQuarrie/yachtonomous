import argparse
import numpy as np
from time import time
from tools import Angle, PlotCtrl
from navigation import NavigationError, plot_course


def _test_navigation_individual() -> None:
    boat = np.array((0.25, 0.25))
    dest = np.array((1.75, 1.75))

    boat_angle = Angle.exp(10, deg=True)
    wind_angle = Angle.exp(70, deg=True)
    crit_angle = Angle.exp(60, deg=True)

    padding = 0.05

    plot_course(boat, dest, wind_angle, boat_angle, crit_angle, border_pad=padding, 
                plot_ctrl=PlotCtrl.ALWAYS)


def _test_navigation() -> None:
    # Create permutations
    crits       = [Angle.exp(i, deg=True) for i in range(30, 70, 10)]
    winds       = [Angle.exp(i, deg=True) for i in range(-180, 180, 20)]
    boats       = [np.array((0.25+i/10, 0.25+j/10)) for i in range(-2, 3) for j in range(-2, 3)]
    dests       = [np.array((1.75+i/10, 1.75+j/10)) for i in range(-2, 3) for j in range(-2, 3)]
    boat_angles = [Angle.exp(i, deg=True) for i in range(10, 90, 10)]

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


if __name__ == '__main__':
    # Parse CLI args
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', dest='indiv', default=False, action='store_true', help='Run individual tests')
    args = parser.parse_args()
    indiv_en = args.indiv

    # Run tests
    if args.indiv: 
        _test_navigation_individual()
    else:          
        _test_navigation()