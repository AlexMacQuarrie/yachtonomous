# Unit test for navigation simulation
import numpy as np
import argparse
from tools.angle import Angle
from tools.plotting import PlotCtrl
from sim.navigation import NavigationError, plot_course


def test_individual() -> None:
    # Override values and use "-i" CLI arg to run a custom test
    boat = np.array((0, 0))
    dest = np.array((-0.5, 2))

    boat_angle = Angle.exp(0 , deg=True)
    wind_angle = Angle.exp(80, deg=True)
    crit_angle = Angle.exp(45, deg=True)

    plot_course(boat, dest, wind_angle, boat_angle, crit_angle, plot_ctrl=PlotCtrl.ALWAYS)


def test_navigation() -> None:
    # Create permutations
    crits = [Angle.exp(i, deg=True) for i in range(30, 65, 5)]
    winds = [Angle.exp(i, deg=True) for i in range(-180, 180, 10)]
    boats = [np.array((i, j)) for i in range(-2, 3) for j in range(-2, 3)]
    dests = [np.array((i, j)) for i in range(-2, 3) for j in range(-3, 3)]
    boat_angle = Angle.exp(0, deg=True)

    # Get number of tests
    num_tests = len(crits)*len(winds)*len(boats)*len(dests)-len(boats)
    print(f'Running {num_tests} tests...')

    # Run all tests
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
    print('All tests passed')


if __name__ == '__main__':
    # Parse CLI args
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', dest='indiv', default=False, action='store_true', help='Run individual test')
    args = parser.parse_args()
    indiv_en = args.indiv

    # Run tests
    if args.indiv: test_individual()
    else:          test_navigation()
