# Yachtonomous Test Suite
import argparse
from test.test_navigation import test_navigation, test_navigation_individual
from test.test_sail_trim import test_sail_trim_individual


if __name__ == '__main__':
    # Parse CLI args
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', dest='indiv', default=False, action='store_true', help='Run individual tests')
    args = parser.parse_args()
    indiv_en = args.indiv

    # Run tests
    if args.indiv: 
        test_navigation_individual()
        test_sail_trim_individual() 
    else:          
        test_navigation()