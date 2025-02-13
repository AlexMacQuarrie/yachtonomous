# Internal
from yachtonomous import run

'''
TODO - Board only
- Wifi stuff (fake sensor read & actuator inputs on board)
- Ensure T is being used right (force constant or update dynamically)

TODO - With PCB
- Actual sensor readings
- Servo outputs
- Updating sample time stuff
- Update config values
'''

def main() -> None:
    # Run the entrypoint
    print('Start run')
    run()
    print('End run')


# Run main
if __name__ == '__main__':
    main()