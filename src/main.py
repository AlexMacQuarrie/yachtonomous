# Internal
from yachtonomous import run

'''
TODO - Board only
- Wifi stuff (fake sensor read & actuator inputs on board)
- Update T dynamically

TODO - With PCB
- Drivers (LED, Switch, Power, Servo, IMU, Wind Sensor, Sail Sensor, ADC, Bluetooth)
- Actual sensor readings (BT, IMU, 2x Rotation)
- Servo outputs (Sail, rudder)
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