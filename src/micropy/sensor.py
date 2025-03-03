# External
from micropython import const
# Internal
from rotation import wind_and_sail_sensors


# Consts
_NUM_BEACONS  = const(4)
_NUM_READINGS = const(_NUM_BEACONS + 3)
_DEFAULT_RSSI = const(0)


# Object to manage updated RSSI values from BLT sensors
class blt_rssi_manager:
    def __init__(self):
        self.__rssi_list = [_DEFAULT_RSSI]*_NUM_BEACONS
        
    def update(self, new_rssi_list:list) -> None:
        self.__rssi_list = new_rssi_list
        
    def get_list(self) -> list:
        return self.__rssi_list
        
rssi_manager = blt_rssi_manager()


def estimate_initial_gamma(num_samples:int, delay_us:int) -> float:
    ''' Estimate initial relative wind angle with average '''
    _, wind_angle = wind_and_sail_sensors.read_avg_angles(num_samples, delay_us)
    return wind_angle


def get_measurements() -> list:
    ''' Get all sensor readings '''
    sail_angle, wind_angle = wind_and_sail_sensors.read_angles()

    z = [0]*_NUM_READINGS
    z[:_NUM_BEACONS]  = rssi_manager.get_list()
    z[_NUM_BEACONS]   = _imu_theta()
    z[_NUM_BEACONS+1] = wind_angle
    z[_NUM_BEACONS+2] = sail_angle
    return z


# TODO
def _imu_theta() -> float:
    ''' Read theta_dot from IMU '''
    return 0.1

