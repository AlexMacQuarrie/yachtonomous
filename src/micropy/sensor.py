# External
from micropython import const


# TODO - START: Fake sensor reading while I only have board -------------------
__TODO__ = 0.5

class fake_rssi:
    def __init__(self):
        self.i = 0
        self.list = [111, 1222, 2333, 3444]

    def get(self) -> int:
        rssi   = self.list[self.i]
        self.i = (self.i + 1) % 4
        return rssi

rssi_gen = fake_rssi()
# TODO - END ------------------------------------------------------------------


# Consts
NUM_BEACONS  = const(4)
NUM_READINGS = const(NUM_BEACONS + 3)


class blt_rssi_manager:
    def __init__(self):
        self.__rssi_list = [0, 0, 0, 0]
        
    def update(self, new_rssi_list:list) -> None:
        self.__rssi_list = new_rssi_list
        
    def get_list(self) -> None:
        return self.__rssi_list
        
        
rssi_manager = blt_rssi_manager()


def estimate_initial_gamma(n_samples:int=5) -> float:
    ''' Estimate initial relative wind angle with average '''
    est_gamma = 0.0
    for _ in range(n_samples):
        est_gamma += _wind_sensor()
    return est_gamma/float(n_samples)


def get_measurements() -> list:
    ''' Get all sensor readings '''
    z = [0]*NUM_READINGS
    z[:NUM_BEACONS]  = rssi_manager.get_list()
    z[NUM_BEACONS]   = _imu_gyro()
    z[NUM_BEACONS+1] = _wind_sensor()
    z[NUM_BEACONS+2] = _sail_sensor()
    return z


def _get_updated_rssi_values() -> list:
    ''' Read RSSI from pin until we get a new RSSI from all clients '''
    rssi_list = [0]*NUM_BEACONS
    # Keep reading RSSI values until we get a value from each client
    while 0 in rssi_list:
        serialized_rssi   = rssi_gen.get()  # TODO
        cli_id, rssi      = _parse_serialized_rssi(serialized_rssi)
        rssi_list[cli_id] = rssi
    return rssi_list


def _parse_serialized_rssi(serialized_rssi:int) -> int:
    ''' Deserialized RSSI to original form '''
    cli_id = serialized_rssi//1000
    rssi   = serialized_rssi %1000  # Assumes positive, if negative, use -1000
    # NOTE: May want rssi = -1*rssi depending on curve fit
    return cli_id, rssi


def _imu_gyro() -> float:
    ''' Read theta_dot from IMU '''
    return __TODO__


def _wind_sensor() -> float:
    ''' Read gamma from wind direction sensor '''
    return __TODO__


def _sail_sensor() -> float:
    ''' Read eta from sail direction sensor '''
    return __TODO__
