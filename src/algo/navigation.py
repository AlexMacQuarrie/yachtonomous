# Yachtonomous Navigation Algorithm
# On startup, nagivation will plot a course from the current location to the destination
import numpy as np
from driver.positioning import position_sensor
from driver.windfinder import wind_sensor
from tools.boat_consts import boat_consts


class path:
    # TODO: Maybe add attribute that stores list of checkpoints in order?
    #       Would need a tolerance on these points (attribute)
    #       Might be best to wait until 446 covers this
    pass


class navigation:
    def __init__(self, boat_sensor_pin:int, dest_sensor_pin:int, wind_sensor_pin:int) -> None:
        self.__boat_sensor = position_sensor(boat_sensor_pin)
        self.__dest_sensor = position_sensor(dest_sensor_pin)
        self.__wind_sensor = wind_sensor(wind_sensor_pin)
        self.__path:path   = None

    def get_boat_position(self) -> tuple:
        ''' Return boat (x,y) coords '''
        return self.__boat_sensor.get_position()
    
    def get_dest_position(self) -> tuple:
        ''' Return destination (x,y) coords '''
        return self.__dest_sensor.get_position()
    
    def get_wind_angle(self) -> tuple:
        ''' Return wind angle relative to boat centerline '''
        return self.__wind_sensor.get_angle()
    
    def get_dist_to_dest(self) -> float:
        boat = np.asarray(self.get_boat_position())
        dest = np.asarray(self.get_dest_position())
        dist_vec = boat - dest
        return np.linalg.norm(dist_vec)
    
    def is_at_dest(self) -> bool:
        return self.get_dist_to_dest() < boat_consts.BOAT_LENGTH*boat_consts.DESTINATION_RADIUS
    
    def plot_course(self) -> None:
        # TODO: Not sure how but this will set self.__path using above methods
        #       Also need some validity check here if possible
        pass
