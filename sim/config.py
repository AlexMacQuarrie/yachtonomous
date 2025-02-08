# External
import json
from typing import Tuple
# Internal
from tools import Angle


__CONFIG_PATH__ = 'sim_config.json'


class settings:
    ''' Object that dynamically creates private properties from dict '''
    def __init__(self, data:dict):
        for key, value in data.items():

            # Convert (list of) values given in degrees to rad [-pi, pi]
            if len(key) > 3 and key[-4:] == '_deg':
                key   = key[:-4]
                if isinstance(value, list):
                    value = [Angle.exp(val, deg=True).log for val in value]
                else:
                    value = Angle.exp(value, deg=True).log

            setattr(self, key, value)

    def __getattr__(self, name):
        # Property-like behavior for private attributes
        if f'{name}' in self.__dict__:
            return getattr(self, f'{name}')
        raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{name}'")
    
    def __setattr__(self, name, value) -> None:
        # Raise an error if someone tries to modify an attribute
        if name in self.__dict__:
            raise AttributeError(f"Cannot modify attribute '{name}', settings are read-only")
        super().__setattr__(name, value)


def parse_config() -> Tuple[settings]:
    ''' Parse JSON config into settings objects '''
    with open(__CONFIG_PATH__, 'r') as config_file:
        data = json.load(config_file)

    return (settings(data[category]) for category in data)
