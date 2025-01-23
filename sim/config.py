import json
import math
from typing import Tuple


class settings:
    def __init__(self, data:dict):
        for key, value in data.items():
            if len(key) > 3 and key[-4:] == '_deg':
                key   = key[:-4]
                value = math.radians(value)
            setattr(self, key, value)

    def __getattr__(self, name):
        # Property-like behavior for private attributes
        if f"{name}" in self.__dict__:
            return getattr(self, f"{name}")
        raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{name}'")
    
    def __setattr__(self, name, value):
        # Raise an error if someone tries to modify an attribute
        if name in self.__dict__:
            raise AttributeError(f"Cannot modify attribute '{name}', settings are read-only")
        super().__setattr__(name, value)


def parse_config() -> Tuple[settings]:
    with open('sim_config.json', 'r') as config_file:
        data = json.load(config_file)

    return (settings(data['sim']),
            settings(data['control']),
            settings(data['boat']), 
            settings(data['test']),
            settings(data['initial']), 
            settings(data['noise']))  
