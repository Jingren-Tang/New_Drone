"""
Simulation settings and configuration
"""
from typing import Dict, Any


class Config:
    """
    Configuration class for simulation parameters
    """
    # Simulation settings
    SIMULATION_TIME = 5000
    
    # Drone settings
    DRONE_SPEED = 5
    DRONE_BATTERY_CAPACITY = 300.0
    DRONE_LOW_BATTERY_THRESHOLD = 50.0
    DRONE_INITIAL_COUNT = 5
    CHARGING_STATION_X = 35
    CHARGING_STATION_Y = 35
    
    # Bus settings
    BUS_SPEED = 5
    BUS_STOP_TIME = 3
    BUS_CAPACITY = 3
    
    # Task optimizer settings
    TASK_WINDOW_SIZE = 3
    MAX_QUEUE_SIZE = 10
    
    # Transport mode decision threshold
    # If drone distance is less than bus distance + this threshold, use direct transport
    DIRECT_TRANSPORT_THRESHOLD = 10.0
    
    # Bus lines
    BUS_LINES = ['Line 1', 'Line 2']
    
    @classmethod
    def as_dict(cls) -> Dict[str, Any]:
        """Convert configuration to dictionary"""
        return {
            key: value for key, value in cls.__dict__.items() 
            if not key.startswith('_') and key.isupper()
        }
    
    @classmethod
    def update(cls, **kwargs):
        """Update config values"""
        for key, value in kwargs.items():
            if hasattr(cls, key) and key.isupper():
                setattr(cls, key, value)


def get_config() -> Config:
    """Get the configuration object"""
    return Config