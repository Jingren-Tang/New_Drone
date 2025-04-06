"""
Drone entity implementation
"""
import math
from typing import Optional, Dict, Any, Tuple

from .entity import Entity
from ..utils.distance import calculate_distance
from ..utils.events import EventEmitter, global_event_emitter
from ..config.settings import Config


class DroneState:
    """Base class for drone states"""
    
    def execute(self, drone: 'Drone') -> None:
        """Execute state logic"""
        pass


class IdleState(DroneState):
    """Drone is idle and waiting for tasks"""
    
    def execute(self, drone: 'Drone') -> None:
        """Nothing to do when idle"""
        pass


class MovingState(DroneState):
    """Drone is moving toward a target"""
    
    def execute(self, drone: 'Drone') -> None:
        """Move toward target"""
        if drone.target_x is None or drone.target_y is None:
            return
            
        # Calculate distance to target
        distance_to_target = calculate_distance(
            drone.x, drone.y, drone.target_x, drone.target_y
        )
        
        # Check if we can reach the target in this step
        step_distance = min(distance_to_target, drone.speed)
        
        # If we can't move, return early
        if step_distance == 0:
            return
            
        # Calculate movement vector
        move_ratio = step_distance / distance_to_target
        dx = move_ratio * (drone.target_x - drone.x)
        dy = move_ratio * (drone.target_y - drone.y)
        
        # Move drone
        drone.x += dx
        drone.y += dy
        
        # Consume battery
        drone.consume_battery(step_distance)
        
        # Update time until idle
        drone.time_until_idle = max(drone.time_until_idle - 1, 0)
        
        # Check if we've arrived at the target
        if abs(drone.x - drone.target_x) < 1e-7 and abs(drone.y - drone.target_y) < 1e-7:
            drone.on_arrival()


class ToChargingStationState(DroneState):
    """Drone is moving to charging station"""
    
    def execute(self, drone: 'Drone') -> None:
        """Move toward charging station"""
        # Calculate distance to charging station
        distance = calculate_distance(
            drone.x, drone.y, 
            Config.CHARGING_STATION_X, Config.CHARGING_STATION_Y
        )
        
        #