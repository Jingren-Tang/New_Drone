"""
Base Entity class for all movable objects in the simulation
"""
from abc import ABC, abstractmethod
from typing import Optional, Tuple, Dict, Any


class Entity(ABC):
    """Base class for all moving entities in the simulation"""
    
    def __init__(self, entity_id: int, x: float, y: float, speed: float):
        """
        Initialize an entity
        
        Args:
            entity_id: Unique identifier
            x: Initial x coordinate
            y: Initial y coordinate
            speed: Movement speed per time unit
        """
        self.entity_id = entity_id
        self.x = x
        self.y = y
        self.speed = speed
        self.status = 'idle'
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
    
    @abstractmethod
    def move(self) -> None:
        """Move the entity according to its logic"""
        pass
    
    def set_target(self, x: float, y: float) -> None:
        """Set target coordinates for movement"""
        self.target_x = x
        self.target_y = y
    
    def get_position(self) -> Tuple[float, float]:
        """Get current position"""
        return (self.x, self.y)
    
    def get_state(self) -> Dict[str, Any]:
        """Get entity state as dictionary for logging/recording"""
        return {
            "id": self.entity_id,
            "x": self.x,
            "y": self.y,
            "status": self.status
        }