"""
Bus station entity representing fixed locations in the transportation network
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class BusStation:
    """Bus station with coordinates and line information"""
    x: float
    y: float 
    line: str
    station_index: Optional[int] = None
    
    def get_position(self):
        """Return the position as a tuple"""
        return (self.x, self.y)
    
    def __str__(self):
        """String representation of the bus station"""
        return f"BusStation(line={self.line}, pos=({self.x}, {self.y}), idx={self.station_index})"
    
    def to_dict(self):
        """Convert to dictionary for serialization"""
        return {
            "x": self.x,
            "y": self.y,
            "line": self.line,
            "station_index": self.station_index
        }