"""
Bus system implementation for managing buses
"""
import math
from typing import List, Dict, Any, Optional, Tuple, Callable
import pandas as pd

from ..entities.bus import Bus
from ..entities.bus_station import BusStation
from ..utils.events import EventEmitter, global_event_emitter
from ..utils.distance import calculate_distance
from ..config.settings import Config


class BusSystem:
    """
    System for managing all buses in the simulation
    """
    
    def __init__(self, event_emitter: Optional[EventEmitter] = None):
        """
        Initialize bus system
        
        Args:
            event_emitter: Optional event emitter for publishing events
        """
        self.buses: List[Bus] = []
        self.bus_id_counter = 0
        self.time = 0
        self.tasks_transported = 0
        self.event_emitter = event_emitter or global_event_emitter
    
    def initialize_buses(self, bus_lines: List[str], bus_stations_df: pd.DataFrame) -> None:
        """
        Initialize buses for each line
        
        Args:
            bus_lines: List of bus line names
            bus_stations_df: DataFrame with bus station data
        """
        for line in bus_lines:
            # Get and sort route stations for this line
            route_stations = sorted(
                [station for station in bus_stations_df.to_dict('records') 
                 if station['line'] == line],
                key=lambda s: s['station_index']
            )
            
            # Skip if not enough stations
            if len(route_stations) <= 1:
                continue
                
            # Create a bus starting at first station, moving forward
            self.buses.append(Bus(
                entity_id=self.bus_id_counter,
                line=line,
                route=route_stations,
                start_station_index=0,
                direction=1,
                event_emitter=self.event_emitter
            ))
            self.bus_id_counter += 1
            
            # Create a bus starting at last station, moving backward
            self.buses.append(Bus(
                entity_id=self.bus_id_counter,
                line=line,
                route=route_stations,
                start_station_index=len(route_stations) - 1,
                direction=-1,
                event_emitter=self.event_emitter
            ))
            self.bus_id_counter += 1
            
            # Emit event
            if self.event_emitter:
                self.event_emitter.emit('buses_initialized', {
                    'line': line,
                    'route_stations_count': len(route_stations),
                    'buses_created': 2  # Forward and backward buses
                })
    
    def update_buses(self) -> None:
        """Update all buses' positions"""
        for bus in self.buses:
            bus.move()
    
    def simulate(self, bus_lines: List[str], bus_stations_df: pd.DataFrame) -> None:
        """
        Simulate bus system for one time step
        
        Args:
            bus_lines: List of bus line names
            bus_stations_df: DataFrame with bus station data
        """
        # Initialize buses periodically
        if self.time % 10 == 0 and self.time <= 50:
            self.initialize_buses(bus_lines, bus_stations_df)
        
        # Update all buses
        self.update_buses()
        self.time += 1
    
    def check_buses_at_station_for_pickup(self, x: float, y: float, 
                                         line: Optional[str] = None, 
                                         delivery_station: Optional[Tuple[float, float]] = None) -> Optional[Bus]:
        """
        Find a bus at the specified location for package pickup
        
        Args:
            x: X coordinate of station
            y: Y coordinate of station
            line: Optional line filter
            delivery_station: Optional delivery station coordinates for direction check
            
        Returns:
            Bus or None: Matching bus or None if not found
        """
        for bus in self.buses:
            # Check if bus is at the specified location
            if bus.x == x and bus.y == y:
                # Check if line filter is specified and bus matches
                if line is not None and not bus.is_on_target_line(line):
                    continue
                
                # Check if delivery station is specified and bus direction is correct
                if delivery_station is not None and not bus.check_direction(delivery_station):
                    continue
                
                # Return matching bus
                return bus
        
        # No matching bus found
        return None
    
    def check_buses_at_station_for_delivery(self, x: float, y: float, 
                                           bus_id: int, 
                                           tolerance: float = 0.00001) -> Optional[Bus]:
        """
        Find a specific bus at the specified location for package delivery
        
        Args:
            x: X coordinate of station
            y: Y coordinate of station
            bus_id: ID of bus to find
            tolerance: Distance tolerance for matching location
            
        Returns:
            Bus or None: Matching bus or None if not found
        """
        for bus in self.buses:
            # Check if bus ID matches and bus is at the specified location
            distance = calculate_distance(bus.x, bus.y, x, y)
            if distance <= tolerance and bus.entity_id == bus_id:
                return bus
        
        # No matching bus found
        return None
    
    def get_buses_by_line(self, line: str) -> List[Bus]:
        """
        Get all buses on a specific line
        
        Args:
            line: Line name
            
        Returns:
            List[Bus]: Buses on the specified line
        """
        return [bus for bus in self.buses if bus.line == line]
    
    def get_bus_by_id(self, bus_id: int) -> Optional[Bus]:
        """
        Get bus by ID
        
        Args:
            bus_id: Bus ID
            
        Returns:
            Bus or None: Bus with matching ID or None if not found
        """
        for bus in self.buses:
            if bus.entity_id == bus_id:
                return bus
        return None
    
    def get_buses_at_station(self, station_x: float, station_y: float, 
                            tolerance: float = 0.00001) -> List[Bus]:
        """
        Get all buses at a specific station
        
        Args:
            station_x: X coordinate of station
            station_y: Y coordinate of station
            tolerance: Distance tolerance for matching location
            
        Returns:
            List[Bus]: Buses at the specified station
        """
        at_station = []
        for bus in self.buses:
            distance = calculate_distance(bus.x, bus.y, station_x, station_y)
            if distance <= tolerance:
                at_station.append(bus)
        return at_station
    
    def for_each_bus(self, callback: Callable[[Bus], None]) -> None:
        """
        Apply callback function to each bus
        
        Args:
            callback: Function to apply to each bus
        """
        for bus in self.buses:
            callback(bus)
    
    def get_state(self) -> Dict[str, Any]:
        """Get system state as dictionary for logging/recording"""
        return {
            "time": self.time,
            "bus_count": len(self.buses),
            "tasks_transported": self.tasks_transported
        }