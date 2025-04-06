"""
Utility functions for bus prediction and route optimization
"""
import math
from typing import List, Dict, Any, Optional, Tuple

from ..utils.distance import calculate_distance


def calculate_bus_travel_time(bus, target_x: float, target_y: float) -> Optional[float]:
    """
    Calculate time for bus to travel to target location
    
    Args:
        bus: Bus object
        target_x: Target x coordinate
        target_y: Target y coordinate
        
    Returns:
        float or None: Travel time or None if target not on route
    """
    route = bus.route
    speed = bus.speed
    stop_time = bus.stop_time
    direction = bus.direction
    current_index = bus.station_index
    
    # Find target station in route
    final_index = None
    for i, station in enumerate(route):
        if station['x'] == target_x and station['y'] == target_y:
            final_index = i
            break
    
    # Return None if target not on route
    if final_index is None:
        return None
    
    # Calculate travel time
    total_time = 0.0
    
    # If bus is moving, add time to reach next station
    if bus.status == 'moving':
        # Find next station
        next_index = current_index + direction
        
        # Check if next station is valid
        if next_index < 0 or next_index >= len(route):
            direction *= -1
            next_index = current_index + direction
        
        # Calculate distance to next station
        next_station = route[next_index]
        dist = calculate_distance(
            bus.x, bus.y, next_station['x'], next_station['y']
        )
        
        # Add travel time to next station
        travel_time = dist / speed
        total_time += travel_time
        
        # Update current index to next station
        current_index = next_index
    else:
        # If bus is at station, add remaining wait time
        if bus.time_at_station < bus.stop_time:
            total_time += (bus.stop_time - bus.time_at_station)
    
    # Simulate bus moving through route
    while current_index != final_index:
        # Calculate next station index
        next_index = current_index + direction
        
        # Check if next station is valid
        if next_index < 0 or next_index >= len(route):
            direction *= -1
            next_index = current_index + direction
        
        # Calculate distance between stations
        current_station = route[current_index]
        next_station = route[next_index]
        dist = calculate_distance(
            current_station['x'], current_station['y'],
            next_station['x'], next_station['y']
        )
        
        # Add travel time
        travel_time = dist / speed
        total_time += travel_time
        
        # Add stop time if not final station
        if next_index != final_index:
            total_time += stop_time
        
        # Move to next station
        current_index = next_index
    
    return total_time


def predict_single_bus_arrival(bus, target_station: Tuple[float, float], 
                             current_time: float) -> Optional[float]:
    """
    Predict arrival time of a single bus at target station
    
    Args:
        bus: Bus object
        target_station: Target station coordinates (x, y)
        current_time: Current simulation time
        
    Returns:
        float or None: Predicted arrival time or None if unreachable
    """
    # Check if target station is on route
    target_station_index = None
    for i, station in enumerate(bus.route):
        if (station['x'], station['y']) == target_station:
            target_station_index = i
            break
    
    if target_station_index is None:
        return None  # Target station not on route
    
    # Check if bus is already at target station
    if (bus.x, bus.y) == target_station and bus.status == 'at_station':
        return current_time  # Already at target
    
    # Calculate travel time
    travel_time = calculate_bus_travel_time(bus, *target_station)
    
    if travel_time is None:
        return None  # Cannot reach target
    
    # Return predicted arrival time
    return current_time + travel_time


def predict_bus_arrival_times(bus_system, line: str, station_coords: Tuple[float, float], 
                            current_time: float) -> List[Dict[str, Any]]:
    """
    Predict arrival times of all buses at target station
    
    Args:
        bus_system: BusSystem object
        line: Target line name
        station_coords: Target station coordinates (x, y)
        current_time: Current simulation time
        
    Returns:
        List[Dict[str, Any]]: Predictions sorted by arrival time
    """
    predictions = []
    
    # Get buses on target line
    line_buses = [bus for bus in bus_system.buses if bus.line == line]
    
    for bus in line_buses:
        # Predict arrival time
        arrival_time = predict_single_bus_arrival(bus, station_coords, current_time)
        
        if arrival_time is not None:
            predictions.append({
                'bus_id': bus.entity_id,
                'current_location': (bus.x, bus.y),
                'current_status': bus.status,
                'direction': bus.direction,
                'predicted_arrival': arrival_time,
                'wait_time': arrival_time - current_time
            })
    
    # Sort by arrival time
    predictions.sort(key=lambda x: x['predicted_arrival'])
    
    return predictions