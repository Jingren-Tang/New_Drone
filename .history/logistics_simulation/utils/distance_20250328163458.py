"""
Utility functions for distance calculations and position operations
"""
import math
import numpy as np
from typing import Tuple, List, Any, Union
from ..entities.bus_station import BusStation


def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Calculate Euclidean distance between two points
    
    Args:
        x1, y1: Coordinates of first point
        x2, y2: Coordinates of second point
        
    Returns:
        float: Euclidean distance
    """
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def calculate_distance_between_positions(pos1: Tuple[float, float], 
                                        pos2: Tuple[float, float]) -> float:
    """
    Calculate distance between two position tuples
    
    Args:
        pos1: (x, y) of first position
        pos2: (x, y) of second position
        
    Returns:
        float: Euclidean distance
    """
    return calculate_distance(pos1[0], pos1[1], pos2[0], pos2[1])


def find_nearest_bus_station(x: float, y: float, 
                            bus_stations: List[BusStation], 
                            line: str) -> BusStation:
    """
    Find the nearest bus station on a specific line
    
    Args:
        x, y: Current coordinates
        bus_stations: List of all bus stations
        line: Target line name
        
    Returns:
        BusStation: Nearest bus station on the specified line
    """
    # Filter stations on the specified line
    filtered_stations = [bs for bs in bus_stations if bs.line == line]
    if not filtered_stations:
        raise ValueError(f"No bus stations found for line {line}.")

    # Find the nearest station
    nearest_station = None
    min_distance = float('inf')
    
    for station in filtered_stations:
        dist = calculate_distance(x, y, station.x, station.y)
        if dist < min_distance:
            min_distance = dist
            nearest_station = station
            
    return nearest_station


def find_nearest_bus_stations(x: float, y: float, 
                             bus_stations: List[BusStation], 
                             line: str, n: int = 3) -> List[BusStation]:
    """
    Find multiple nearest bus stations on a specific line
    
    Args:
        x, y: Current coordinates
        bus_stations: List of all bus stations
        line: Target line name
        n: Number of stations to return
        
    Returns:
        List[BusStation]: List of nearest bus stations on the specified line
    """
    # Filter stations on the specified line
    filtered_stations = [bs for bs in bus_stations if bs.line == line]
    if not filtered_stations:
        raise ValueError(f"No bus stations found for line {line}.")

    # Calculate distance to each station
    stations_with_distances = []
    for station in filtered_stations:
        dist = calculate_distance(x, y, station.x, station.y)
        stations_with_distances.append((station, dist))
    
    # Sort by distance
    sorted_stations = sorted(stations_with_distances, key=lambda x: x[1])
    
    # Return the nearest n stations
    return [station for station, _ in sorted_stations[:n]]


def calculate_best_line(pickup_x: float, pickup_y: float, 
                       delivery_x: float, delivery_y: float, 
                       bus_stations: List[BusStation], 
                       bus_lines: List[str]) -> Tuple[str, float]:
    """
    Find the best bus line for a journey between two points
    
    Args:
        pickup_x, pickup_y: Pickup coordinates
        delivery_x, delivery_y: Delivery coordinates
        bus_stations: List of all bus stations
        bus_lines: List of available bus lines
        
    Returns:
        Tuple[str, float]: Best line and total distance
    """
    def find_station(x, y, stations, line):
        # Filter stations on the specified line
        stations_in_line = [station for station in stations if station.line == line]
        if not stations_in_line:
            return None, float('inf')

        # Find the nearest station
        min_distance = float('inf')
        nearest_station = None
        for station in stations_in_line:
            distance = calculate_distance(x, y, station.x, station.y)
            if distance < min_distance:
                min_distance = distance
                nearest_station = station
                
        return nearest_station, min_distance

    best_line = None
    min_total_distance = float('inf')

    for line in bus_lines:
        # Find nearest pickup station on this line
        pickup_station, pickup_distance = find_station(pickup_x, pickup_y, bus_stations, line)
        # Find nearest delivery station on this line
        delivery_station, delivery_distance = find_station(delivery_x, delivery_y, bus_stations, line)

        # Skip this line if no suitable stations found
        if pickup_station is None or delivery_station is None:
            continue

        # Calculate total distance
        total_distance = pickup_distance + delivery_distance

        # Update best line if this is better
        if total_distance < min_total_distance:
            min_total_distance = total_distance
            best_line = line

    return best_line, min_total_distance


def calculate_transport_distance(pickup_x: float, pickup_y: float, 
                                delivery_x: float, delivery_y: float, 
                                bus_stations: List[BusStation], 
                                bus_lines: List[str]) -> Tuple[float, float]:
    """
    Calculate distances for bus transport vs direct drone transport
    
    Args:
        pickup_x, pickup_y: Pickup coordinates
        delivery_x, delivery_y: Delivery coordinates
        bus_stations: List of all bus stations
        bus_lines: List of available bus lines
        
    Returns:
        Tuple[float, float]: (bus_distance, drone_distance)
    """
    # Direct drone distance (straight line)
    drone_distance = calculate_distance(pickup_x, pickup_y, delivery_x, delivery_y)

    # Calculate bus transport distance
    bus_distance = float('inf')
    for line in bus_lines:
        # Find nearest pickup station
        pickup_station = find_nearest_bus_station(pickup_x, pickup_y, bus_stations, line)
        if pickup_station:
            pickup_distance = calculate_distance(pickup_x, pickup_y, pickup_station.x, pickup_station.y)
        else:
            pickup_distance = float('inf')

        # Find nearest delivery station
        delivery_station = find_nearest_bus_station(delivery_x, delivery_y, bus_stations, line)
        if delivery_station:
            delivery_distance = calculate_distance(delivery_x, delivery_y, delivery_station.x, delivery_station.y)
        else:
            delivery_distance = float('inf')

        # Calculate total bus distance
        if pickup_station and delivery_station:
            current_bus_distance = pickup_distance + delivery_distance
            bus_distance = min(bus_distance, current_bus_distance)

    return bus_distance, drone_distance