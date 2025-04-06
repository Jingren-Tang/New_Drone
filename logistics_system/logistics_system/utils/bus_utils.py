import numpy as np
from scipy.spatial.distance import cdist

def find_nearest_bus_station(x, y, bus_stations, line):
    """
    Find the nearest bus station on a specific line to a given location.
    
    Args:
        x (float): X-coordinate of the location
        y (float): Y-coordinate of the location
        bus_stations (list): List of bus station objects
        line (str): Bus line name
        
    Returns:
        BusStation: The nearest bus station object
        
    Raises:
        ValueError: If no bus stations are found for the line
    """
    # Filter for stations on the specified line
    filtered_stations = [bs for bs in bus_stations if bs.line == line]
    if not filtered_stations:
        raise ValueError(f"No bus stations found for line {line}.")

    # Use cdist to calculate distances and find the nearest station
    distances = cdist([(x, y)], [(bs.x, bs.y) for bs in filtered_stations])
    nearest_index = np.argmin(distances)
    return filtered_stations[nearest_index]

def find_nearest_bus_stations(x, y, bus_stations, line, n=3):
    """
    Find multiple nearest bus stations on a specific line to a given location.
    
    Args:
        x (float): X-coordinate of the location
        y (float): Y-coordinate of the location
        bus_stations (list): List of bus station objects
        line (str): Bus line name
        n (int, optional): Number of stations to return. Defaults to 3.
        
    Returns:
        list: List of nearest bus station objects sorted by distance
        
    Raises:
        ValueError: If no bus stations are found for the line
    """
    # Filter for stations on the specified line
    filtered_stations = [bs for bs in bus_stations if bs.line == line]
    if not filtered_stations:
        raise ValueError(f"No bus stations found for line {line}.")

    # Calculate distance for each station
    stations_with_distances = []
    for station in filtered_stations:
        distance = np.sqrt((station.x - x) ** 2 + (station.y - y) ** 2)
        stations_with_distances.append((station, distance))

    # Sort by distance
    sorted_stations = sorted(stations_with_distances, key=lambda x: x[1])
    
    # Return the nearest n stations (or fewer if there aren't n stations)
    return [station for station, _ in sorted_stations[:n]]

def calculate_best_line(pickup_x, pickup_y, delivery_x, delivery_y, bus_stations, bus_lines):
    """
    Calculate the best bus line for a delivery task.
    
    Args:
        pickup_x (float): X-coordinate of pickup point
        pickup_y (float): Y-coordinate of pickup point
        delivery_x (float): X-coordinate of delivery point
        delivery_y (float): Y-coordinate of delivery point
        bus_stations (list): List of bus station objects
        bus_lines (list): List of bus line names
        
    Returns:
        tuple: (best_line, min_total_distance) - The best line and the minimum total distance
    """
    def find_station(x, y, stations, line):
        # Filter for stations on the current line
        stations_in_line = [station for station in stations if station.line == line]
        if not stations_in_line:
            return None, float('inf')

        # Find the nearest station
        min_distance = float('inf')
        nearest_station = None
        for station in stations_in_line:
            distance = ((station.x - x) ** 2 + (station.y - y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_station = station
        return nearest_station, min_distance

    best_line = None
    min_total_distance = float('inf')

    for line in bus_lines:
        # Find the nearest station and distance for the pickup point
        pickup_station, pickup_distance = find_station(pickup_x, pickup_y, bus_stations, line)
        # Find the nearest station and distance for the delivery point
        delivery_station, delivery_distance = find_station(delivery_x, delivery_y, bus_stations, line)

        # Skip this line if no matching stations were found
        if pickup_station is None or delivery_station is None:
            continue

        # Calculate the total distance
        total_distance = pickup_distance + delivery_distance

        # Update the best line if this one is better
        if total_distance < min_total_distance:
            min_total_distance = total_distance
            best_line = line

    return best_line, min_total_distance

def is_bus_on_line(bus, line):
    """
    Check if a bus is on a specific line.
    
    Args:
        bus: Bus object
        line (str): Bus line name
        
    Returns:
        bool: True if the bus is on the line, False otherwise
    """
    return any(station['line'] == line for station in bus.route)