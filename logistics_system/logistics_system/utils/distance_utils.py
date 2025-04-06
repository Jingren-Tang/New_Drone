import math
from utils.bus_utils import find_nearest_bus_station

def calculate_transport_distance(pickup_x, pickup_y, delivery_x, delivery_y, bus_stations, bus_lines):
    """
    Calculate the total distance for both bus transport and direct drone transport.
    
    Args:
        pickup_x (float): X-coordinate of pickup point
        pickup_y (float): Y-coordinate of pickup point
        delivery_x (float): X-coordinate of delivery point
        delivery_y (float): Y-coordinate of delivery point
        bus_stations (list): List of bus station objects
        bus_lines (list): List of bus line names
        
    Returns:
        tuple: (bus_distance, drone_distance) - The bus transport distance and direct drone distance
    """
    # Direct drone transport distance
    drone_distance = math.sqrt((pickup_x - delivery_x)**2 + (pickup_y - delivery_y)**2)

    # Calculate bus transport distance
    bus_distance = float('inf')
    for line in bus_lines:
        # Calculate distance from pickup point to nearest pickup station
        pickup_station = find_nearest_bus_station(pickup_x, pickup_y, bus_stations, line)
        if pickup_station:
            pickup_distance = math.sqrt((pickup_x - pickup_station.x)**2 + (pickup_y - pickup_station.y)**2)
        else:
            pickup_distance = float('inf')

        # Calculate distance from nearest delivery station to delivery point
        delivery_station = find_nearest_bus_station(delivery_x, delivery_y, bus_stations, line)
        if delivery_station:
            delivery_distance = math.sqrt((delivery_x - delivery_station.x)**2 + (delivery_y - delivery_station.y)**2)
        else:
            delivery_distance = float('inf')

        # Combined distance: pickup point -> pickup station + delivery station -> delivery point
        # Note: This doesn't include the distance traveled by the bus between stations
        if pickup_station and delivery_station:
            current_bus_distance = pickup_distance + delivery_distance
            bus_distance = min(bus_distance, current_bus_distance)

    return bus_distance, drone_distance

def calculate_bus_travel_time(bus, target_x, target_y):
    """
    Calculate the travel time for a bus to reach a target location.
    
    Args:
        bus: Bus object
        target_x (float): X-coordinate of target location
        target_y (float): Y-coordinate of target location
        
    Returns:
        float or None: The travel time, or None if the target is not reachable
    """
    route = bus.route
    speed = bus.speed
    stop_time = bus.stop_time
    direction = bus.direction
    current_index = bus.station_index

    # Find the target station index in the route
    final_index = None
    for i, station in enumerate(route):
        if station['x'] == target_x and station['y'] == target_y:
            final_index = i
            break
    
    # If target station not found, return None
    if final_index is None:
        return None

    # Accumulate travel time
    total_time = 0.0

    # Simulate the bus moving station by station
    while current_index != final_index:
        # Get next station index
        next_index = current_index + direction
        
        # If out of bounds, reverse direction
        if next_index < 0 or next_index >= len(route):
            direction *= -1
            next_index = current_index + direction
        
        # Calculate distance and travel time between stations
        dist = math.sqrt(
            (route[next_index]['x'] - route[current_index]['x'])**2 + 
            (route[next_index]['y'] - route[current_index]['y'])**2
        )
        travel_time = dist / speed
        total_time += travel_time

        # Arrived at next station
        current_index = next_index
        # Add stop time if not the final station
        if current_index != final_index:
            total_time += stop_time
    
    return total_time