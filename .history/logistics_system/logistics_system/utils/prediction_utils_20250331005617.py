import math
from utils.bus_utils import find_nearest_bus_station, find_nearest_bus_stations

def predict_single_bus_arrival(bus, target_station, current_time):
    """
    Predict when a specific bus will arrive at a target station.
    
    Args:
        bus: Bus object
        target_station (tuple): Target station coordinates (x, y)
        current_time (float): Current simulation time
        
    Returns:
        float or None: Predicted arrival time, or None if the bus won't reach the target
    """
    # First check if the bus route passes through the target station
    target_station_index = None
    for i, station in enumerate(bus.route):
        if (station['x'], station['y']) == target_station:
            target_station_index = i
            break
    
    if target_station_index is None:
        return None  # Target station not on bus route
    
    # Check if the bus is already at the target station
    if (bus.x, bus.y) == target_station and bus.status == 'at_station':
        return current_time  # Bus is already at the target station
    
    # Calculate how long it will take to reach the target
    time_needed = 0
    current_index = bus.station_index
    
    # If the bus is moving, calculate time to reach the next station
    if bus.status == 'moving':
        next_station_index = current_index + bus.direction
        if next_station_index < 0 or next_station_index >= len(bus.route):
            # If next station is terminus, bus will reverse direction
            bus_direction = -bus.direction
            next_station_index = current_index + bus_direction
        else:
            bus_direction = bus.direction
            
        next_station = bus.route[next_station_index]
        
        # Calculate remaining distance to next station
        total_distance = math.sqrt(
            (next_station['x'] - bus.route[current_index]['x'])**2 + 
            (next_station['y'] - bus.route[current_index]['y'])**2
        )
        
        traveled_distance = math.sqrt(
            (bus.x - bus.route[current_index]['x'])**2 + 
            (bus.y - bus.route[current_index]['y'])**2
        )
        
        remaining_distance = total_distance - traveled_distance
        
        # Calculate time to reach next station
        time_to_next_station = remaining_distance / bus.speed
        time_needed += time_to_next_station
        
        # Update current index to next station
        current_index = next_station_index
    else:
        # If bus is at a station, calculate remaining stop time
        if bus.time_at_station < bus.stop_time:
            time_needed += (bus.stop_time - bus.time_at_station)
        
        # Use bus's current direction
        bus_direction = bus.direction
    
    # Simulate bus travel along route to target station
    while current_index != target_station_index:
        # Calculate direction to next station
        next_index = current_index + bus_direction
        
        # Check if need to reverse direction
        if next_index < 0 or next_index >= len(bus.route):
            bus_direction = -bus_direction
            next_index = current_index + bus_direction
        
        # If direction reversal doesn't help, target is unreachable
        if next_index == current_index:
            return None
        
        # Calculate distance and travel time to next station
        current_station = bus.route[current_index]
        next_station = bus.route[next_index]
        
        distance = math.sqrt(
            (next_station['x'] - current_station['x'])**2 + 
            (next_station['y'] - current_station['y'])**2
        )
        
        travel_time = distance / bus.speed
        time_needed += travel_time
        
        # Add stop time if not at target station
        if next_index != target_station_index:
            time_needed += bus.stop_time
        
        # Move to next station
        current_index = next_index
    
    # Return predicted arrival time
    return current_time + time_needed

def predict_bus_arrival_times(bus_system, line, station_coords, current_time):
    """
    Predict arrival times for all buses on a specific line to a station.
    
    Args:
        bus_system: BusSystem object
        line (str): Bus line name
        station_coords (tuple): Station coordinates (x, y)
        current_time (float): Current simulation time
        
    Returns:
        list: Sorted list of dictionaries with predicted arrival times
    """
    predictions = []
    
    # Filter buses on the target line
    line_buses = [bus for bus in bus_system.buses if bus.line == line]
    
    for bus in line_buses:
        # Predict arrival time for this bus
        arrival_time = predict_single_bus_arrival(bus, station_coords, current_time)
        
        if arrival_time is not None:
            predictions.append({
                'bus_id': bus.bus_id,
                'current_location': (bus.x, bus.y),
                'current_status': bus.status,
                'direction': bus.direction,
                'predicted_arrival': arrival_time,
                'wait_time': arrival_time - current_time
            })
    
    # Sort by predicted arrival time
    predictions.sort(key=lambda x: x['predicted_arrival'])
    return predictions

def optimize_station_selection(task, bus_system, bus_stations, current_time):
    """
    Optimize station selection based on real-time bus predictions.
    
    Args:
        task (dict): Task object
        bus_system: BusSystem object
        bus_stations (list): List of bus station objects
        current_time (float): Current simulation time
        
    Returns:
        tuple: (best_pickup_rank, best_delivery_rank, analysis_results)
    """
    line = task['line']
    pickup_x, pickup_y = task['pickup_x'], task['pickup_y']
    delivery_x, delivery_y = task['delivery_x'], task['delivery_y']
    
    # Get nearby stations for pickup
    pickup_stations = find_nearest_bus_stations(pickup_x, pickup_y, bus_stations, line, 3)
    
    # Get nearby stations for delivery
    delivery_stations = find_nearest_bus_stations(delivery_x, delivery_y, bus_stations, line, 3)
    
    best_combination = None
    min_total_time = float('inf')
    results = []
    
    for pickup_rank, pickup_station in enumerate(pickup_stations):
        # Calculate drone travel time from pickup point to station
        drone_to_pickup_station_dist = math.sqrt(
            (pickup_x - pickup_station.x)**2 + (pickup_y - pickup_station.y)**2
        )
        drone_to_pickup_time = drone_to_pickup_station_dist / 5  # Assuming drone speed is 5
        
        # Predict bus arrivals at pickup station
        pickup_predictions = predict_bus_arrival_times(
            bus_system, line, (pickup_station.x, pickup_station.y), 
            current_time + drone_to_pickup_time  # Time when drone arrives at station
        )
        
        if not pickup_predictions:
            continue  # No buses will arrive at this station
        
        # Choose the fastest arriving bus
        best_pickup_bus = pickup_predictions[0]
        pickup_bus_arrival = best_pickup_bus['predicted_arrival']
        
        for delivery_rank, delivery_station in enumerate(delivery_stations):
            # Calculate drone travel time from delivery station to delivery point
            drone_from_delivery_station_dist = math.sqrt(
                (delivery_x - delivery_station.x)**2 + (delivery_y - delivery_station.y)**2
            )
            drone_from_delivery_time = drone_from_delivery_station_dist / 5
            
            # Predict when the same bus will arrive at delivery station
            bus_id = best_pickup_bus['bus_id']
            bus = next((b for b in bus_system.buses if b.bus_id == bus_id), None)
            
            if bus:
                # Predict when bus will arrive at delivery station after pickup
                delivery_arrival = predict_single_bus_arrival(
                    bus, 
                    (delivery_station.x, delivery_station.y), 
                    pickup_bus_arrival
                )
                
                if delivery_arrival is not None:
                    # Total time = drone to pickup station + bus transport + drone from delivery station
                    total_time = (drone_to_pickup_time + 
                                 (delivery_arrival - pickup_bus_arrival) + 
                                 drone_from_delivery_time)
                    
                    combination = {
                        'pickup_station_rank': pickup_rank,
                        'pickup_station': (pickup_station.x, pickup_station.y),
                        'pickup_drone_time': drone_to_pickup_time,
                        'bus_id': bus_id,
                        'bus_pickup_time': pickup_bus_arrival - current_time,
                        'delivery_station_rank': delivery_rank,
                        'delivery_station': (delivery_station.x, delivery_station.y),
                        'bus_travel_time': delivery_arrival - pickup_bus_arrival,
                        'delivery_drone_time': drone_from_delivery_time,
                        'total_time': total_time
                    }
                    
                    results.append(combination)
                    
                    if total_time < min_total_time:
                        min_total_time = total_time
                        best_combination = combination
    
    # Sort results by total time
    results.sort(key=lambda x: x['total_time'])
    
    if best_combination:
        return (
            best_combination['pickup_station_rank'],
            best_combination['delivery_station_rank'],
            {
                'task_id': task['index'],
                'line': line,
                'pickup_point': (pickup_x, pickup_y),
                'delivery_point': (delivery_x, delivery_y),
                'best_combination': best_combination,
                'all_combinations': results[:5]  # Return top 5 combinations
            }
        )
    else:
        return 0, 0, {'error': 'No valid combinations found'}