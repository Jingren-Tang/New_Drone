import math
import numpy as np
import pandas as pd
from collections import deque

# Import models
from models.bus import Bus, BusSystem
from models.bus_station import BusStation
from models.drone import Drone, DroneFleet
from models.package import Package, get_package_by_id, get_task_by_id
from models.task_optimizer import MultiTaskOptimizer

# Import utilities
from utils.bus_utils import find_nearest_bus_station, find_nearest_bus_stations, calculate_best_line
from utils.distance_utils import calculate_transport_distance
from utils.prediction_utils import (
    predict_bus_arrival_times, 
    predict_single_bus_arrival, 
    optimize_station_selection
)

def run_simulation(task_list, bus_stations_df, total_time=5000):
    """
    Run the logistics simulation.
    
    Args:
        task_list (list): List of task dictionaries
        bus_stations_df (DataFrame): DataFrame of bus stations
        total_time (int, optional): Total simulation time. Defaults to 5000.
        
    Returns:
        tuple: (simulation_records, bus_records, package_records) - Simulation data for analysis
    """
    # Set simulation parameters
    tolerance = 10
    station_rank = 1
    
    # Create bus stations
    bus_stations = [
        BusStation(row['x'], row['y'], row['line'])
        for index, row in bus_stations_df.iterrows()
    ]
    
    # Initialize simulation
    drone_fleet = DroneFleet(initial_drone_count=5)
    bus_lines = ['Line 1', 'Line 2']
    
    bus_system = BusSystem()
    bus_system.simulate(bus_lines, bus_stations_df)
    
    # Initialize task status and packages
    tasks = task_list
    packages = []
    
    for task in tasks:
        task['pickup_status'] = 'unassigned'
        task['delivery_status'] = 'unassigned'
        # Use task index as package ID
        package = Package(task['index'], task['index'])
        packages.append(package)
        task['package_id'] = task['index']
    
    # Calculate bus station mapping
    station_mapping = {(station.x, station.y): idx for idx, station in enumerate(bus_stations)}
    
    # Create multi-task optimizer
    task_optimizer = MultiTaskOptimizer(drone_fleet, bus_system, bus_stations, bus_lines)
    
    # Initialize simulation records
    simulation_records = []
    bus_records = []
    package_records = []
    
    # Start simulation
    current_time = 0
    task_index = 0
    
    # Main simulation loop
    while current_time <= total_time:
        # Process task queues
        drone_fleet.process_delivery_task_queue()
        drone_fleet.process_direct_task_queue()
        drone_fleet.process_pickup_task_queue()
        
        # Check for new tasks
        new_tasks_found = False
        while task_index < len(tasks) and tasks[task_index]['pickup_time'] <= current_time:
            task = tasks[task_index]
            if task['pickup_status'] == 'unassigned':
                # Add task to optimizer
                task_optimizer.add_task(task)
                new_tasks_found = True
            
            task_index += 1
        
        # If new tasks found, run optimization
        if new_tasks_found:
            task_optimizer.process_tasks(current_time)
        
        # Process completed pickup tasks that need delivery
        for task in tasks:
            if (
                task['pickup_status'] == 'completed'
                and task['delivery_status'] == 'unassigned'
            ):
                # Check if delivery assignment time has been calculated
                if 'delivery_assign_time' in task:
                    # If time to assign delivery, do so
                    if current_time >= task['delivery_assign_time']:
                        # Use real-time bus prediction to choose best delivery station
                        _, delivery_rank, _ = optimize_station_selection(
                            task, bus_system, bus_stations, current_time
                        )
                        
                        success_delivery = drone_fleet.try_allocate_delivery_task(
                            task, 
                            task['delivery_x'], 
                            task['delivery_y'], 
                            auto_create_drone=False
                        )
                        
                        if not success_delivery:
                            drone_fleet.add_delivery_task_to_queue(
                                task, task['delivery_x'], task['delivery_y']
                            )
                else:
                    # If no delivery_assign_time, predict bus arrival
                    
                    # Find bus transporting the package
                    package = get_package_by_id(task['package_id'], packages)
                    if package and package.status == 'on_bus' and package.bus_id is not None:
                        bus = next((b for b in bus_system.buses if b.bus_id == package.bus_id), None)
                        
                        if bus:
                            # Find nearest delivery stations
                            delivery_stations = find_nearest_bus_stations(
                                task['delivery_x'], task['delivery_y'],
                                bus_stations, task['line'], 3
                            )
                            
                            if delivery_stations:
                                # Predict when bus will arrive at delivery stations
                                best_station = None
                                best_arrival_time = float('inf')
                                
                                for station in delivery_stations:
                                    arrival_time = predict_single_bus_arrival(
                                        bus, (station.x, station.y), current_time
                                    )
                                    
                                    if arrival_time is not None and arrival_time < best_arrival_time:
                                        best_arrival_time = arrival_time
                                        best_station = station
                                
                                if best_station:
                                    # Set delivery task time
                                    task['delivery_assign_time'] = best_arrival_time
                                    task['selected_delivery_station'] = (best_station.x, best_station.y)
                                    print(f"Task {task['index']}: Scheduled delivery at time {best_arrival_time}")
        
        # Move drones and buses
        drone_fleet.move_all_drones()
        bus_system.simulate(bus_lines, bus_stations_df)
        
        # Handle drone-bus interactions
        for drone in drone_fleet.drones:
            # Handle drones waiting for pickup bus
            if (
                drone.status == 'waiting_at_bus_station_for_pickup_bus' and 
                drone.current_task is not None and 
                drone.current_task['pickup_status'] == 'waiting_for_bus'
            ):
                # Use real-time bus prediction to find fastest arriving bus
                current_line = drone.current_task['line']
                
                # Get nearest delivery station
                nearest_delivery_station = find_nearest_bus_station(
                    drone.current_task['delivery_x'], 
                    drone.current_task['delivery_y'], 
                    bus_stations, 
                    current_line
                )
                
                # Get bus arrival predictions
                bus_predictions = predict_bus_arrival_times(
                    bus_system, current_line, (drone.x, drone.y), current_time
                )
                
                # Check if bus is already at station
                bus = bus_system.check_buses_at_station_for_pickup(
                    drone.x, drone.y, 
                    line=current_line, 
                    delivery_station=(nearest_delivery_station.x, nearest_delivery_station.y)
                )
                
                if bus and bus.can_pickup_package():
                    # If bus is at station and can take package
                    package = get_package_by_id(drone.current_task['package_id'], packages)
                    if package:
                        bus.load_package(package)
                        package.bus_id = bus.bus_id
                        package.current_drone_id = None
                        package.update_status('on_bus')
                        drone.current_task['pickup_status'] = 'completed'
                        drone.current_task = None
                        drone.status = 'idle'
                        drone.pickup_tasks_completed += 1
                        
                        # Predict package arrival at delivery station for scheduling
                        if package.task_id is not None:
                            task = get_task_by_id(package.task_id, tasks)
                            if task:
                                # Find nearest delivery station
                                delivery_station = find_nearest_bus_station(
                                    task['delivery_x'], task['delivery_y'], 
                                    bus_stations, current_line
                                )
                                
                                # Predict bus arrival at delivery station
                                arrival_time = predict_single_bus_arrival(
                                    bus, (delivery_station.x, delivery_station.y), current_time
                                )
                                
                                if arrival_time is not None:
                                    # Set delivery task time
                                    task['delivery_assign_time'] = arrival_time
                                    task['selected_delivery_station'] = (delivery_station.x, delivery_station.y)
                                    print(f"Task {task['index']}: Scheduled delivery at time {arrival_time}")
                
                elif not bus and bus_predictions:
                    # If no bus at station but one coming soon, wait
                    next_bus = bus_predictions[0]
                    wait_time = next_bus['wait_time']
                    
                    if wait_time <= 5:  # If wait time is short, print info
                        print(f"Drone {drone.drone_id} waiting for bus {next_bus['bus_id']} arriving in {wait_time:.1f} time units")
            
            # Handle drones waiting for delivery bus
            elif drone.status == 'waiting_at_bus_station_for_delivery':
                package = get_package_by_id(drone.current_task['package_id'], packages)
                
                if package and package.bus_id is not None:
                    # Use real-time prediction to check for approaching bus
                    bus_predictions = predict_bus_arrival_times(
                        bus_system, drone.current_task['line'], (drone.x, drone.y), current_time
                    )
                    
                    # Check if target bus is in predictions
                    target_bus_prediction = next(
                        (pred for pred in bus_predictions if pred['bus_id'] == package.bus_id), 
                        None
                    )
                    
                    # Check if bus is at station
                    bus = bus_system.check_buses_at_station_for_delivery(drone.x, drone.y, package.bus_id)
                    
                    if bus and bus.current_package:
                        package = bus.unload_package()
                        if package:
                            package.update_status('on_delivery_drone')
                            
                            task = get_task_by_id(package.task_id, tasks)
                            if task is not None:
                                drone.assign_delivery_task_to_delivery(
                                    task, task['delivery_x'], task['delivery_y'], task['line']
                                )
                                package.current_drone_id = drone.drone_id
                            else:
                                print(f"Error: Task not found for package_id {package.package_id}")
                    
                    elif target_bus_prediction and target_bus_prediction['wait_time'] <= 5:
                        # If bus arriving soon, print wait info
                        print(f"Drone {drone.drone_id} waiting for bus {package.bus_id} arriving in {target_bus_prediction['wait_time']:.1f} time units")
        
        # Record package states
        for package in packages:
            task = get_task_by_id(package.task_id, tasks)
            if package.status == 'at_pickup_point':
                x, y = task['pickup_x'], task['pickup_y']
            elif package.status == 'on_pickup_drone':
                drone = next((d for d in drone_fleet.drones if d.drone_id == package.current_drone_id), None)
                x, y = (drone.x, drone.y) if drone else (None, None)
            elif package.status == 'on_bus':
                bus = next((b for b in bus_system.buses if b.current_package == package), None)
                x, y = (bus.x, bus.y) if bus else (None, None)
            elif package.status == 'on_delivery_drone':
                drone = next((d for d in drone_fleet.drones if d.drone_id == package.current_drone_id), None)
                x, y = (drone.x, drone.y) if drone else (None, None)
            elif package.status == 'delivered':
                x, y = task['delivery_x'], task['delivery_y']
            elif package.status == 'on_direct_drone':
                drone = next((d for d in drone_fleet.drones if d.drone_id == package.current_drone_id), None)
                x, y = (drone.x, drone.y) if drone else (None, None)
            elif package.status == 'directly_delivered':
                x, y = task['delivery_x'], task['delivery_y']
            else:
                x, y = None, None

            # Record package state
            package_records.append({
                "time": current_time,
                "package_id": package.package_id,
                "task_id": package.task_id,
                "status": package.status,
                "x": x,
                "y": y,
                "bus_id": package.bus_id,
                "drone_id": package.current_drone_id
            })
        
        # Record drone and bus states
        for drone in drone_fleet.drones:
            simulation_records.append({
                "time": current_time,
                "entity": "drone",
                "id": drone.drone_id,
                "x": drone.x,
                "y": drone.y,
                "status": drone.status,
                "battery": drone.battery_left
            })
        
        for bus in bus_system.buses:
            simulation_records.append({
                "time": current_time,
                "entity": "bus",
                "id": bus.bus_id,
                "x": bus.x,
                "y": bus.y,
                "status": bus.status,
                "direction": bus.direction,
                "has_package": bus.current_package is not None
            })
        
        # Record bus station data
        for bus in bus_system.buses:
            # Get current station number
            current_station = station_mapping.get((bus.x, bus.y), None)
            if current_station is not None:
                # Get corresponding station_index and line info
                station_index = next(
                    (station['station_index'] for station in bus.route if 
                     station['x'] == bus.x and station['y'] == bus.y),
                    None
                )
                line = bus.route[0]['line'] if bus.route else None
                
                # Record data
                bus_records.append({
                    "time": current_time,
                    "entity": "bus",
                    "id": bus.bus_id,
                    "station": current_station,
                    "station_index": station_index,
                    "line": line,
                    "has_package": bus.current_package is not None
                })
        
        # Print status every 50 time units
        if current_time % 50 == 0:
            idle_drones = sum(1 for drone in drone_fleet.drones if drone.status == 'idle')
            active_drones = len(drone_fleet.drones) - idle_drones
            packages_on_drones = sum(1 for package in packages if package.status in 
                                   ['on_pickup_drone', 'on_delivery_drone', 'on_direct_drone'])
            packages_on_buses = sum(1 for package in packages if package.status == 'on_bus')
            packages_delivered = sum(1 for package in packages if package.status in 
                                    ['delivered', 'directly_delivered'])
            
            print(f"\nTime {current_time} - System Status:")
            print(f"Active drones: {active_drones}, Idle drones: {idle_drones}")
            print(f"Packages on drones: {packages_on_drones}, On buses: {packages_on_buses}, Delivered: {packages_delivered}")
            print(f"Remaining tasks in optimizer: {len(task_optimizer.pending_tasks)}")
            
            # Print drone battery stats
            low_battery_drones = sum(1 for drone in drone_fleet.drones if drone.battery_left < 50)
            if low_battery_drones > 0:
                print(f"Warning: {low_battery_drones} drones have low battery (<50)")
        
        current_time += 1
    
    # Print final summary
    print("\nSimulation complete. Task and drone summary:")
    for drone in drone_fleet.drones:
        print(f"Drone {drone.drone_id}: Completed {drone.delivery_tasks_completed} delivery tasks and {drone.pickup_tasks_completed} pickup tasks.")
    
    # Calculate completion rate and average delivery time
    completed_tasks = sum(1 for task in tasks if task['delivery_status'] == 'completed')
    completion_rate = completed_tasks / len(tasks) * 100 if tasks else 0
    
    delivery_times = []
    for package in packages:
        if package.status in ['delivered', 'directly_delivered']:
            # Find first and last package records to calculate delivery time
            first_record = next((r for r in package_records if r['package_id'] == package.package_id), None)
            last_record = next((r for r in reversed(package_records) if r['package_id'] == package.package_id 
                               and r['status'] in ['delivered', 'directly_delivered']), None)
            
            if first_record and last_record:
                delivery_time = last_record['time'] - first_record['time']
                delivery_times.append(delivery_time)
    
    avg_delivery_time = sum(delivery_times) / len(delivery_times) if delivery_times else 0
    
    print(f"\nOverall Performance:")
    print(f"Task completion rate: {completion_rate:.2f}%")
    print(f"Average delivery time: {avg_delivery_time:.2f} time units")
    print(f"Total drone count: {len(drone_fleet.drones)}")
    
    # Return simulation data
    return simulation_records, bus_records, package_records

if __name__ == "__main__":
    # Load task data and bus station data
    # This is a placeholder - in a real application, you would load this data from files
    task_list = []  # Load your task list here
    bus_stations_df = pd.DataFrame()  # Load your bus stations data here
    
    # Run simulation
    simulation_records, bus_records, package_records = run_simulation(task_list, bus_stations_df)
    
    # Convert records to DataFrames for analysis
    simulation_df = pd.DataFrame(simulation_records)
    bus_simulation_df = pd.DataFrame(bus_records)
    package_simulation_df = pd.DataFrame(package_records)
    
    print("Simulation data saved to DataFrames for analysis")