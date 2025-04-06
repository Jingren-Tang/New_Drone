"""
Main simulation controller
"""
import pandas as pd
import math
import time
from typing import List, Dict, Any, Optional, Tuple, Callable
import os

from .entities.bus_station import BusStation
from .entities.package import Package
from .systems.drone_fleet import DroneFleet
from .systems.bus_system import BusSystem
from .systems.task_optimizer import TaskOptimizer
from .utils.events import EventEmitter, global_event_emitter
from .utils.logger import SimulationLogger, simulation_logger
from .utils.distance import (
    calculate_distance, find_nearest_bus_station, 
    find_nearest_bus_stations, calculate_best_line
)
from .utils.prediction import predict_bus_arrival_times, predict_single_bus_arrival
from .config.settings import Config


class Simulation:
    """
    Main simulation controller for logistics system
    """
    
    def __init__(self, task_list: List[Dict[str, Any]], 
                bus_stations_df: pd.DataFrame,
                station_rank: int = 0,
                total_time: int = Config.SIMULATION_TIME,
                event_emitter: Optional[EventEmitter] = None,
                logger: Optional[SimulationLogger] = None):
        """
        Initialize simulation
        
        Args:
            task_list: List of delivery tasks
            bus_stations_df: DataFrame with bus station data
            station_rank: Default rank for station selection
            total_time: Total simulation time
            event_emitter: Optional event emitter for events
            logger: Optional logger for simulation data
        """
        self.task_list = task_list
        self.bus_stations_df = bus_stations_df
        self.station_rank = station_rank
        self.total_time = total_time
        self.current_time = 0
        
        # Set up event emitter
        self.event_emitter = event_emitter or global_event_emitter
        
        # Set up logger
        self.logger = logger or simulation_logger
        
        # Convert bus stations DataFrame to BusStation objects
        self.bus_stations = [
            BusStation(
                x=row['x'],
                y=row['y'],
                line=row['line'],
                station_index=row['station_index'] if 'station_index' in row else None
            )
            for index, row in bus_stations_df.iterrows()
        ]
        
        # Create bus station mapping
        self.station_mapping = {
            (station.x, station.y): idx 
            for idx, station in enumerate(self.bus_stations)
        }
        
        # Initialize systems
        self.bus_lines = Config.BUS_LINES
        self.drone_fleet = DroneFleet(Config.DRONE_INITIAL_COUNT, self.event_emitter)
        self.bus_system = BusSystem(self.event_emitter)
        
        # Create task optimizer
        self.task_optimizer = TaskOptimizer(
            self.drone_fleet, 
            self.bus_system, 
            self.bus_stations, 
            self.bus_lines,
            self.event_emitter
        )
        
        # Initialize tasks and packages
        self.init_tasks_and_packages()
        
        # Simulation records
        self.simulation_records = []
        self.bus_records = []
        self.package_records = []
        
        # Simulation statistics
        self.delivered_packages = 0
        self.total_packages = len(self.tasks)
        
        # Subscribe to events
        self._subscribe_to_events()
        
        # Log initialization
        self.logger.info(f"Simulation initialized with {len(self.tasks)} tasks")
    
    def init_tasks_and_packages(self) -> None:
        """Initialize tasks and create packages"""
        # Initialize task state
        self.tasks = self.task_list
        self.packages = []
        self.task_index = 0
        
        for task in self.tasks:
            # Initialize task status
            task['pickup_status'] = 'unassigned'
            task['delivery_status'] = 'unassigned'
            
            # Create package
            package = Package(
                package_id=task['index'],
                task_id=task['index'],
                event_emitter=self.event_emitter
            )
            
            # Set package reference in task
            task['package_id'] = task['index']
            
            # Add package to list
            self.packages.append(package)
        
        self.logger.info(f"Initialized {len(self.tasks)} tasks and {len(self.packages)} packages")
    
    def _subscribe_to_events(self) -> None:
        """Subscribe to simulation events"""
        # Package events
        self.event_emitter.subscribe('package_status_changed', self._on_package_status_changed)
        
        # Delivery events
        self.event_emitter.subscribe('package_delivered', self._on_package_delivered)
        self.event_emitter.subscribe('package_directly_delivered', self._on_package_delivered)
    
    def _on_package_status_changed(self, data: Dict[str, Any]) -> None:
        """Handle package status change event"""
        package_id = data['package_id']
        new_status = data['new_status']
        
        # Find package and update its position
        package = self._get_package_by_id(package_id)
        if package:
            # Update package position based on status
            self._update_package_position(package)
            
            # Log package status change
            if new_status in ['delivered', 'directly_delivered']:
                self.logger.info(f"Package {package_id} delivered")
    
    def _on_package_delivered(self, data: Dict[str, Any]) -> None:
        """Handle package delivery event"""
        self.delivered_packages += 1
    
    def _update_package_position(self, package: Package) -> None:
        """Update package position based on its status and carrier"""
        task = self._get_task_by_id(package.task_id)
        if not task:
            return
        
        if package.status == 'at_pickup_point':
            # At pickup point
            package.update_position(task['pickup_x'], task['pickup_y'])
        elif 'pickup_drone' in package.status:
            # On pickup drone
            drone = self._get_drone_by_id(package.current_drone_id)
            if drone:
                package.update_position(drone.x, drone.y)
        elif package.status == 'on_bus':
            # On bus
            bus = self._get_bus_by_id(package.bus_id)
            if bus:
                package.update_position(bus.x, bus.y)
        elif 'delivery_drone' in package.status or 'direct_drone' in package.status:
            # On delivery or direct drone
            drone = self._get_drone_by_id(package.current_drone_id)
            if drone:
                package.update_position(drone.x, drone.y)
        elif package.status in ['delivered', 'directly_delivered']:
            # At delivery point
            package.update_position(task['delivery_x'], task['delivery_y'])
    
    def _get_package_by_id(self, package_id: int) -> Optional[Package]:
        """Get package by ID"""
        return next((p for p in self.packages if p.package_id == package_id), None)
    
    def _get_task_by_id(self, task_id: int) -> Optional[Dict[str, Any]]:
        """Get task by ID"""
        return next((t for t in self.tasks if t['index'] == task_id), None)
    
    def _get_drone_by_id(self, drone_id: int) -> Optional:
        """Get drone by ID"""
        if drone_id is None:
            return None
        return next((d for d in self.drone_fleet.drones if d.entity_id == drone_id), None)
    
    def _get_bus_by_id(self, bus_id: int) -> Optional:
        """Get bus by ID"""
        if bus_id is None:
            return None
        return next((b for b in self.bus_system.buses if b.entity_id == bus_id), None)
    
    def handle_drone_bus_interactions(self) -> None:
        """Handle interactions between drones and buses"""
        # Handle drone waiting for pickup bus
        for drone in self.drone_fleet.drones:
            if (drone.status == 'waiting_at_bus_station_for_pickup_bus' and 
                drone.current_task is not None and 
                drone.current_task['pickup_status'] == 'waiting_for_bus'):
                
                # Get line and delivery station
                current_line = drone.current_task['line']
                
                # Find nearest delivery station
                nearest_delivery_station = find_nearest_bus_station(
                    drone.current_task['delivery_x'], 
                    drone.current_task['delivery_y'], 
                    self.bus_stations, 
                    current_line
                )
                
                # Check for bus at station
                bus = self.bus_system.check_buses_at_station_for_pickup(
                    drone.x, drone.y, 
                    line=current_line, 
                    delivery_station=(nearest_delivery_station.x, nearest_delivery_station.y)
                )
                
                if bus and bus.can_pickup_package():
                    # Get package
                    package = self._get_package_by_id(drone.current_task['package_id'])
                    
                    if package:
                        # Load package onto bus
                        bus.load_package(package)
                        package.set_carrier('bus', bus.entity_id)
                        package.update_status('on_bus')
                        
                        # Update task and drone
                        drone.current_task['pickup_status'] = 'completed'
                        drone.current_task = None
                        drone.set_state('idle')
                        drone.pickup_tasks_completed += 1
                        
                        # Handle delivery scheduling
                        if package.task_id is not None:
                            task = self._get_task_by_id(package.task_id)
                            if task:
                                # Find delivery station
                                delivery_station = find_nearest_bus_station(
                                    task['delivery_x'], task['delivery_y'], 
                                    self.bus_stations, current_line
                                )
                                
                                # Predict bus arrival at delivery station
                                arrival_time = predict_single_bus_arrival(
                                    bus, (delivery_station.x, delivery_station.y), 
                                    self.current_time
                                )
                                
                                if arrival_time is not None:
                                    # Schedule delivery task
                                    task['delivery_assign_time'] = arrival_time
                                    task['selected_delivery_station'] = (
                                        delivery_station.x, delivery_station.y
                                    )
                                    
                                    self.logger.info(
                                        f"Scheduled delivery for task {task['index']} at time "
                                        f"{arrival_time} (current: {self.current_time})"
                                    )
            
            # Handle drone waiting for delivery bus
            elif drone.status == 'waiting_at_bus_station_for_delivery':
                package_id = drone.current_task.get('package_id') if drone.current_task else None
                package = self._get_package_by_id(package_id)
                
                if package and package.bus_id is not None:
                    # Check if bus has arrived
                    bus = self.bus_system.check_buses_at_station_for_delivery(
                        drone.x, drone.y, package.bus_id
                    )
                    
                    if bus and bus.current_package == package:
                        # Unload package from bus
                        unloaded_package = bus.unload_package()
                        
                        if unloaded_package:
                            # Update package status
                            unloaded_package.update_status('on_delivery_drone')
                            
                            # Assign delivery task to drone
                            task = self._get_task_by_id(unloaded_package.task_id)
                            if task:
                                drone.assign_delivery_task_to_delivery(
                                    task, task['delivery_x'], task['delivery_y'], 
                                    task['line']
                                )
                                
                                # Update package carrier
                                unloaded_package.set_carrier('drone', drone.entity_id)
                                
                                self.logger.info(
                                    f"Package {unloaded_package.package_id} transferred from "
                                    f"bus {bus.entity_id} to drone {drone.entity_id} for final delivery"
                                )
    
    def process_delivery_scheduling(self) -> None:
        """Process tasks that need delivery scheduling"""
        for task in self.tasks:
            if (task['pickup_status'] == 'completed' and 
                task['delivery_status'] == 'unassigned'):
                
                # Check if delivery time has been scheduled
                if 'delivery_assign_time' in task:
                    # Check if it's time to assign delivery
                    if self.current_time >= task['delivery_assign_time']:
                        # Try to allocate delivery task
                        success = self.drone_fleet.try_allocate_delivery_task(
                            task, 
                            task['delivery_x'], 
                            task['delivery_y'], 
                            self.bus_stations,
                            auto_create_drone=False
                        )
                        
                        if not success:
                            # Add to queue if allocation failed
                            self.drone_fleet.add_delivery_task_to_queue(
                                task, task['delivery_x'], task['delivery_y']
                            )
                else:
                    # No delivery time scheduled yet, find the bus with the package
                    package = self._get_package_by_id(task['package_id'])
                    
                    if package and package.status == 'on_bus' and package.bus_id is not None:
                        bus = self._get_bus_by_id(package.bus_id)
                        
                        if bus:
                            # Find delivery station
                            delivery_stations = find_nearest_bus_stations(
                                task['delivery_x'], task['delivery_y'],
                                self.bus_stations, task['line'], 3
                            )
                            
                            if delivery_stations:
                                # Find best station based on bus arrival time
                                best_station = None
                                best_arrival_time = float('inf')
                                
                                for station in delivery_stations:
                                    arrival_time = predict_single_bus_arrival(
                                        bus, (station.x, station.y), self.current_time
                                    )
                                    
                                    if arrival_time is not None and arrival_time < best_arrival_time:
                                        best_arrival_time = arrival_time
                                        best_station = station
                                
                                if best_station:
                                    # Schedule delivery task
                                    task['delivery_assign_time'] = best_arrival_time
                                    task['selected_delivery_station'] = (
                                        best_station.x, best_station.y
                                    )
                                    
                                    self.logger.info(
                                        f"Task {task['index']}: Scheduled delivery at time "
                                        f"{best_arrival_time} (current: {self.current_time})"
                                    )
    
    def record_state(self) -> None:
        """Record simulation state for analysis"""
        # Record drone states
        for drone in self.drone_fleet.drones:
            self.simulation_records.append({
                "time": self.current_time,
                "entity": "drone",
                "id": drone.entity_id,
                "x": drone.x,
                "y": drone.y,
                "status": drone.status,
                "battery": drone.battery_left
            })
        
        # Record bus states
        for bus in self.bus_system.buses:
            self.simulation_records.append({
                "time": self.current_time,
                "entity": "bus",
                "id": bus.entity_id,
                "x": bus.x,
                "y": bus.y,
                "status": bus.status,
                "direction": bus.direction,
                "has_package": bus.current_package is not None
            })
            
            # Record bus station data if at station
            current_station = bus.get_current_station()
            if current_station:
                station_id = self.station_mapping.get(
                    (current_station['x'], current_station['y']), None
                )
                
                if station_id is not None:
                    self.bus_records.append({
                        "time": self.current_time,
                        "entity": "bus",
                        "id": bus.entity_id,
                        "station": station_id,
                        "station_index": bus.station_index,
                        "line": bus.line,
                        "has_package": bus.current_package is not None
                    })
        
        # Record package states
        for package in self.packages:
            # Update package position
            self._update_package_position(package)
            
            self.package_records.append({
                "time": self.current_time,
                "package_id": package.package_id,
                "task_id": package.task_id,
                "status": package.status,
                "x": package.x,
                "y": package.y,
                "bus_id": package.bus_id,
                "drone_id": package.current_drone_id
            })
            
            # Record to logger
            self.logger.record_package_state(
                self.current_time, 
                package.package_id, 
                package.get_state()
            )
    
    def print_status(self) -> None:
        """Print simulation status to console"""
        if self.current_time % 50 == 0:
            idle_drones = self.drone_fleet.get_idle_drone_count()
            active_drones = self.drone_fleet.get_active_drone_count()
            low_battery_drones = self.drone_fleet.get_low_battery_drone_count()
            
            packages_on_drones = sum(
                1 for package in self.packages 
                if 'drone' in package.status
            )
            packages_on_buses = sum(
                1 for package in self.packages 
                if package.status == 'on_bus'
            )
            packages_delivered = sum(
                1 for package in self.packages 
                if package.status in ['delivered', 'directly_delivered']
            )
            
            print(f"\nTime {self.current_time} - System Status:")
            print(f"Active drones: {active_drones}, Idle drones: {idle_drones}")
            print(f"Packages on drones: {packages_on_drones}, On buses: {packages_on_buses}, Delivered: {packages_delivered}")
            print(f"Remaining tasks in optimizer: {self.task_optimizer.get_pending_task_count()}")
            print(f"Completion: {packages_delivered}/{self.total_packages} packages ({packages_delivered/self.total_packages*100:.1f}%)")
            
            # Battery warnings
            if low_battery_drones > 0:
                print(f"Warning: {low_battery_drones} drones have low battery (<{Config.DRONE_LOW_BATTERY_THRESHOLD})")
    
    def run_simulation(self) -> None:
        """Run the simulation for the specified number of time steps"""
        self.logger.info(f"Starting simulation for {self.total_time} time steps")
        
        # Initialize events to track
        new_tasks_found = False
        
        # Main simulation loop
        while self.current_time <= self.total_time:
            # Process task queues
            self.drone_fleet.process_delivery_task_queue(self.bus_stations)
            self.drone_fleet.process_direct_task_queue()
            self.drone_fleet.process_pickup_task_queue(self.bus_stations)
            
            # Check for new tasks
            new_tasks_found = False
            while (self.task_index < len(self.tasks) and 
                   self.tasks[self.task_index]['pickup_time'] <= self.current_time):
                task = self.tasks[self.task_index]
                
                if task['pickup_status'] == 'unassigned':
                    # Add task to optimizer
                    self.task_optimizer.add_task(task)
                    new_tasks_found = True
                
                self.task_index += 1
            
            # Run task optimizer if new tasks were found
            if new_tasks_found:
                self.task_optimizer.process_tasks(self.current_time)
            
            # Process delivery scheduling
            self.process_delivery_scheduling()
            
            # Move entities
            self.drone_fleet.move_all_drones()
            self.bus_system.simulate(self.bus_lines, self.bus_stations_df)
            
            # Handle drone-bus interactions
            self.handle_drone_bus_interactions()
            
            # Record state
            self.record_state()
            
            # Print status
            self.print_status()
            
            # Update time
            self.current_time += 1
        
        # Save simulation records
        self._save_simulation_records()
        
        # Print summary
        self.print_summary()
    
    def _save_simulation_records(self) -> None:
        """Save simulation records to CSV files"""
        # Create DataFrames
        simulation_df = pd.DataFrame(self.simulation_records)
        bus_simulation_df = pd.DataFrame(self.bus_records)
        package_simulation_df = pd.DataFrame(self.package_records)
        
        # Create output directory if it doesn't exist
        os.makedirs('simulation_output', exist_ok=True)
        
        # Save to CSV
        simulation_df.to_csv('simulation_output/simulation_records.csv', index=False)
        bus_simulation_df.to_csv('simulation_output/bus_records.csv', index=False)
        package_simulation_df.to_csv('simulation_output/package_records.csv', index=False)
        
        # Also save using logger
        self.logger.simulation_records = self.simulation_records
        self.logger.bus_records = self.bus_records
        self.logger.package_records = self.package_records
        self.logger.save_records()
        
        self.logger.info("Simulation records saved to CSV files")
    
    def calculate_statistics(self) -> Dict[str, Any]:
        """Calculate simulation statistics"""
        # Calculate task completion rate
        completed_tasks = sum(
            1 for task in self.tasks 
            if task['delivery_status'] == 'completed'
        )
        completion_rate = completed_tasks / len(self.tasks) * 100 if self.tasks else 0
        
        # Calculate average delivery time
        delivery_times = []
        for package in self.packages:
            if package.status in ['delivered', 'directly_delivered']:
                # Find first and last package records
                package_records = [
                    r for r in self.package_records 
                    if r['package_id'] == package.package_id
                ]
                
                if package_records:
                    first_record = package_records[0]
                    last_record = next(
                        (r for r in reversed(package_records) 
                         if r['status'] in ['delivered', 'directly_delivered']),
                        None
                    )
                    
                    if first_record and last_record:
                        delivery_time = last_record['time'] - first_record['time']
                        delivery_times.append(delivery_time)
        
        avg_delivery_time = sum(delivery_times) / len(delivery_times) if delivery_times else 0
        
        # Calculate drone utilization
        drone_states = {}
        for record in self.simulation_records:
            if record['entity'] == 'drone':
                drone_id = record['id']
                status = record['status']
                
                if drone_id not in drone_states:
                    drone_states[drone_id] = {'idle': 0, 'active': 0}
                
                if status == 'idle':
                    drone_states[drone_id]['idle'] += 1
                else:
                    drone_states[drone_id]['active'] += 1
        
        # Calculate utilization percentage
        utilization = {}
        for drone_id, states in drone_states.items():
            total_time = states['idle'] + states['active']
            if total_time > 0:
                utilization[drone_id] = (states['active'] / total_time) * 100
        
        avg_utilization = sum(utilization.values()) / len(utilization) if utilization else 0
        
        # Return statistics
        return {
            'completed_tasks': completed_tasks,
            'total_tasks': len(self.tasks),
            'completion_rate': completion_rate,
            'avg_delivery_time': avg_delivery_time,
            'drone_count': len(self.drone_fleet.drones),
            'bus_count': len(self.bus_system.buses),
            'avg_drone_utilization': avg_utilization
        }
    
    def print_summary(self) -> None:
        """Print simulation summary"""
        stats = self.calculate_statistics()
        
        print("\n=== Simulation Summary ===")
        print(f"Tasks completed: {stats['completed_tasks']}/{stats['total_tasks']} ({stats['completion_rate']:.2f}%)")
        print(f"Average delivery time: {stats['avg_delivery_time']:.2f} time units")
        print(f"Total drone count: {stats['drone_count']}")
        print(f"Total bus count: {stats['bus_count']}")
        print(f"Average drone utilization: {stats['avg_drone_utilization']:.2f}%")
        
        # Per-drone statistics
        print("\nPer-drone statistics:")
        for drone in self.drone_fleet.drones:
            print(f"Drone {drone.entity_id}: Completed {drone.pickup_tasks_completed} pickups and {drone.delivery_tasks_completed} deliveries")
        
        self.logger.info("Simulation completed successfully")