"""
Task optimizer for efficient task allocation
"""
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import scipy.optimize

from ..entities.drone import Drone
from ..entities.bus_station import BusStation
from ..utils.events import EventEmitter, global_event_emitter
from ..utils.distance import (
    calculate_distance, find_nearest_bus_station, 
    find_nearest_bus_stations, calculate_transport_distance,
    calculate_best_line
)
from ..utils.prediction import predict_bus_arrival_times, predict_single_bus_arrival
from ..config.settings import Config


class TaskOptimizer:
    """
    Optimize task allocation for maximum efficiency
    """
    
    def __init__(self, drone_fleet, bus_system, bus_stations: List[BusStation], 
                bus_lines: List[str], event_emitter: Optional[EventEmitter] = None):
        """
        Initialize task optimizer
        
        Args:
            drone_fleet: DroneFleet object
            bus_system: BusSystem object
            bus_stations: List of bus stations
            bus_lines: List of bus lines
            event_emitter: Optional event emitter for publishing events
        """
        self.drone_fleet = drone_fleet
        self.bus_system = bus_system
        self.bus_stations = bus_stations
        self.bus_lines = bus_lines
        self.pending_tasks = []
        self.task_window_size = Config.TASK_WINDOW_SIZE
        self.max_queue_size = Config.MAX_QUEUE_SIZE
        self.event_emitter = event_emitter or global_event_emitter
    
    def add_task(self, task: Dict[str, Any]) -> bool:
        """
        Add task to optimization queue
        
        Args:
            task: Task to add
            
        Returns:
            bool: True if added successfully
        """
        if len(self.pending_tasks) < self.max_queue_size:
            self.pending_tasks.append(task)
            
            # Emit event
            if self.event_emitter:
                self.event_emitter.emit('task_added_to_optimizer', {
                    'task_id': task.get('index'),
                    'queue_size': len(self.pending_tasks)
                })
                
            return True
            
        return False
    
    def optimize_station_selection(self, task: Dict[str, Any], current_time: int) -> Tuple[int, int, Dict[str, Any]]:
        """
        Optimize station selection based on real-time bus predictions
        
        Args:
            task: Task to optimize
            current_time: Current simulation time
            
        Returns:
            Tuple[int, int, Dict[str, Any]]: (pickup_rank, delivery_rank, analysis)
        """
        line = task['line']
        pickup_x, pickup_y = task['pickup_x'], task['pickup_y']
        delivery_x, delivery_y = task['delivery_x'], task['delivery_y']
        
        # Get nearby stations for pickup and delivery
        pickup_stations = find_nearest_bus_stations(pickup_x, pickup_y, self.bus_stations, line, 3)
        delivery_stations = find_nearest_bus_stations(delivery_x, delivery_y, self.bus_stations, line, 3)
        
        best_combination = None
        min_total_time = float('inf')
        results = []
        
        for pickup_rank, pickup_station in enumerate(pickup_stations):
            # Calculate drone time to pickup station
            drone_to_pickup_station_dist = calculate_distance(
                pickup_x, pickup_y, pickup_station.x, pickup_station.y
            )
            drone_to_pickup_time = drone_to_pickup_station_dist / Config.DRONE_SPEED
            
            # Predict bus arrival times at pickup station
            pickup_predictions = predict_bus_arrival_times(
                self.bus_system, line, 
                (pickup_station.x, pickup_station.y),
                current_time + drone_to_pickup_time
            )
            
            if not pickup_predictions:
                continue  # No buses will arrive at this station
            
            # Choose fastest bus
            best_pickup_bus = pickup_predictions[0]
            pickup_bus_arrival = best_pickup_bus['predicted_arrival']
            
            for delivery_rank, delivery_station in enumerate(delivery_stations):
                # Calculate drone time from delivery station to delivery point
                drone_from_delivery_station_dist = calculate_distance(
                    delivery_x, delivery_y, delivery_station.x, delivery_station.y
                )
                drone_from_delivery_time = drone_from_delivery_station_dist / Config.DRONE_SPEED
                
                # Predict when bus will arrive at delivery station
                bus_id = best_pickup_bus['bus_id']
                bus = next((b for b in self.bus_system.buses if b.entity_id == bus_id), None)
                
                if bus:
                    # Predict bus arrival at delivery station
                    delivery_arrival = predict_single_bus_arrival(
                        bus, (delivery_station.x, delivery_station.y),
                        pickup_bus_arrival
                    )
                    
                    if delivery_arrival is not None:
                        # Calculate total time
                        total_time = (
                            drone_to_pickup_time +  # Drone to pickup
                            (delivery_arrival - pickup_bus_arrival) +  # Bus travel
                            drone_from_delivery_time  # Drone to delivery
                        )
                        
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
                    'task_id': task.get('index'),
                    'line': line,
                    'pickup_point': (pickup_x, pickup_y),
                    'delivery_point': (delivery_x, delivery_y),
                    'best_combination': best_combination,
                    'all_combinations': results[:5]  # Top 5 combinations
                }
            )
        else:
            return 0, 0, {'error': 'No valid combinations found'}
    
    def process_tasks(self, current_time: int) -> None:
        """
        Process pending tasks
        
        Args:
            current_time: Current simulation time
        """
        # Skip if no tasks
        if not self.pending_tasks:
            return
        
        # Determine tasks to process
        tasks_to_process = min(len(self.pending_tasks), self.task_window_size)
        current_tasks = self.pending_tasks[:tasks_to_process]
        
        # Determine best lines for tasks if not already set
        for task in current_tasks:
            if 'line' not in task:
                best_line, _ = calculate_best_line(
                    task['pickup_x'], task['pickup_y'],
                    task['delivery_x'], task['delivery_y'],
                    self.bus_stations, self.bus_lines
                )
                task['line'] = best_line
        
        # Get idle drones
        idle_drones = [drone for drone in self.drone_fleet.drones if drone.status == 'idle']
        
        # Skip if no idle drones
        if not idle_drones:
            if self.event_emitter:
                self.event_emitter.emit('no_idle_drones_for_optimizer', {
                    'pending_tasks': len(self.pending_tasks)
                })
            return
        
        # Optimize task assignment
        optimized_tasks = []
        for task in current_tasks:
            pickup_rank, delivery_rank, analysis = self.optimize_station_selection(
                task, current_time
            )
            
            if 'best_combination' in analysis:
                optimized_tasks.append({
                    'task': task,
                    'pickup_rank': pickup_rank,
                    'delivery_rank': delivery_rank,
                    'analysis': analysis
                })
        
        # Skip if no optimized tasks
        if not optimized_tasks:
            if self.event_emitter:
                self.event_emitter.emit('no_optimized_tasks', {
                    'current_time': current_time
                })
            return
        
        # Find optimal assignments
        assignments = self._find_optimal_assignments(idle_drones, optimized_tasks)
        
        # Execute assignments
        for drone_id, task_info in assignments.items():
            drone = next((d for d in idle_drones if d.entity_id == drone_id), None)
            if drone and task_info:
                task = task_info['task']
                pickup_rank = task_info['pickup_rank']
                
                # Remove task from pending queue
                if task in self.pending_tasks:
                    self.pending_tasks.remove(task)
                
                # Assign task
                if task_info.get('transport_mode') == 'bus':
                    # Use bus transport
                    success = self.drone_fleet.try_allocate_pickup_task(
                        task, task['pickup_x'], task['pickup_y'], self.bus_stations, pickup_rank
                    )
                    
                    if success:
                        if self.event_emitter:
                            self.event_emitter.emit('optimizer_assigned_pickup', {
                                'drone_id': drone.entity_id,
                                'task_id': task.get('index'),
                                'pickup_rank': pickup_rank
                            })
                        
                        # Schedule delivery task
                        delivery_rank = task_info['delivery_rank']
                        success_delivery = self.drone_fleet.try_allocate_delivery_task(
                            task, task['delivery_x'], task['delivery_y'], 
                            self.bus_stations, False
                        )
                        
                        if not success_delivery:
                            self.drone_fleet.add_delivery_task_to_queue(
                                task, task['delivery_x'], task['delivery_y']
                            )
                    else:
                        # Failed to assign, add back to queue
                        self.add_task(task)
                        
                        if self.event_emitter:
                            self.event_emitter.emit('optimizer_failed_assignment', {
                                'drone_id': drone.entity_id,
                                'task_id': task.get('index')
                            })
                else:
                    # Use direct transport
                    success = self.drone_fleet.try_allocate_direct_transport(
                        task, task['pickup_x'], task['pickup_y'],
                        task['delivery_x'], task['delivery_y']
                    )
                    
                    if not success:
                        self.drone_fleet.add_direct_task_to_queue(
                            task, task['pickup_x'], task['pickup_y'],
                            task['delivery_x'], task['delivery_y']
                        )
                        
                        if self.event_emitter:
                            self.event_emitter.emit('optimizer_failed_direct_assignment', {
                                'drone_id': drone.entity_id,
                                'task_id': task.get('index')
                            })
    
    def _find_optimal_assignments(self, drones: List[Drone], 
                                 optimized_tasks: List[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
        """
        Find optimal assignment of drones to tasks
        
        Args:
            drones: List of idle drones
            optimized_tasks: List of optimized task info
            
        Returns:
            Dict[int, Dict[str, Any]]: Map of drone ID to task info
        """
        n_drones = len(drones)
        n_tasks = len(optimized_tasks)
        
        if n_drones == 0 or n_tasks == 0:
            return {}
        
        # Create cost matrix
        cost_matrix = np.zeros((n_drones, n_tasks))
        
        # Calculate costs for all drone-task pairs
        for i, drone in enumerate(drones):
            for j, task_info in enumerate(optimized_tasks):
                task = task_info['task']
                
                # Determine transport mode
                bus_distance, drone_distance = calculate_transport_distance(
                    task['pickup_x'], task['pickup_y'],
                    task['delivery_x'], task['delivery_y'],
                    self.bus_stations, self.bus_lines
                )
                
                # Use direct transport if more efficient
                if drone_distance - Config.DIRECT_TRANSPORT_THRESHOLD < bus_distance:
                    # Direct transport mode
                    transport_mode = 'direct'
                    
                    # Calculate cost as drone->pickup->delivery distance
                    dist_to_pickup = calculate_distance(
                        drone.x, drone.y, task['pickup_x'], task['pickup_y']
                    )
                    dist_pickup_to_delivery = drone_distance
                    total_cost = dist_to_pickup + dist_pickup_to_delivery
                    
                    # Check battery capacity
                    if total_cost > drone.battery_left:
                        # Add large penalty for insufficient battery
                        total_cost = 10000
                else:
                    # Bus transport mode
                    transport_mode = 'bus'
                    analysis = task_info['analysis']
                    best_combo = analysis.get('best_combination', {})
                    
                    # Get pickup station
                    pickup_station = best_combo.get('pickup_station')
                    if pickup_station:
                        # Calculate cost as drone->pickup->station distance
                        dist_to_pickup = calculate_distance(
                            drone.x, drone.y, task['pickup_x'], task['pickup_y']
                        )
                        pickup_x, pickup_y = task['pickup_x'], task['pickup_y']
                        station_x, station_y = pickup_station
                        dist_pickup_to_station = calculate_distance(
                            pickup_x, pickup_y, station_x, station_y
                        )
                        total_cost = dist_to_pickup + dist_pickup_to_station
                        
                        # Check battery capacity
                        if total_cost > drone.battery_left:
                            # Add large penalty for insufficient battery
                            total_cost = 10000
                    else:
                        # No valid station info
                        total_cost = 5000
                
                # Store cost in matrix
                cost_matrix[i, j] = total_cost
                
                # Store transport mode
                optimized_tasks[j]['transport_mode'] = transport_mode
        
        # Solve assignment problem
        try:
            row_ind, col_ind = scipy.optimize.linear_sum_assignment(cost_matrix)
            
            # Create assignments map
            assignments = {}
            for i, j in zip(row_ind, col_ind):
                drone_id = drones[i].entity_id
                task_info = optimized_tasks[j]
                
                # Only include assignments with reasonable cost
                if cost_matrix[i, j] < 5000:
                    assignments[drone_id] = task_info
            
            return assignments
        except Exception as e:
            # Handle errors in assignment algorithm
            if self.event_emitter:
                self.event_emitter.emit('optimizer_assignment_error', {
                    'error': str(e)
                })
            
            # Return empty assignments
            return {}
    
    def get_pending_task_count(self) -> int:
        """Get number of pending tasks"""
        return len(self.pending_tasks)
    
    def get_state(self) -> Dict[str, Any]:
        """Get system state as dictionary for logging/recording"""
        return {
            "pending_tasks": len(self.pending_tasks),
            "task_window_size": self.task_window_size,
            "max_queue_size": self.max_queue_size
        }