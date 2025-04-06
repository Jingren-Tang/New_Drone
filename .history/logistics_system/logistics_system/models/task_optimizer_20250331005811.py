import numpy as np
from scipy.optimize import linear_sum_assignment

from utils.bus_utils import calculate_best_line, find_nearest_bus_station
from utils.distance_utils import calculate_transport_distance
from utils.prediction_utils import optimize_station_selection

class MultiTaskOptimizer:
    """
    Optimizer for assigning multiple tasks to drones efficiently.
    """
    def __init__(self, drone_fleet, bus_system, bus_stations, bus_lines):
        """
        Initialize the multi-task optimizer.
        
        Args:
            drone_fleet: DroneFleet object
            bus_system: BusSystem object
            bus_stations (list): List of bus station objects
            bus_lines (list): List of bus line names
        """
        self.drone_fleet = drone_fleet
        self.bus_system = bus_system
        self.bus_stations = bus_stations
        self.bus_lines = bus_lines
        self.pending_tasks = []  # Task queue
        self.task_window_size = 3  # How many tasks to consider at once
        self.max_queue_size = 10  # Maximum queue size
    
    def add_task(self, task):
        """
        Add a task to the optimization queue.
        
        Args:
            task (dict): Task to add
            
        Returns:
            bool: True if the task was added, False if the queue is full
        """
        if len(self.pending_tasks) < self.max_queue_size:
            self.pending_tasks.append(task)
            return True
        return False
    
    def process_tasks(self, current_time):
        """
        Process tasks in the queue, finding optimal assignments.
        
        Args:
            current_time (float): Current simulation time
        """
        # If queue is empty, return
        if len(self.pending_tasks) == 0:
            return
        
        # Determine how many tasks to process this time
        tasks_to_process = min(len(self.pending_tasks), self.task_window_size)
        current_tasks = self.pending_tasks[:tasks_to_process]
        
        # Determine best line for each task
        for task in current_tasks:
            if 'line' not in task:
                best_line, _ = calculate_best_line(
                    task['pickup_x'], task['pickup_y'], 
                    task['delivery_x'], task['delivery_y'], 
                    self.bus_stations, self.bus_lines
                )
                task['line'] = best_line
        
        # Get available idle drones
        idle_drones = [drone for drone in self.drone_fleet.drones if drone.status == 'idle']
        
        # If no idle drones, return
        if not idle_drones:
            return
        
        # Calculate optimal station selections for each task
        optimized_tasks = []
        for task in current_tasks:
            pickup_rank, delivery_rank, analysis = optimize_station_selection(
                task, self.bus_system, self.bus_stations, current_time
            )
            
            if 'best_combination' in analysis:
                optimized_tasks.append({
                    'task': task,
                    'pickup_rank': pickup_rank,
                    'delivery_rank': delivery_rank,
                    'analysis': analysis
                })
        
        # If no optimizable tasks, return
        if not optimized_tasks:
            return
        
        # Find optimal drone-task assignments
        assignments = self._find_optimal_assignments(idle_drones, optimized_tasks)
        
        # Execute assignments
        for drone_id, task_info in assignments.items():
            drone = next((d for d in idle_drones if d.drone_id == drone_id), None)
            if drone and task_info:
                task = task_info['task']
                pickup_rank = task_info['pickup_rank']
                
                # Remove task from pending queue
                if task in self.pending_tasks:
                    self.pending_tasks.remove(task)
                
                # Assign task to drone
                if task_info['transport_mode'] == 'bus':
                    # Bus transport mode
                    success = self.drone_fleet.try_allocate_pickup_task(
                        task, task['pickup_x'], task['pickup_y'], pickup_rank
                    )
                    
                    if success:
                        print(f"Assigned task {task['index']} to drone {drone.drone_id} with pickup station rank {pickup_rank}")
                        
                        # Allocate delivery drone
                        delivery_rank = task_info['delivery_rank']
                        success_delivery = self.drone_fleet.try_allocate_delivery_task(
                            task, task['delivery_x'], task['delivery_y'], 
                            auto_create_drone=False
                        )
                        
                        if not success_delivery:
                            self.drone_fleet.add_delivery_task_to_queue(
                                task, task['delivery_x'], task['delivery_y']
                            )
                    else:
                        # If allocation failed, requeue task
                        print(f"Failed to assign task {task['index']} to drone {drone.drone_id}")
                        self.add_task(task)
                else:
                    # Direct transport mode
                    success = self.drone_fleet.try_allocate_direct_transport(
                        task, task['pickup_x'], task['pickup_y'], 
                        task['delivery_x'], task['delivery_y']
                    )
                    
                    if not success:
                        self.drone_fleet.add_direct_task_to_queue(
                            task, task['pickup_x'], task['pickup_y'],
                            task['delivery_x'], task['delivery_y']
                        )
    
    def _find_optimal_assignments(self, drones, optimized_tasks):
        """
        Find optimal drone-task assignments using the Hungarian algorithm.
        
        Args:
            drones (list): List of available drones
            optimized_tasks (list): List of optimized tasks
            
        Returns:
            dict: Mapping of drone IDs to task information
        """
        # Build cost matrix
        n_drones = len(drones)
        n_tasks = len(optimized_tasks)
        
        if n_drones == 0 or n_tasks == 0:
            return {}
        
        # Cost matrix: [number of drones x number of tasks]
        cost_matrix = np.zeros((n_drones, n_tasks))
        
        # Calculate cost for each drone-task pair
        for i, drone in enumerate(drones):
            for j, task_info in enumerate(optimized_tasks):
                task = task_info['task']
                
                # Determine transport mode: direct or bus
                bus_distance, drone_distance = calculate_transport_distance(
                    task['pickup_x'], task['pickup_y'], 
                    task['delivery_x'], task['delivery_y'],
                    self.bus_stations, self.bus_lines
                )
                
                if drone_distance - 10 < bus_distance:  # Using a tolerance
                    # Direct transport mode
                    transport_mode = 'direct'
                    # Calculate drone->pickup->delivery total distance
                    dist_to_pickup = np.sqrt(
                        (drone.x - task['pickup_x'])**2 + (drone.y - task['pickup_y'])**2
                    )
                    dist_pickup_to_delivery = drone_distance
                    total_cost = dist_to_pickup + dist_pickup_to_delivery
                else:
                    # Bus transport mode
                    transport_mode = 'bus'
                    analysis = task_info['analysis']
                    best_combo = analysis.get('best_combination', {})
                    
                    # Get pickup station coordinates
                    pickup_station = best_combo.get('pickup_station', None)
                    if pickup_station:
                        # Calculate drone->pickup_point->pickup_station total distance
                        dist_to_pickup = np.sqrt(
                            (drone.x - task['pickup_x'])**2 + (drone.y - task['pickup_y'])**2
                        )
                        pickup_x, pickup_y = task['pickup_x'], task['pickup_y']
                        station_x, station_y = pickup_station
                        dist_pickup_to_station = np.sqrt(
                            (pickup_x - station_x)**2 + (pickup_y - station_y)**2
                        )
                        total_cost = dist_to_pickup + dist_pickup_to_station
                    else:
                        # If no valid station info, use a large cost
                        total_cost = 1000
                
                # Set cost matrix value
                cost_matrix[i, j] = total_cost
                
                # Store transport mode in task info
                optimized_tasks[j]['transport_mode'] = transport_mode
        
        # Use Hungarian algorithm to find optimal assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        
        # Create assignment result
        assignments = {}
        for i, j in zip(row_ind, col_ind):
            drone_id = drones[i].drone_id
            task_info = optimized_tasks[j]
            assignments[drone_id] = task_info
        
        return assignments