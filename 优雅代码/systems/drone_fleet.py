"""
Drone fleet implementation for managing drones
"""
import math
from collections import deque
from typing import List, Dict, Any, Optional, Tuple, Deque

from ..entities.drone import Drone
from ..entities.bus_station import BusStation
from ..utils.events import EventEmitter, global_event_emitter
from ..utils.distance import (
    calculate_distance, find_nearest_bus_station, 
    find_nearest_bus_stations, calculate_transport_distance
)
from ..config.settings import Config


class DroneFleet:
    """
    System for managing all drones in the simulation
    """
    
    def __init__(self, initial_drone_count: int = Config.DRONE_INITIAL_COUNT,
                event_emitter: Optional[EventEmitter] = None):
        """
        Initialize drone fleet
        
        Args:
            initial_drone_count: Number of drones to create initially
            event_emitter: Optional event emitter for publishing events
        """
        self.event_emitter = event_emitter or global_event_emitter
        
        # Create initial drones
        self.drones: List[Drone] = [
            Drone(
                entity_id=i,
                region='pickup',
                x=Config.CHARGING_STATION_X,
                y=Config.CHARGING_STATION_Y,
                event_emitter=self.event_emitter
            ) 
            for i in range(initial_drone_count)
        ]
        
        self.drone_count = initial_drone_count
        
        # Task queues
        self.pickup_task_queue: Deque[Tuple[Dict[str, Any], float, float, int]] = deque()
        self.delivery_task_queue: Deque[Tuple[Dict[str, Any], float, float]] = deque()
        self.direct_task_queue: Deque[Tuple[Dict[str, Any], float, float, float, float]] = deque()
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('drone_fleet_initialized', {
                'drone_count': initial_drone_count,
                'charging_station': (Config.CHARGING_STATION_X, Config.CHARGING_STATION_Y)
            })
    
    def get_idle_drone(self, target_x: float, target_y: float) -> Optional[Drone]:
        """
        Find the idle drone closest to target position
        
        Args:
            target_x: Target x coordinate
            target_y: Target y coordinate
            
        Returns:
            Drone or None: Nearest idle drone or None if no idle drones
        """
        nearest_drone = None
        min_distance = float('inf')
        
        for drone in self.drones:
            if drone.status == 'idle':
                distance = calculate_distance(drone.x, drone.y, target_x, target_y)
                if distance < min_distance:
                    min_distance = distance
                    nearest_drone = drone
        
        return nearest_drone
    
    def create_new_drone(self) -> Drone:
        """
        Create a new drone and add to fleet
        
        Returns:
            Drone: The newly created drone
        """
        new_drone = Drone(
            entity_id=self.drone_count,
            region='pickup',
            x=Config.CHARGING_STATION_X,
            y=Config.CHARGING_STATION_Y,
            event_emitter=self.event_emitter
        )
        
        self.drones.append(new_drone)
        self.drone_count += 1
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('drone_created', {
                'drone_id': new_drone.entity_id,
                'position': (new_drone.x, new_drone.y)
            })
        
        return new_drone
    
    def ensure_delivery_drone_availability(self, task: Dict[str, Any], 
                                          target_x: float, target_y: float,
                                          bus_stations: List[BusStation]) -> None:
        """
        Create a new drone to handle delivery if needed
        
        Args:
            task: Task to assign
            target_x: Target x coordinate
            target_y: Target y coordinate
            bus_stations: List of bus stations
        """
        # Create a new drone
        new_drone = self.create_new_drone()
        
        # Find nearest bus station and assign delivery task
        nearest_station = find_nearest_bus_station(
            target_x, target_y, bus_stations, task['line']
        )
        
        new_drone.assign_delivery_task_to_station(
            task, nearest_station.x, nearest_station.y, task['line']
        )
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('new_drone_assigned_delivery', {
                'drone_id': new_drone.entity_id,
                'task_id': task.get('index'),
                'target': (target_x, target_y),
                'station': (nearest_station.x, nearest_station.y),
                'line': task['line']
            })
    
    def try_allocate_direct_transport(self, task: Dict[str, Any], 
                                     pickup_x: float, pickup_y: float,
                                     delivery_x: float, delivery_y: float) -> bool:
        """
        Allocate a drone for direct transport from pickup to delivery
        
        Args:
            task: Task to assign
            pickup_x: Pickup x coordinate
            pickup_y: Pickup y coordinate
            delivery_x: Delivery x coordinate
            delivery_y: Delivery y coordinate
            
        Returns:
            bool: True if drone allocated successfully
        """
        # Find nearest idle drone
        nearest_drone = self.get_idle_drone(pickup_x, pickup_y)
        if not nearest_drone:
            # No idle drone available
            if self.event_emitter:
                self.event_emitter.emit('no_idle_drone', {
                    'task_id': task.get('index'),
                    'task_type': 'direct_transport',
                    'pickup': (pickup_x, pickup_y)
                })
            return False
        
        # Calculate total distance needed
        dist_to_pickup = calculate_distance(
            nearest_drone.x, nearest_drone.y, pickup_x, pickup_y
        )
        dist_pickup_to_delivery = calculate_distance(
            pickup_x, pickup_y, delivery_x, delivery_y
        )
        total_distance = dist_to_pickup + dist_pickup_to_delivery
        
        # Check if drone has enough battery
        if total_distance > nearest_drone.battery_left:
            # Not enough battery, send to charge
            nearest_drone.go_to_charge()
            
            if self.event_emitter:
                self.event_emitter.emit('drone_battery_insufficient', {
                    'drone_id': nearest_drone.entity_id,
                    'task_id': task.get('index'),
                    'battery_left': nearest_drone.battery_left,
                    'distance_needed': total_distance
                })
            
            return False
        
        # Assign direct transport task
        nearest_drone.assign_direct_transport(
            task, pickup_x, pickup_y, delivery_x, delivery_y
        )
        
        # Update task status
        task['pickup_status'] = 'assigned'
        task['delivery_status'] = 'assigned'
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('direct_transport_assigned', {
                'drone_id': nearest_drone.entity_id,
                'task_id': task.get('index'),
                'pickup': (pickup_x, pickup_y),
                'delivery': (delivery_x, delivery_y)
            })
        
        return True
    
    def try_allocate_delivery_task(self, task: Dict[str, Any], 
                                  target_x: float, target_y: float,
                                  bus_stations: List[BusStation],
                                  auto_create_drone: bool = False) -> bool:
        """
        Allocate a drone for delivery task
        
        Args:
            task: Task to assign
            target_x: Target x coordinate
            target_y: Target y coordinate
            bus_stations: List of bus stations
            auto_create_drone: Whether to automatically create a new drone if none available
            
        Returns:
            bool: True if drone allocated successfully
        """
        # Find nearest bus station
        nearest_station = find_nearest_bus_station(
            target_x, target_y, bus_stations, task['line']
        )
        
        # Find nearest idle drone
        nearest_drone = self.get_idle_drone(nearest_station.x, nearest_station.y)
        
        # If no idle drone available and auto-create enabled
        if nearest_drone is None:
            if auto_create_drone:
                self.ensure_delivery_drone_availability(task, target_x, target_y, bus_stations)
                return True
            else:
                # No drone available
                if self.event_emitter:
                    self.event_emitter.emit('no_idle_drone', {
                        'task_id': task.get('index'),
                        'task_type': 'delivery',
                        'target': (target_x, target_y)
                    })
                return False
        
        # If delivery point is at bus station, mark as completed
        if nearest_station.x == target_x and nearest_station.y == target_y:
            task['delivery_status'] = 'completed'
            nearest_drone.delivery_tasks_completed += 1
            
            if self.event_emitter:
                self.event_emitter.emit('delivery_completed_at_station', {
                    'task_id': task.get('index'),
                    'position': (target_x, target_y)
                })
            
            return True
        
        # Calculate distances to check battery
        dist_to_station = calculate_distance(
            nearest_drone.x, nearest_drone.y, nearest_station.x, nearest_station.y
        )
        dist_to_delivery = calculate_distance(
            nearest_station.x, nearest_station.y, target_x, target_y
        )
        total_needed = dist_to_station + dist_to_delivery
        
        # Check if drone has enough battery
        if total_needed > nearest_drone.battery_left:
            # Not enough battery, send to charge
            nearest_drone.go_to_charge()
            
            if self.event_emitter:
                self.event_emitter.emit('drone_battery_insufficient', {
                    'drone_id': nearest_drone.entity_id,
                    'task_id': task.get('index'),
                    'battery_left': nearest_drone.battery_left,
                    'distance_needed': total_needed
                })
            
            return False
        
        # Assign delivery task
        nearest_drone.assign_delivery_task_to_station(
            task, nearest_station.x, nearest_station.y, task['line']
        )
        
        # Update task status
        task['delivery_status'] = 'assigned'
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('delivery_task_assigned', {
                'drone_id': nearest_drone.entity_id,
                'task_id': task.get('index'),
                'target': (target_x, target_y),
                'station': (nearest_station.x, nearest_station.y),
                'line': task['line']
            })
        
        return True
    
    def try_allocate_pickup_task(self, task: Dict[str, Any], 
                               target_x: float, target_y: float,
                               bus_stations: List[BusStation],
                               station_rank: int = 0) -> bool:
        """
        Allocate a drone for pickup task
        
        Args:
            task: Task to assign
            target_x: Target x coordinate
            target_y: Target y coordinate
            bus_stations: List of bus stations
            station_rank: Rank of station to choose (0 = nearest)
            
        Returns:
            bool: True if drone allocated successfully
        """
        # Get multiple nearest bus stations
        nearest_stations = find_nearest_bus_stations(
            target_x, target_y, bus_stations, task['line'], 3
        )
        
        # Ensure station_rank is valid
        if station_rank >= len(nearest_stations):
            station_rank = len(nearest_stations) - 1
        
        # Select station based on rank
        selected_station = nearest_stations[station_rank]
        
        # Find nearest idle drone
        nearest_drone = self.get_idle_drone(target_x, target_y)
        if not nearest_drone:
            # No idle drone available
            if self.event_emitter:
                self.event_emitter.emit('no_idle_drone', {
                    'task_id': task.get('index'),
                    'task_type': 'pickup',
                    'target': (target_x, target_y)
                })
            return False
        
        # Calculate distances to check battery
        dist_to_pickup = calculate_distance(
            nearest_drone.x, nearest_drone.y, target_x, target_y
        )
        dist_to_station = calculate_distance(
            target_x, target_y, selected_station.x, selected_station.y
        )
        total_needed = dist_to_pickup + dist_to_station
        
        # Check if drone has enough battery
        if total_needed > nearest_drone.battery_left:
            # Not enough battery, send to charge
            nearest_drone.go_to_charge()
            
            if self.event_emitter:
                self.event_emitter.emit('drone_battery_insufficient', {
                    'drone_id': nearest_drone.entity_id,
                    'task_id': task.get('index'),
                    'battery_left': nearest_drone.battery_left,
                    'distance_needed': total_needed
                })
            
            return False
        
        # Assign pickup task
        nearest_drone.assign_pickup_task(
            task, target_x, target_y, task['line'], station_rank
        )
        
        # Update task status
        task['pickup_status'] = 'assigned'
        
        # Record selected station
        task['selected_station_x'] = selected_station.x
        task['selected_station_y'] = selected_station.y
        task['station_rank'] = station_rank
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('pickup_task_assigned', {
                'drone_id': nearest_drone.entity_id,
                'task_id': task.get('index'),
                'target': (target_x, target_y),
                'station': (selected_station.x, selected_station.y),
                'line': task['line'],
                'station_rank': station_rank
            })
        
        return True
    
    def add_pickup_task_to_queue(self, task: Dict[str, Any], 
                                target_x: float, target_y: float, 
                                station_rank: int = 0) -> None:
        """
        Add pickup task to waiting queue
        
        Args:
            task: Task to queue
            target_x: Target x coordinate
            target_y: Target y coordinate
            station_rank: Rank of station to choose (0 = nearest)
        """
        self.pickup_task_queue.append((task, target_x, target_y, station_rank))
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('pickup_task_queued', {
                'task_id': task.get('index'),
                'target': (target_x, target_y),
                'station_rank': station_rank,
                'queue_length': len(self.pickup_task_queue)
            })
    
    def add_delivery_task_to_queue(self, task: Dict[str, Any], 
                                  target_x: float, target_y: float) -> None:
        """
        Add delivery task to waiting queue
        
        Args:
            task: Task to queue
            target_x: Target x coordinate
            target_y: Target y coordinate
        """
        self.delivery_task_queue.append((task, target_x, target_y))
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('delivery_task_queued', {
                'task_id': task.get('index'),
                'target': (target_x, target_y),
                'queue_length': len(self.delivery_task_queue)
            })
    
    def add_direct_task_to_queue(self, task: Dict[str, Any], 
                               pickup_x: float, pickup_y: float,
                               delivery_x: float, delivery_y: float) -> None:
        """
        Add direct transport task to waiting queue
        
        Args:
            task: Task to queue
            pickup_x: Pickup x coordinate
            pickup_y: Pickup y coordinate
            delivery_x: Delivery x coordinate
            delivery_y: Delivery y coordinate
        """
        self.direct_task_queue.append((task, pickup_x, pickup_y, delivery_x, delivery_y))
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('direct_task_queued', {
                'task_id': task.get('index'),
                'pickup': (pickup_x, pickup_y),
                'delivery': (delivery_x, delivery_y),
                'queue_length': len(self.direct_task_queue)
            })
    
    def process_pickup_task_queue(self, bus_stations: List[BusStation]) -> None:
        """
        Process queued pickup tasks
        
        Args:
            bus_stations: List of bus stations
        """
        for item in list(self.pickup_task_queue):
            # Handle different tuple formats for backward compatibility
            if len(item) == 4:  # (task, x, y, station_rank)
                task, target_x, target_y, station_rank = item
                success = self.try_allocate_pickup_task(
                    task, target_x, target_y, bus_stations, station_rank
                )
            else:  # (task, x, y)
                task, target_x, target_y = item
                success = self.try_allocate_pickup_task(
                    task, target_x, target_y, bus_stations
                )
            
            # Remove from queue if successfully allocated
            if success:
                self.pickup_task_queue.remove(item)
                
                # Emit event
                if self.event_emitter:
                    self.event_emitter.emit('pickup_task_dequeued', {
                        'task_id': task.get('index'),
                        'success': True,
                        'remaining_queue': len(self.pickup_task_queue)
                    })
    
    def process_delivery_task_queue(self, bus_stations: List[BusStation]) -> None:
        """
        Process queued delivery tasks
        
        Args:
            bus_stations: List of bus stations
        """
        for task, target_x, target_y in list(self.delivery_task_queue):
            success = self.try_allocate_delivery_task(
                task, target_x, target_y, bus_stations, False
            )
            
            # Remove from queue if successfully allocated
            if success:
                self.delivery_task_queue.remove((task, target_x, target_y))
                
                # Emit event
                if self.event_emitter:
                    self.event_emitter.emit('delivery_task_dequeued', {
                        'task_id': task.get('index'),
                        'success': True,
                        'remaining_queue': len(self.delivery_task_queue)
                    })
    
    def process_direct_task_queue(self) -> None:
        """Process queued direct transport tasks"""
        for item in list(self.direct_task_queue):
            task, pickup_x, pickup_y, delivery_x, delivery_y = item
            success = self.try_allocate_direct_transport(
                task, pickup_x, pickup_y, delivery_x, delivery_y
            )
            
            # Remove from queue if successfully allocated
            if success:
                self.direct_task_queue.remove(item)
                
                # Emit event
                if self.event_emitter:
                    self.event_emitter.emit('direct_task_dequeued', {
                        'task_id': task.get('index'),
                        'success': True,
                        'remaining_queue': len(self.direct_task_queue)
                    })
    
    def get_pickup_queue(self) -> List[Tuple[Dict[str, Any], float, float, int]]:
        """Get pickup task queue"""
        return list(self.pickup_task_queue)
    
    def get_delivery_queue(self) -> List[Tuple[Dict[str, Any], float, float]]:
        """Get delivery task queue"""
        return list(self.delivery_task_queue)
    
    def get_direct_queue(self) -> List[Tuple[Dict[str, Any], float, float, float, float]]:
        """Get direct transport task queue"""
        return list(self.direct_task_queue)
    
    def move_all_drones(self) -> None:
        """Move all drones"""
        for drone in self.drones:
            drone.move()
    
    def get_idle_drone_count(self) -> int:
        """Get number of idle drones"""
        return sum(1 for drone in self.drones if drone.status == 'idle')
    
    def get_active_drone_count(self) -> int:
        """Get number of active drones"""
        return len(self.drones) - self.get_idle_drone_count()
    
    def get_low_battery_drone_count(self) -> int:
        """Get number of drones with low battery"""
        return sum(1 for drone in self.drones 
                  if drone.battery_left < Config.DRONE_LOW_BATTERY_THRESHOLD)
    
    def get_state(self) -> Dict[str, Any]:
        """Get system state as dictionary for logging/recording"""
        return {
            "drone_count": len(self.drones),
            "idle_drones": self.get_idle_drone_count(),
            "active_drones": self.get_active_drone_count(),
            "low_battery_drones": self.get_low_battery_drone_count(),
            "pickup_queue": len(self.pickup_task_queue),
            "delivery_queue": len(self.delivery_task_queue),
            "direct_queue": len(self.direct_task_queue)
        }