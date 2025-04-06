"""
Drone entity implementation
"""
import math
from typing import Optional, Dict, Any, Tuple

from .entity import Entity
from ..utils.distance import calculate_distance
from ..utils.events import EventEmitter, global_event_emitter
from ..config.settings import Config


class DroneState:
    """Base class for drone states"""
    
    def execute(self, drone: 'Drone') -> None:
        """Execute state logic"""
        pass


class IdleState(DroneState):
    """Drone is idle and waiting for tasks"""
    
    def execute(self, drone: 'Drone') -> None:
        """Nothing to do when idle"""
        pass


class MovingState(DroneState):
    """Drone is moving toward a target"""
    
    def execute(self, drone: 'Drone') -> None:
        """Move toward target"""
        if drone.target_x is None or drone.target_y is None:
            return
            
        # Calculate distance to target
        distance_to_target = calculate_distance(
            drone.x, drone.y, drone.target_x, drone.target_y
        )
        
        # Check if we can reach the target in this step
        step_distance = min(distance_to_target, drone.speed)
        
        # If we can't move, return early
        if step_distance == 0:
            return
            
        # Calculate movement vector
        move_ratio = step_distance / distance_to_target
        dx = move_ratio * (drone.target_x - drone.x)
        dy = move_ratio * (drone.target_y - drone.y)
        
        # Move drone
        drone.x += dx
        drone.y += dy
        
        # Consume battery
        drone.consume_battery(step_distance)
        
        # Update time until idle
        drone.time_until_idle = max(drone.time_until_idle - 1, 0)
        
        # Check if we've arrived at the target
        if abs(drone.x - drone.target_x) < 1e-7 and abs(drone.y - drone.target_y) < 1e-7:
            drone.on_arrival()


class ToChargingStationState(DroneState):
    """Drone is moving to charging station"""
    
    def execute(self, drone: 'Drone') -> None:
        """Move toward charging station"""
        # Calculate distance to charging station
        distance = calculate_distance(
            drone.x, drone.y, 
            Config.CHARGING_STATION_X, Config.CHARGING_STATION_Y
        )
        
        if distance <= drone.speed:
            # We can reach the charging station in this step
            drone.x = Config.CHARGING_STATION_X
            drone.y = Config.CHARGING_STATION_Y
            
            # Fully recharge battery
            drone.battery_left = drone.battery_capacity
            
            # Switch back to idle state
            drone.is_going_to_charge = False
            drone.set_state('idle')
            
            # Publish event
            if drone.event_emitter:
                drone.event_emitter.emit('drone_recharged', {
                    'drone_id': drone.entity_id,
                    'position': (drone.x, drone.y),
                    'battery': drone.battery_left
                })
        else:
            # Move toward charging station
            move_ratio = drone.speed / distance
            dx = move_ratio * (Config.CHARGING_STATION_X - drone.x)
            dy = move_ratio * (Config.CHARGING_STATION_Y - drone.y)
            
            # Move drone
            drone.x += dx
            drone.y += dy
            
            # Consume battery
            step_distance = calculate_distance(0, 0, dx, dy)
            drone.consume_battery(step_distance)


class ChargingState(DroneState):
    """Drone is at charging station and charging"""
    
    def execute(self, drone: 'Drone') -> None:
        """Charge the drone"""
        # Drone is fully charged
        drone.battery_left = drone.battery_capacity
        
        # Return to idle state
        drone.set_state('idle')
        
        # Publish event
        if drone.event_emitter:
            drone.event_emitter.emit('drone_charged', {
                'drone_id': drone.entity_id,
                'position': (drone.x, drone.y),
                'battery': drone.battery_left
            })


# Dictionary mapping state names to state objects
DRONE_STATES = {
    'idle': IdleState(),
    'moving_to_pickup': MovingState(),
    'moving_to_bus_station_finish_pickup': MovingState(),
    'waiting_at_bus_station_for_pickup_bus': IdleState(),
    'moving_to_bus_station_for_delivery': MovingState(),
    'waiting_at_bus_station_for_delivery': IdleState(),
    'moving_to_delivery': MovingState(),
    'direct_transport_to_pickup': MovingState(),
    'direct_transport_to_delivery': MovingState(),
    'to_charging_station': ToChargingStationState(),
    'charging': ChargingState()
}


class Drone(Entity):
    """
    Drone entity for package transport
    """
    
    def __init__(self, entity_id: int, region: str, x: float, y: float, 
                 speed: float = Config.DRONE_SPEED,
                 event_emitter: Optional[EventEmitter] = None):
        """
        Initialize a drone
        
        Args:
            entity_id: Unique identifier for this drone
            region: Region identifier (e.g., 'pickup', 'delivery')
            x: Initial x coordinate
            y: Initial y coordinate
            speed: Movement speed per time unit
            event_emitter: Optional event emitter for publishing events
        """
        super().__init__(entity_id, x, y, speed)
        self.region = region
        
        # Task related attributes
        self.current_task: Optional[Dict[str, Any]] = None
        self.time_until_idle = 0
        self.delivery_tasks_completed = 0
        self.pickup_tasks_completed = 0
        self.bus_id = None
        
        # Battery related attributes
        self.battery_capacity = Config.DRONE_BATTERY_CAPACITY
        self.battery_left = Config.DRONE_BATTERY_CAPACITY
        
        # Charging related attributes
        self.is_going_to_charge = False
        self.old_status = None
        self.old_target_x = None
        self.old_target_y = None
        self.old_task = None
        
        # State management
        self.state = DRONE_STATES['idle']
        self.status = 'idle'  # Keep status string for backward compatibility
        
        # Event emitter
        self.event_emitter = event_emitter or global_event_emitter
    
    def set_state(self, state_name: str) -> None:
        """Set drone state"""
        if state_name in DRONE_STATES:
            self.state = DRONE_STATES[state_name]
            self.status = state_name  # Update status string for compatibility
            
            # Publish state change event
            if self.event_emitter:
                self.event_emitter.emit('drone_state_changed', {
                    'drone_id': self.entity_id,
                    'old_state': self.status,
                    'new_state': state_name,
                    'position': (self.x, self.y)
                })
        else:
            raise ValueError(f"Invalid drone state: {state_name}")
    
    def move(self) -> None:
        """Execute current state logic"""
        # If drone is idle or has time_until_idle=0, do nothing
        if self.status == 'idle' or self.time_until_idle <= 0:
            return
        
        # Determine if we need to go to charging station based on battery
        if not self.is_going_to_charge and self.should_recharge():
            self.go_to_charge()
            return
        
        # Execute current state
        self.state.execute(self)
    
    def consume_battery(self, distance: float) -> None:
        """Consume battery based on distance traveled"""
        self.battery_left -= distance
        if self.battery_left < 0:
            self.battery_left = 0
    
    def should_recharge(self) -> bool:
        """Check if drone should go to charging station"""
        # If no target is set, we don't need to recharge
        if self.target_x is None or self.target_y is None:
            return False
            
        # Calculate distance to target
        distance_to_target = calculate_distance(
            self.x, self.y, self.target_x, self.target_y
        )
        
        # If battery too low to reach target, or below threshold, go to charging
        return (distance_to_target > self.battery_left or 
                self.battery_left < Config.DRONE_LOW_BATTERY_THRESHOLD)
    
    def go_to_charge(self) -> None:
        """Interrupt current task and go to charging station"""
        # Save current state
        self.old_status = self.status
        self.old_target_x = self.target_x
        self.old_target_y = self.target_y
        self.old_task = self.current_task
        
        # Switch to charging mode
        self.is_going_to_charge = True
        self.set_state('to_charging_station')
        
        # Publish event
        if self.event_emitter:
            self.event_emitter.emit('drone_going_to_charge', {
                'drone_id': self.entity_id,
                'position': (self.x, self.y),
                'battery_left': self.battery_left
            })
    
    def on_arrival(self) -> None:
        """Handle arrival at target destination based on current state"""
        if self.status == 'moving_to_pickup':
            self._handle_arrival_at_pickup()
        elif self.status == 'moving_to_bus_station_finish_pickup':
            self._handle_arrival_at_bus_station_for_pickup()
        elif self.status == 'moving_to_bus_station_for_delivery':
            self._handle_arrival_at_bus_station_for_delivery()
        elif self.status == 'moving_to_delivery':
            self._handle_arrival_at_delivery()
        elif self.status == 'direct_transport_to_pickup':
            self._handle_direct_transport_pickup()
        elif self.status == 'direct_transport_to_delivery':
            self._handle_direct_transport_delivery()
    
    def _handle_arrival_at_pickup(self) -> None:
        """Handle arrival at pickup location"""
        # Update package status if we have task
        if self.current_task and 'package_id' in self.current_task:
            # Emit event for package pickup
            if self.event_emitter:
                self.event_emitter.emit('package_picked_up', {
                    'drone_id': self.entity_id,
                    'package_id': self.current_task['package_id'],
                    'position': (self.x, self.y)
                })
        
        # Get station rank from task
        station_rank = self.current_task.get('station_rank', 0) if self.current_task else 0
        
        # Now proceed to nearest bus station or directly to delivery
        # This is a simplified version - full implementation would involve
        # calculating the nearest bus stations and selecting one based on rank
        if self.current_task:
            # Move to bus station to finish pickup
            self.set_state('moving_to_bus_station_finish_pickup')
            
            # In a real implementation, you would lookup the nearest bus station
            # and set the target_x and target_y - this is mock code
            # that assumes target is already calculated
            if 'selected_station_x' in self.current_task and 'selected_station_y' in self.current_task:
                self.target_x = self.current_task['selected_station_x']
                self.target_y = self.current_task['selected_station_y']
                
                # Calculate time until idle
                distance_to_target = calculate_distance(
                    self.x, self.y, self.target_x, self.target_y
                )
                self.time_until_idle = math.ceil(distance_to_target / self.speed)
    
    def _handle_arrival_at_bus_station_for_pickup(self) -> None:
        """Handle arrival at bus station for pickup"""
        # Set status to waiting for bus
        self.set_state('waiting_at_bus_station_for_pickup_bus')
        
        if self.current_task:
            self.current_task['pickup_status'] = 'waiting_for_bus'
            
            # Emit event
            if self.event_emitter:
                self.event_emitter.emit('drone_waiting_for_bus', {
                    'drone_id': self.entity_id,
                    'task_id': self.current_task.get('index'),
                    'position': (self.x, self.y),
                    'line': self.current_task.get('line')
                })
    
    def _handle_arrival_at_bus_station_for_delivery(self) -> None:
        """Handle arrival at bus station for delivery"""
        # Check if delivery point is the same as current position
        if (self.current_task and 
            'delivery_x' in self.current_task and 
            'delivery_y' in self.current_task and
            self.current_task['delivery_x'] == self.x and 
            self.current_task['delivery_y'] == self.y):
            # Delivery complete at bus station
            self._complete_delivery()
        else:
            # Wait for bus to arrive with package
            self.set_state('waiting_at_bus_station_for_delivery')
            
            if self.current_task:
                self.current_task['delivery_status'] = 'waiting_for_bus'
                
                # Emit event
                if self.event_emitter:
                    self.event_emitter.emit('drone_waiting_for_delivery', {
                        'drone_id': self.entity_id,
                        'task_id': self.current_task.get('index'),
                        'position': (self.x, self.y),
                        'line': self.current_task.get('line')
                    })
    
    def _handle_arrival_at_delivery(self) -> None:
        """Handle arrival at final delivery location"""
        self._complete_delivery()
    
    def _handle_direct_transport_pickup(self) -> None:
        """Handle pickup for direct transport"""
        # Set state to moving to delivery
        self.set_state('direct_transport_to_delivery')
        
        if self.current_task:
            # Set target to delivery location
            self.target_x = self.current_task.get('delivery_x')
            self.target_y = self.current_task.get('delivery_y')
            
            # Calculate time until idle
            if self.target_x is not None and self.target_y is not None:
                distance_to_delivery = calculate_distance(
                    self.x, self.y, self.target_x, self.target_y
                )
                self.time_until_idle = math.ceil(distance_to_delivery / self.speed)
            
            # Emit event for package pickup
            if self.event_emitter and 'package_id' in self.current_task:
                self.event_emitter.emit('package_direct_pickup', {
                    'drone_id': self.entity_id,
                    'package_id': self.current_task['package_id'],
                    'position': (self.x, self.y)
                })
    
    def _handle_direct_transport_delivery(self) -> None:
        """Handle delivery for direct transport"""
        # Mark both pickup and delivery as completed
        if self.current_task:
            self.current_task['pickup_status'] = 'completed'
            self.current_task['delivery_status'] = 'completed'
        
        # Update completion counters
        self.pickup_tasks_completed += 1
        self.delivery_tasks_completed += 1
        
        # Emit event for package delivery
        if self.event_emitter and self.current_task and 'package_id' in self.current_task:
            self.event_emitter.emit('package_directly_delivered', {
                'drone_id': self.entity_id,
                'package_id': self.current_task['package_id'],
                'position': (self.x, self.y)
            })
        
        # Reset drone state
        self.set_state('idle')
        self.current_task = None
        self.target_x = None
        self.target_y = None
        self.bus_id = None
    
    def _complete_delivery(self) -> None:
        """Complete delivery task"""
        # Update task status
        if self.current_task:
            self.current_task['delivery_status'] = 'completed'
        
        # Update completion counter
        self.delivery_tasks_completed += 1
        
        # Emit event for package delivery
        if self.event_emitter and self.current_task and 'package_id' in self.current_task:
            self.event_emitter.emit('package_delivered', {
                'drone_id': self.entity_id,
                'package_id': self.current_task['package_id'],
                'position': (self.x, self.y)
            })
        
        # Reset drone state
        self.set_state('idle')
        self.current_task = None
        self.target_x = None
        self.target_y = None
        self.bus_id = None
    
    def assign_pickup_task(self, task: Dict[str, Any], target_x: float, target_y: float, 
                          line: str, station_rank: int = 0) -> None:
        """
        Assign pickup task to drone
        
        Args:
            task: Task object
            target_x: Target x coordinate
            target_y: Target y coordinate
            line: Bus line name
            station_rank: Station rank for picking nearest bus station
        """
        self.current_task = task
        self.set_state('moving_to_pickup')
        self.target_x = target_x
        self.target_y = target_y
        
        # Store line in task
        self.current_task['line'] = line
        
        # Calculate time until idle
        distance_to_target = calculate_distance(self.x, self.y, target_x, target_y)
        self.time_until_idle = math.ceil(distance_to_target / self.speed)
        
        # Store station rank
        self.current_task['station_rank'] = station_rank
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('drone_assigned_pickup', {
                'drone_id': self.entity_id,
                'task_id': task.get('index'),
                'package_id': task.get('package_id'),
                'target': (target_x, target_y),
                'line': line,
                'station_rank': station_rank
            })
    
    def assign_delivery_task_to_station(self, task: Dict[str, Any], target_x: float, 
                                       target_y: float, line: str) -> None:
        """
        Assign delivery task with target at bus station
        
        Args:
            task: Task object
            target_x: Target x coordinate of bus station
            target_y: Target y coordinate of bus station
            line: Bus line name
        """
        self.current_task = task
        self.set_state('moving_to_bus_station_for_delivery')
        self.target_x = target_x
        self.target_y = target_y
        
        # Store line in task
        self.current_task['line'] = line
        
        # Calculate time until idle
        distance_to_target = calculate_distance(self.x, self.y, target_x, target_y)
        self.time_until_idle = math.ceil(distance_to_target / self.speed)
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('drone_assigned_delivery_to_station', {
                'drone_id': self.entity_id,
                'task_id': task.get('index'),
                'package_id': task.get('package_id'),
                'target': (target_x, target_y),
                'line': line
            })
    
    def assign_delivery_task_to_delivery(self, task: Dict[str, Any], target_x: float, 
                                        target_y: float, line: str) -> None:
        """
        Assign delivery task with target at final delivery point
        
        Args:
            task: Task object
            target_x: Target x coordinate of delivery point
            target_y: Target y coordinate of delivery point
            line: Bus line name
        """
        self.current_task = task
        self.set_state('moving_to_delivery')
        self.target_x = target_x
        self.target_y = target_y
        
        # Store line in task
        self.current_task['line'] = line
        
        # Calculate time until idle
        distance_to_target = calculate_distance(self.x, self.y, target_x, target_y)
        self.time_until_idle = math.ceil(distance_to_target / self.speed)
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('drone_assigned_delivery', {
                'drone_id': self.entity_id,
                'task_id': task.get('index'),
                'package_id': task.get('package_id'),
                'target': (target_x, target_y),
                'line': line
            })
    
    def assign_direct_transport(self, task: Dict[str, Any], pickup_x: float, pickup_y: float, 
                               delivery_x: float, delivery_y: float) -> None:
        """
        Assign direct transport task from pickup to delivery
        
        Args:
            task: Task object
            pickup_x: Pickup x coordinate
            pickup_y: Pickup y coordinate
            delivery_x: Delivery x coordinate
            delivery_y: Delivery y coordinate
        """
        self.current_task = task
        self.set_state('direct_transport_to_pickup')
        self.target_x = pickup_x
        self.target_y = pickup_y
        
        # Store delivery coordinates in task
        self.current_task['delivery_x'] = delivery_x
        self.current_task['delivery_y'] = delivery_y
        
        # Calculate time until idle
        distance_to_pickup = calculate_distance(self.x, self.y, pickup_x, pickup_y)
        self.time_until_idle = math.ceil(distance_to_pickup / self.speed)
        
        # Emit event
        if self.event_emitter:
            self.event_emitter.emit('drone_assigned_direct_transport', {
                'drone_id': self.entity_id,
                'task_id': task.get('index'),
                'package_id': task.get('package_id'),
                'pickup': (pickup_x, pickup_y),
                'delivery': (delivery_x, delivery_y)
            })
    
    def get_state(self) -> Dict[str, Any]:
        """Get drone state as dictionary for logging/recording"""
        return {
            "id": self.entity_id,
            "x": self.x,
            "y": self.y,
            "status": self.status,
            "battery": self.battery_left,
            "task_id": self.current_task.get('index') if self.current_task else None,
            "pickup_tasks_completed": self.pickup_tasks_completed,
            "delivery_tasks_completed": self.delivery_tasks_completed
        }