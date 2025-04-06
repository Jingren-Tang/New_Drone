"""
Package class representing deliveries in the system
"""
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from ..utils.events import EventEmitter


@dataclass
class PackageState:
    """Base state for package status with allowed transitions"""
    name: str
    allowed_transitions: List[str] = field(default_factory=list)


# Define possible package states
PACKAGE_STATES = {
    'at_pickup_point': PackageState('at_pickup_point', 
                                   ['pickup_drone_on_the_way', 'direct_drone_on_the_way']),
    'pickup_drone_on_the_way': PackageState('pickup_drone_on_the_way', 
                                           ['on_pickup_drone']),
    'on_pickup_drone': PackageState('on_pickup_drone', 
                                   ['on_pickup_drone_waiting_for_bus']),
    'on_pickup_drone_waiting_for_bus': PackageState('on_pickup_drone_waiting_for_bus', 
                                                   ['on_bus']),
    'on_bus': PackageState('on_bus', 
                          ['on_bus_waiting_for_delivery_drone']),
    'on_bus_waiting_for_delivery_drone': PackageState('on_bus_waiting_for_delivery_drone', 
                                                     ['on_delivery_drone']),
    'on_delivery_drone': PackageState('on_delivery_drone', 
                                     ['delivered']),
    'delivered': PackageState('delivered', []),
    'direct_drone_on_the_way': PackageState('direct_drone_on_the_way', 
                                           ['on_direct_drone']),
    'on_direct_drone': PackageState('on_direct_drone', 
                                   ['directly_delivered']),
    'directly_delivered': PackageState('directly_delivered', [])
}


class Package:
    """
    Package entity representing a delivery in the system with state machine for status
    """
    
    def __init__(self, package_id: int, task_id: int, event_emitter: EventEmitter = None):
        """
        Initialize a package
        
        Args:
            package_id: Unique identifier for this package
            task_id: ID of the associated task
            event_emitter: Optional event emitter for publishing status changes
        """
        self.package_id = package_id
        self.task_id = task_id
        self.status = 'at_pickup_point'  # Initial status
        self.bus_id = None
        self.current_drone_id = None
        self.x = None  # Current x position, updated from carrier
        self.y = None  # Current y position, updated from carrier
        self.event_emitter = event_emitter
    
    def update_status(self, new_status: str) -> bool:
        """
        Update the package status following valid state transitions
        
        Args:
            new_status: New status to transition to
            
        Returns:
            bool: True if status was updated, False if invalid transition
        """
        if new_status not in PACKAGE_STATES:
            raise ValueError(f"Invalid package status: {new_status}")
            
        # Check if transition is allowed
        current_state = PACKAGE_STATES[self.status]
        if new_status not in current_state.allowed_transitions and new_status != self.status:
            return False
        
        # Store old status for event
        old_status = self.status
        
        # Update status
        self.status = new_status
        
        # Emit event if event emitter is available
        if self.event_emitter:
            self.event_emitter.emit('package_status_changed', {
                'package_id': self.package_id,
                'old_status': old_status,
                'new_status': new_status
            })
        
        return True
    
    def set_carrier(self, carrier_type: str, carrier_id: int) -> None:
        """Set the current carrier of the package"""
        if carrier_type == 'drone':
            self.current_drone_id = carrier_id
            self.bus_id = None
        elif carrier_type == 'bus':
            self.bus_id = carrier_id
            self.current_drone_id = None
    
    def update_position(self, x: float, y: float) -> None:
        """Update the package position"""
        self.x = x
        self.y = y
    
    def get_state(self) -> Dict[str, Any]:
        """Get package state as dictionary for logging/recording"""
        return {
            "package_id": self.package_id,
            "task_id": self.task_id,
            "status": self.status,
            "x": self.x,
            "y": self.y,
            "bus_id": self.bus_id,
            "drone_id": self.current_drone_id
        }