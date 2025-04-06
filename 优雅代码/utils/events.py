"""
Event system for decoupled communication between components
"""
from typing import Dict, List, Callable, Any


class EventEmitter:
    """
    Event emitter for pub/sub communication between components
    """
    
    def __init__(self):
        """Initialize with empty listeners dictionary"""
        self.listeners: Dict[str, List[Callable]] = {}
    
    def subscribe(self, event_type: str, listener: Callable) -> None:
        """
        Subscribe a listener function to an event type
        
        Args:
            event_type: Type of event to listen for
            listener: Callback function to execute when event occurs
        """
        if event_type not in self.listeners:
            self.listeners[event_type] = []
        self.listeners[event_type].append(listener)
    
    def unsubscribe(self, event_type: str, listener: Callable) -> bool:
        """
        Remove a listener from an event type
        
        Args:
            event_type: Type of event
            listener: Callback function to remove
            
        Returns:
            bool: True if listener was removed, False if not found
        """
        if event_type in self.listeners and listener in self.listeners[event_type]:
            self.listeners[event_type].remove(listener)
            return True
        return False
    
    def emit(self, event_type: str, data: Any = None) -> None:
        """
        Emit an event to all subscribed listeners
        
        Args:
            event_type: Type of event to emit
            data: Data to pass to listeners
        """
        if event_type in self.listeners:
            for listener in self.listeners[event_type]:
                listener(data)
    
    def clear(self, event_type: str = None) -> None:
        """
        Clear all listeners or listeners for a specific event type
        
        Args:
            event_type: Optional event type to clear
        """
        if event_type:
            if event_type in self.listeners:
                self.listeners[event_type] = []
        else:
            self.listeners = {}


# Create a global event emitter instance
global_event_emitter = EventEmitter()