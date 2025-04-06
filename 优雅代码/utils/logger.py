"""
Logging utilities for simulation events and data collection
"""
import logging
import os
import json
from typing import Dict, Any, List, Optional
import pandas as pd


# Configure basic logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


class SimulationLogger:
    """Logger class for recording simulation events and data"""
    
    def __init__(self, log_dir: str = 'logs'):
        """
        Initialize the simulation logger
        
        Args:
            log_dir: Directory to store log files
        """
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        # Create logger
        self.logger = logging.getLogger('simulation')
        
        # Records for entities, packages, etc.
        self.simulation_records: List[Dict[str, Any]] = []
        self.bus_records: List[Dict[str, Any]] = []
        self.package_records: List[Dict[str, Any]] = []
        
        # Setup file handler
        file_handler = logging.FileHandler(os.path.join(log_dir, 'simulation.log'))
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(file_handler)
    
    def info(self, message: str) -> None:
        """Log an info message"""
        self.logger.info(message)
    
    def warning(self, message: str) -> None:
        """Log a warning message"""
        self.logger.warning(message)
    
    def error(self, message: str) -> None:
        """Log an error message"""
        self.logger.error(message)
    
    def debug(self, message: str) -> None:
        """Log a debug message"""
        self.logger.debug(message)
    
    def record_entity_state(self, time: int, entity_type: str, 
                           entity_id: int, state: Dict[str, Any]) -> None:
        """
        Record entity state to simulation records
        
        Args:
            time: Current simulation time
            entity_type: Type of entity ('drone' or 'bus')
            entity_id: Entity ID
            state: State data dictionary
        """
        record = {
            "time": time,
            "entity": entity_type,
            "id": entity_id,
            **state
        }
        self.simulation_records.append(record)
    
    def record_bus_state(self, time: int, bus_id: int, 
                        state: Dict[str, Any]) -> None:
        """
        Record bus state to bus records
        
        Args:
            time: Current simulation time
            bus_id: Bus ID
            state: State data dictionary
        """
        record = {
            "time": time,
            "entity": "bus",
            "id": bus_id,
            **state
        }
        self.bus_records.append(record)
    
    def record_package_state(self, time: int, package_id: int, 
                           state: Dict[str, Any]) -> None:
        """
        Record package state to package records
        
        Args:
            time: Current simulation time
            package_id: Package ID
            state: State data dictionary
        """
        record = {
            "time": time,
            "package_id": package_id,
            **state
        }
        self.package_records.append(record)
    
    def save_records(self, prefix: Optional[str] = None) -> None:
        """
        Save all records to CSV files
        
        Args:
            prefix: Optional prefix for filenames
        """
        prefix = prefix or ''
        
        # Convert records to DataFrames
        if self.simulation_records:
            df_simulation = pd.DataFrame(self.simulation_records)
            df_simulation.to_csv(os.path.join(self.log_dir, f'{prefix}simulation_records.csv'), index=False)
        
        if self.bus_records:
            df_bus = pd.DataFrame(self.bus_records)
            df_bus.to_csv(os.path.join(self.log_dir, f'{prefix}bus_records.csv'), index=False)
        
        if self.package_records:
            df_package = pd.DataFrame(self.package_records)
            df_package.to_csv(os.path.join(self.log_dir, f'{prefix}package_records.csv'), index=False)
    
    def get_simulation_df(self) -> pd.DataFrame:
        """Get simulation records as DataFrame"""
        return pd.DataFrame(self.simulation_records)
    
    def get_bus_df(self) -> pd.DataFrame:
        """Get bus records as DataFrame"""
        return pd.DataFrame(self.bus_records)
    
    def get_package_df(self) -> pd.DataFrame:
        """Get package records as DataFrame"""
        return pd.DataFrame(self.package_records)


# Create a global logger instance
simulation_logger = SimulationLogger()