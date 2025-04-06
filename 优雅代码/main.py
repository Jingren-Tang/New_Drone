"""
Main entry point for logistics simulation
"""
import pandas as pd
import numpy as np
import argparse
import json
import os
import time
from typing import List, Dict, Any

from .simulation import Simulation
from .utils.events import global_event_emitter
from .utils.logger import simulation_logger
from .config.settings import Config


def load_tasks(file_path: str) -> List[Dict[str, Any]]:
    """
    Load tasks from JSON file
    
    Args:
        file_path: Path to JSON file with task data
        
    Returns:
        List[Dict[str, Any]]: List of task dictionaries
    """
    with open(file_path, 'r') as f:
        return json.load(f)


def load_bus_stations(file_path: str) -> pd.DataFrame:
    """
    Load bus stations from CSV file
    
    Args:
        file_path: Path to CSV file with bus station data
        
    Returns:
        pd.DataFrame: DataFrame with bus station data
    """
    return pd.read_csv(file_path)


def create_sample_tasks(num_tasks: int = 10) -> List[Dict[str, Any]]:
    """
    Create sample tasks for testing
    
    Args:
        num_tasks: Number of tasks to create
        
    Returns:
        List[Dict[str, Any]]: List of task dictionaries
    """
    tasks = []
    for i in range(num_tasks):
        # Random pickup and delivery locations
        pickup_x = np.random.uniform(0, 100)
        pickup_y = np.random.uniform(0, 100)
        delivery_x = np.random.uniform(0, 100)
        delivery_y = np.random.uniform(0, 100)
        
        # Random pickup time
        pickup_time = np.random.randint(0, 500)
        
        tasks.append({
            'index': i,
            'pickup_task': f'pickup_{i}',
            'delivery_task': f'delivery_{i}',
            'pickup_x': pickup_x,
            'pickup_y': pickup_y,
            'delivery_x': delivery_x,
            'delivery_y': delivery_y,
            'pickup_time': pickup_time
        })
    
    return tasks


def create_sample_bus_stations() -> pd.DataFrame:
    """
    Create sample bus stations for testing
    
    Returns:
        pd.DataFrame: DataFrame with bus station data
    """
    # Line 1: East-West line
    line1_x = np.linspace(10, 90, 9)
    line1_y = np.ones(9) * 30
    line1_stations = [
        {'x': x, 'y': y, 'line': 'Line 1', 'station_index': i}
        for i, (x, y) in enumerate(zip(line1_x, line1_y))
    ]
    
    # Line 2: North-South line
    line2_x = np.ones(9) * 50
    line2_y = np.linspace(10, 90, 9)
    line2_stations = [
        {'x': x, 'y': y, 'line': 'Line 2', 'station_index': i}
        for i, (x, y) in enumerate(zip(line2_x, line2_y))
    ]
    
    # Combine stations
    all_stations = line1_stations + line2_stations
    
    return pd.DataFrame(all_stations)


def save_sample_data(tasks: List[Dict[str, Any]], bus_stations: pd.DataFrame) -> None:
    """
    Save sample data to files
    
    Args:
        tasks: List of task dictionaries
        bus_stations: DataFrame with bus station data
    """
    # Create data directory if it doesn't exist
    os.makedirs('data', exist_ok=True)
    
    # Save tasks to JSON
    with open('data/sample_tasks.json', 'w') as f:
        json.dump(tasks, f, indent=2)
    
    # Save bus stations to CSV
    bus_stations.to_csv('data/sample_bus_stations.csv', index=False)
    
    print("Sample data saved to 'data' directory")


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Logistics Simulation")
    
    parser.add_argument(
        "--tasks", 
        type=str, 
        help="Path to task JSON file"
    )
    
    parser.add_argument(
        "--stations", 
        type=str, 
        help="Path to bus stations CSV file"
    )
    
    parser.add_argument(
        "--generate-samples",
        action="store_true",
        help="Generate sample data instead of running simulation"
    )
    
    parser.add_argument(
        "--sample-tasks",
        type=int,
        default=20,
        help="Number of sample tasks to generate"
    )
    
    parser.add_argument(
        "--simulation-time",
        type=int,
        default=Config.SIMULATION_TIME,
        help="Total simulation time"
    )
    
    parser.add_argument(
        "--station-rank",
        type=int,
        default=0,
        help="Default station rank for optimization"
    )
    
    parser.add_argument(
        "--drone-count",
        type=int,
        default=Config.DRONE_INITIAL_COUNT,
        help="Initial number of drones"
    )
    
    parser.add_argument(
        "--battery-capacity",
        type=float,
        default=Config.DRONE_BATTERY_CAPACITY,
        help="Drone battery capacity"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default="simulation_output",
        help="Directory for output files"
    )
    
    return parser.parse_args()


def update_config(args):
    """Update configuration with command line arguments"""
    Config.SIMULATION_TIME = args.simulation_time
    Config.DRONE_INITIAL_COUNT = args.drone_count
    Config.DRONE_BATTERY_CAPACITY = args.battery_capacity
    
    # Configure logger
    os.makedirs(args.output_dir, exist_ok=True)
    simulation_logger.log_dir = args.output_dir


def run_simulation(tasks, bus_stations_df, station_rank, output_dir):
    """Run the simulation with the given inputs"""
    print(f"Starting simulation with {len(tasks)} tasks")
    
    # Create simulation
    simulation = Simulation(
        task_list=tasks,
        bus_stations_df=bus_stations_df,
        station_rank=station_rank,
        total_time=Config.SIMULATION_TIME,
        event_emitter=global_event_emitter,
        logger=simulation_logger
    )
    
    # Run simulation
    start_time = time.time()
    simulation.run_simulation()
    end_time = time.time()
    
    # Print execution time
    execution_time = end_time - start_time
    print(f"Simulation completed in {execution_time:.2f} seconds")
    
    # Return statistics
    return simulation.calculate_statistics()


def main():
    """Main entry point"""
    args = parse_args()
    
    # Update configuration
    update_config(args)
    
    # Generate sample data if requested
    if args.generate_samples:
        tasks = create_sample_tasks(args.sample_tasks)
        bus_stations = create_sample_bus_stations()
        save_sample_data(tasks, bus_stations)
        return
    
    # Load tasks and bus stations
    tasks_path = args.tasks or 'data/sample_tasks.json'
    stations_path = args.stations or 'data/sample_bus_stations.csv'
    
    # Check if files exist
    if not os.path.exists(tasks_path):
        if args.tasks:
            print(f"Error: Task file '{tasks_path}' not found")
            return
        else:
            print("No tasks file found. Generating sample data...")
            tasks = create_sample_tasks(args.sample_tasks)
            bus_stations = create_sample_bus_stations()
            save_sample_data(tasks, bus_stations)
    else:
        # Load data
        try:
            tasks = load_tasks(tasks_path)
            bus_stations_df = load_bus_stations(stations_path)
            
            # Run simulation
            stats = run_simulation(tasks, bus_stations_df, args.station_rank, args.output_dir)
            
            # Save statistics
            stats_path = os.path.join(args.output_dir, 'simulation_stats.json')
            with open(stats_path, 'w') as f:
                json.dump(stats, f, indent=2)
                
            print(f"Statistics saved to {stats_path}")
            
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()