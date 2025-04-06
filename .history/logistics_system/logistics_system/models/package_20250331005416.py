class Package:
    """
    Represents a package in the logistics system.
    """
    def __init__(self, package_id, task_id):
        """
        Initialize a package.
        
        Args:
            package_id (int): Unique identifier for the package
            task_id (int): ID of the associated delivery task
        """
        self.package_id = package_id
        self.task_id = task_id  # Store only the task ID
        self.status = 'at_pickup_point'  # Initial status
        self.bus_id = None
        self.current_drone_id = None

    def update_status(self, new_status):
        """
        Update the status of the package.
        
        Args:
            new_status (str): New status to set for the package
        """
        print(f"[DEBUG] Package {self.package_id} status updated to {new_status}.")
        old_status = self.status
        self.status = new_status
        print(f"Package {self.package_id} status changed: {old_status} -> {new_status}")


# Helper functions to find packages and tasks
def get_task_by_id(task_id, tasks):
    """
    Find a task by its ID.
    
    Args:
        task_id (int): ID of the task to find
        tasks (list): List of tasks to search in
        
    Returns:
        dict or None: The task if found, None otherwise
    """
    return next((task for task in tasks if task['index'] == task_id), None)

def get_package_by_id(package_id, packages):
    """
    Find a package by its ID.
    
    Args:
        package_id (int): ID of the package to find
        packages (list): List of packages to search in
        
    Returns:
        Package or None: The package if found, None otherwise
    """
    return next((package for package in packages if package.package_id == package_id), None)