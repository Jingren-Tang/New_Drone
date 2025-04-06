class BusStation:
    """
    Bus station class representing a stop on a bus route.
    """
    def __init__(self, x, y, line):
        """
        Initialize a bus station.
        
        Args:
            x (float): X-coordinate of the station
            y (float): Y-coordinate of the station
            line (str): The bus line this station belongs to
        """
        self.x = x
        self.y = y
        self.line = line