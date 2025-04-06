import math

class Bus:
    """
    Represents a bus in the transportation system.
    """
    def __init__(self, bus_id, route, line, start_station_index, direction, speed=5, stop_time=3):
        """
        Initialize a bus.
        
        Args:
            bus_id (int): Unique identifier for the bus
            route (list): List of stations on the bus route
            line (str): The bus line this bus belongs to
            start_station_index (int): Starting station index in the route
            direction (int): Direction of travel (1 for forward, -1 for backward)
            speed (float, optional): Bus speed. Defaults to 5.
            stop_time (int, optional): Time to stop at each station. Defaults to 3.
        """
        self.bus_id = bus_id
        self.line = line 
        self.route = route
        self.station_index = start_station_index
        self.direction = direction  # 1 表示向前，-1 表示向后
        self.x = self.route[self.station_index]['x']
        self.y = self.route[self.station_index]['y']
        self.speed = speed
        self.time_at_station = 0
        self.stop_time = stop_time
        self.status = 'at_station'  # 初始状态为在站点
        self.capacity = 3  # 每辆巴士的容量限制
        self.current_package = None  # 巴士当前携带的包裹

    def move(self):
        """
        Move the bus along its route based on current status.
        """
        if self.status == 'at_station':
            # 等待时间逻辑
            self.time_at_station += 1
            if self.time_at_station >= self.stop_time:
                self.status = 'moving'
                self.time_at_station = 0
        elif self.status == 'moving':
            next_station_index = self.station_index + self.direction

            # 判断是否到达终点站
            if next_station_index < 0 or next_station_index >= len(self.route):
                # 到达终点站，反转方向
                self.direction *= -1
                next_station_index = self.station_index + self.direction

            # 计算到下一个站点的距离
            next_station = self.route[next_station_index]
            distance_to_next_station = math.sqrt(
                (self.x - next_station['x']) ** 2 + (self.y - next_station['y']) ** 2
            )

            if distance_to_next_station <= self.speed:
                # 到达下一个站点
                self.x = next_station['x']
                self.y = next_station['y']
                self.station_index = next_station_index
                self.status = 'at_station'
            else:
                # 向目标站点移动
                move_ratio = self.speed / distance_to_next_station
                self.x += move_ratio * (next_station['x'] - self.x)
                self.y += move_ratio * (next_station['y'] - self.y)

    def can_pickup_package(self):
        """
        Check if the bus can pick up a package.
        
        Returns:
            bool: True if the bus can pick up a package, False otherwise
        """
        return self.current_package is None

    def load_package(self, package):
        """
        Load a package onto the bus.
        
        Args:
            package: The package to load
        """
        if self.can_pickup_package():
            self.current_package = package
            package.update_status('on_bus')
            package.bus_id = self.bus_id
            print(f"Package {package.package_id} loaded onto Bus {self.bus_id}.")
        else:
            print(f"Bus {self.bus_id} is full, cannot load Package {package.package_id}.")

    def unload_package(self):
        """
        Unload the package from the bus.
        
        Returns:
            Package or None: The unloaded package, or None if there was no package
        """
        package = self.current_package
        self.current_package = None
        return package
    
    def check_direction(self, target_station):
        """
        Check if the bus direction is heading towards the target station.
        
        Args:
            target_station (tuple): Target station coordinates (x, y)
            
        Returns:
            bool: True if the bus will eventually reach the target station, False otherwise
        """
        next_station_index = self.station_index + self.direction

        # 检查当前方向是否合法，非法则自动反转方向
        if next_station_index < 0 or next_station_index >= len(self.route):
            return True  # 到达终点自动反方向行驶，目标有可能在新方向上

        # 模拟沿当前方向行驶路径，检查目标站点是否在路径上
        current_index = self.station_index
        while 0 <= current_index < len(self.route):
            # 如果找到目标站点，方向正确
            station = self.route[current_index]
            if (station['x'], station['y']) == target_station:
                return True
            current_index += self.direction

        # 如果未找到目标站点，方向错误
        return False
    
    def is_on_target_line(self, target_line):
        """
        Check if the bus is on the target line.
        
        Args:
            target_line (str): Name of the target line
            
        Returns:
            bool: True if the bus is on the target line, False otherwise
        """
        return self.line == target_line


class BusSystem:
    """
    Manages a fleet of buses and their operations.
    """
    def __init__(self):
        """
        Initialize the bus system.
        """
        self.tasks_transported = 0  # 统计巴士运输的次数
        self.buses = []  # 公交车列表
        self.bus_id_counter = 0  # 用来给每辆公交车分配唯一ID
        self.time = 0  # 系统的全局时间
        
    def initialize_buses(self, bus_lines, bus_stations_df):
        """
        Initialize buses for each line.
        
        Args:
            bus_lines (list): List of bus line names
            bus_stations_df (DataFrame): DataFrame of bus stations
        """
        for line in bus_lines:
            # 获取并排序线路站点
            route_stations = sorted(
                [station for station in bus_stations_df.to_dict('records') if station['line'] == line],
                key=lambda s: s['station_index']
            )
            # print(f"Initializing buses for {line}: {route_stations}")

            # 确保第一个和最后一个站点被正确初始化
            # 初始化巴士
            if len(route_stations) > 1:
                # 在第一站生成一个向前行驶的巴士
                self.buses.append(Bus(
                    bus_id=self.bus_id_counter,
                    line=line,
                    route=route_stations,
                    start_station_index=0,
                    direction=1
                ))
                self.bus_id_counter += 1

                # 最后一站生成一个向后行驶的巴士
                self.buses.append(Bus(
                    bus_id=self.bus_id_counter,
                    line=line,
                    route=route_stations,
                    start_station_index=len(route_stations) - 1,
                    direction=-1
                ))
                self.bus_id_counter += 1

    def update_buses(self):
        """
        Update the positions of all buses.
        """
        for bus in self.buses:
            bus.move()
            # print(f"Bus {bus.bus_id} at ({bus.x}, {bus.y}), Status: {bus.status}, Direction: {bus.direction}")

    def simulate(self, bus_lines, bus_stations_df):
        """
        Simulate bus operations, spawning new buses and updating positions.
        
        Args:
            bus_lines (list): List of bus line names
            bus_stations_df (DataFrame): DataFrame of bus stations
        """
        # 每隔50个时间单位添加新的公交车
        if self.time % 10 == 0 and self.time <= 50:
            # print(f"Starting new buses on all lines at time {self.time}.")
            self.initialize_buses(bus_lines, bus_stations_df)

        # 更新所有公交车的位置
        self.update_buses()
        self.time += 1  # 时间步长

    def check_buses_at_station_for_pickup(self, x, y, line=None, delivery_station=None):
        """
        Check if there's a bus at the specified location for pickup.
        
        Args:
            x (float): X-coordinate of the station
            y (float): Y-coordinate of the station
            line (str, optional): Line name to filter buses. Defaults to None.
            delivery_station (tuple, optional): Target delivery station coordinates (x, y). Defaults to None.
            
        Returns:
            Bus or None: The matching bus if found, None otherwise
        """
        for bus in self.buses:
            if bus.x == x and bus.y == y:
                # 如果提供了线路信息，先检查巴士是否在目标线路上
                if line is not None and not bus.is_on_target_line(line):
                    continue
                
                # 如果给定了目标送货站点，需要检查巴士方向
                if delivery_station is not None:
                    if not bus.check_direction(delivery_station):
                        continue
                
                # 如果通过以上检查，返回这辆巴士
                return bus
        
        return None

    def check_buses_at_station_for_delivery(self, x, y, bus_id, tolerance=0.00001):
        """
        Check if there's a specific bus at the specified location for delivery.
        
        Args:
            x (float): X-coordinate of the station
            y (float): Y-coordinate of the station
            bus_id (int): ID of the bus to match
            tolerance (float, optional): Tolerance for position matching. Defaults to 0.00001.
            
        Returns:
            Bus or None: The matching bus if found, None otherwise
        """
        for bus in self.buses:
            distance = math.sqrt((bus.x - x) ** 2 + (bus.y - y) ** 2)
            if distance <= tolerance and bus.bus_id == bus_id:
                return bus
        return None  # 未找到匹配的公交车