import math
import numpy as np
from collections import deque

from .package import get_package_by_id
from utils.bus_utils import find_nearest_bus_station, find_nearest_bus_stations

class Drone:
    """
    Represents a delivery drone in the logistics system.
    """
    def __init__(self, drone_id, region, x, y, speed=5, packages=None):
        """
        Initialize a drone.
        
        Args:
            drone_id (int): Unique identifier for the drone
            region (str): Region the drone is responsible for
            x (float): Initial X-coordinate
            y (float): Initial Y-coordinate
            speed (float, optional): Drone speed. Defaults to 5.
        """
        self.drone_id = drone_id
        self.region = region  # 无人机负责的区域编号
        self.x = x
        self.y = y
        self.speed = speed
        self.packages = packages

        # --- 原有属性 ---
        self.status = 'idle'  
        self.current_task = None  
        self.target_x = None
        self.target_y = None
        self.time_until_idle = 0  
        self.delivery_tasks_completed = 0  
        self.pickup_tasks_completed = 0  
        self.bus_id = None  

        # --- 新增属性：电量相关 ---
        self.battery_capacity = 300.0  # 假设最大续航对应300米/单位距离
        self.battery_left = 300.0      # 初始电量充满
        self.charging_station_x = 35
        self.charging_station_y = 35

        # 标识是否正在去充电桩/充电，以及要不要回到之前的任务
        self.is_going_to_charge = False
        self.old_status = None
        self.old_target_x = None
        self.old_target_y = None
        self.old_task = None

    def assign_pickup_task(self, task, target_x, target_y, line, station_rank=0):
        """
        Assign a pickup task to the drone.
        
        Args:
            task (dict): Task to assign
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            line (str): Bus line for the task
            station_rank (int, optional): Rank of chosen station. Defaults to 0.
        """
        self.current_task = task
        self.status = 'moving_to_pickup'
        self.target_x = target_x
        self.target_y = target_y
        self.current_task['line'] = line  

        distance_to_target = np.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
        self.time_until_idle = int(np.ceil(distance_to_target / self.speed))

        if self.packages is not None:
            package = get_package_by_id(task['package_id'], self.packages)
            if package:
                package.update_status('pickup_drone_on_the_way')
                package.current_drone_id = self.drone_id
        
        # 记录选择的巴士站排名
        self.current_task['station_rank'] = station_rank

    def assign_delivery_task_to_station(self, task, target_x, target_y, line):
        """
        Assign a delivery task to the drone, setting the target to the nearest bus station.
        
        Args:
            task (dict): Task to assign
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            line (str): Bus line for the task
        """
        self.current_task = task
        self.status = 'moving_to_bus_station_for_delivery'
        self.target_x = target_x
        self.target_y = target_y
        self.current_task['line'] = line  

        distance_to_target = np.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
        self.time_until_idle = int(np.ceil(distance_to_target / self.speed))
    
    def assign_delivery_task_to_delivery(self, task, target_x, target_y, line):
        """
        Assign a delivery task to the drone, setting the target to the final delivery point.
        
        Args:
            task (dict): Task to assign
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            line (str): Bus line for the task
        """
        self.current_task = task
        self.status = 'moving_to_delivery'
        self.target_x = target_x
        self.target_y = target_y
        self.current_task['line'] = line  

        distance_to_target = np.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
        self.time_until_idle = int(np.ceil(distance_to_target / self.speed))

    def assign_direct_transport(self, task, pickup_x, pickup_y, delivery_x, delivery_y):
        """
        Assign a direct transport task to the drone.
        
        Args:
            task (dict): Task to assign
            pickup_x (float): Pickup X-coordinate
            pickup_y (float): Pickup Y-coordinate
            delivery_x (float): Delivery X-coordinate
            delivery_y (float): Delivery Y-coordinate
        """
        self.current_task = task
        self.status = 'direct_transport_to_pickup'
        self.target_x = pickup_x
        self.target_y = pickup_y

        distance_to_pickup = math.sqrt((self.x - pickup_x) ** 2 + (self.y - pickup_y) ** 2)
        self.time_until_idle = int(math.ceil(distance_to_pickup / self.speed))

        # 存储送货点坐标到任务中
        self.current_task['delivery_x'] = delivery_x
        self.current_task['delivery_y'] = delivery_y
        if self.packages is not None:
            package = get_package_by_id(task['package_id'], self.packages)
            if package:
                package.update_status('direct_drone_on_the_way')
                package.current_drone_id = self.drone_id  

    def move(self):
        """
        Move the drone based on its current status, with battery management.
        """
        # 如果无人机空闲、或剩余时间为 0，就不做任何移动
        if self.status == 'idle' or self.time_until_idle <= 0:
            return

        # 1) 检查是否正在去充电桩
        if self.is_going_to_charge:
            # 如果还没到达充电桩，就先飞往充电桩
            dist_to_station = math.sqrt((self.x - self.charging_station_x)**2 + (self.y - self.charging_station_y)**2)
            if dist_to_station <= self.speed:
                # 本次可直接到达充电桩
                self.x = self.charging_station_x
                self.y = self.charging_station_y

                # 模拟充电完成
                self.battery_left = self.battery_capacity
                self.is_going_to_charge = False
                self.status = "idle"

                print(f"Drone {self.drone_id} has arrived at charging station, battery recharged to full. Resuming task if any.")
                return  # 这一帧先退出，下一帧再继续执行正常任务逻辑
            else:
                # 距离充电桩还比较远，就走一小步
                move_ratio = self.speed / dist_to_station
                move_x = move_ratio * (self.charging_station_x - self.x)
                move_y = move_ratio * (self.charging_station_y - self.y)

                self.x += move_x
                self.y += move_y

                # 记得消耗电量
                distance_this_step = math.sqrt(move_x**2 + move_y**2)
                self.battery_left -= distance_this_step

                # 如果万一因为某些原因电量耗尽（不太可能，因为已经要回去了），也可以在这里做一下防御性判断
                if self.battery_left < 0:
                    self.battery_left = 0
                return

        # 2) 如果不在回充模式，就按原有逻辑移动，但要先检查电量是否足够走到目标
        distance_to_target = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        
        # -- 简易判断：如果"飞到目标"的距离 > 当前剩余电量，可以支持的距离
        if distance_to_target > self.battery_left or self.battery_left < 50:
            # 说明电量肯定不够飞到目标点，先去充电
            self.go_to_charge()
            return
        else:
            # 电量够，就飞过去（或飞一部分）
            step_distance = min(distance_to_target, self.speed)  # 单次 move() 只能飞 speed 距离
            if step_distance == 0:
                return

            move_ratio = step_distance / distance_to_target
            move_x = move_ratio * (self.target_x - self.x)
            move_y = move_ratio * (self.target_y - self.y)
            self.x += move_x
            self.y += move_y

            self.battery_left -= step_distance  # 按飞的距离扣电量
            self.time_until_idle = max(self.time_until_idle - 1, 0)

            # ------------------------------
            # 到这儿说明本次 move 飞行成功，继续执行原先的抵达逻辑
            # ------------------------------
            # 如果已经到达（可能恰好 step_distance == distance_to_target）
            if abs(self.x - self.target_x) < 1e-7 and abs(self.y - self.target_y) < 1e-7:
                self._on_arrival_logic(packages=self.packages, bus_stations=self.bus_stations, task=self.current_task)

    def go_to_charge(self):
        """
        Interrupt current task and switch to charging mode.
        """
        # 记录当前任务现场
        self.old_status = self.status
        self.old_target_x = self.target_x
        self.old_target_y = self.target_y
        self.old_task = self.current_task

        # 切换到回充模式
        self.is_going_to_charge = True
        self.status = 'to_charging_station'
        print(f"Drone {self.drone_id} battery not enough, going to charge...")

    def _on_arrival_logic(self, packages=None, bus_stations=None, task=None):
        """
        Logic to execute when the drone arrives at its target.
        """
        self.packages = packages
        self.bus_stations = bus_stations
        self.current_task = task

        if self.status == 'moving_to_pickup':
            if self.packages is not None:
                package = get_package_by_id(task['package_id'], self.packages)
                if package:
                    package.update_status('on_pickup_drone')
                    package.current_drone_id = self.drone_id  

            # 获取station_rank，如果没有则默认为0
            station_rank = self.current_task.get('station_rank', 0)
            
            # 获取多个最近的巴士站并选择指定排名的站点
            nearest_stations = find_nearest_bus_stations(
                self.current_task['pickup_x'],
                self.current_task['pickup_y'],
                bus_stations,
                self.current_task['line'],
                3  # 获取三个最近的站点
            )
            
            if station_rank >= len(nearest_stations):
                station_rank = len(nearest_stations) - 1
            
            selected_station = nearest_stations[station_rank]
            
            if (self.current_task['pickup_x'] == selected_station.x and 
                self.current_task['pickup_y'] == selected_station.y):
                # 如果取货点就是所选车站，那就直接等待巴士
                self.status = 'waiting_at_bus_station_for_pickup_bus'
                self.current_task['pickup_status'] = 'waiting_for_bus'
                package.update_status('on_pickup_drone_waiting_for_bus')
            else:
                # 否则，先飞到所选车站
                self.status = 'moving_to_bus_station_finish_pickup'
                self.target_x, self.target_y = selected_station.x, selected_station.y
                distance_to_target = np.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
                self.time_until_idle = int(np.ceil(distance_to_target / self.speed))
                
                print(f"Drone {self.drone_id} moving to selected station (rank {station_rank}) at ({selected_station.x}, {selected_station.y})")
    

        elif self.status == 'moving_to_bus_station_finish_pickup':
            # 如果取货点和送货点是同一个站，则直接完成
            nearest_pickup_bus_station = find_nearest_bus_station(
                self.current_task['pickup_x'],
                self.current_task['pickup_y'],
                bus_stations,
                self.current_task['line']
            )
            if (self.current_task['delivery_x'] == nearest_pickup_bus_station.x and 
                self.current_task['delivery_y'] == nearest_pickup_bus_station.y):
                # 直接完成
                self.status = 'idle'
                self.delivery_tasks_completed += 1
                self.current_task['pickup_status'] = 'completed'
                self.current_task['delivery_status'] = 'completed'
                if 'package_id' in self.current_task and self.current_task['package_id'] is not None:
                    package = get_package_by_id(self.current_task['package_id'], packages)
                    if package is not None:
                        package.update_status('delivered')
                        print(f"Package {package.package_id} has been delivered directly at the pickup point.")
                    else:
                        print(f"错误: 未找到 package_id 为 {self.current_task['package_id']} 的包裹。")

                self.current_task = None
                self.target_x = None
                self.target_y = None
            else:
                # 否则等待巴士
                self.status = 'waiting_at_bus_station_for_pickup_bus'
                self.current_task['pickup_status'] = 'waiting_for_bus'
                package = get_package_by_id(self.current_task['package_id'], packages)
                if package:
                    package.update_status('on_pickup_drone_waiting_for_bus')
                print(f"Drone {self.drone_id} is now waiting at the bus station ({self.x}, {self.y}) for the pickup bus.")

        elif self.status == 'moving_to_bus_station_for_delivery':
            # 如果送货点就是车站，那就直接完成
            nearest_bus_station = find_nearest_bus_station(
                self.current_task['delivery_x'],
                self.current_task['delivery_y'],
                bus_stations,
                self.current_task['line']
            )
            if (self.current_task['delivery_x'] == nearest_bus_station.x and 
                self.current_task['delivery_y'] == nearest_bus_station.y):
                self.status = 'idle'
                self.delivery_tasks_completed += 1
                self.current_task['delivery_status'] = 'completed'
                if 'package_id' in self.current_task and self.current_task['package_id'] is not None:
                    package = get_package_by_id(self.current_task['package_id'], packages)
                    if package:
                        package.update_status('delivered')
                        print(f"Package {package.package_id} has been delivered.")
                    else:
                        print(f"错误: 未找到 package_id 为 {self.current_task['package_id']} 的包裹。")

                self.current_task = None
                self.target_x = None
                self.target_y = None
            else:
                # 否则，等待巴士送到站
                self.current_task['delivery_status'] = 'waiting_for_bus'
                self.status = 'waiting_at_bus_station_for_delivery'
                package = get_package_by_id(self.current_task['package_id'], packages)
                if package:
                    package.update_status('on_bus_waiting_for_delivery_drone')

        elif self.status == 'moving_to_delivery':
            # 送货到最终点
            self.status = 'idle'
            self.delivery_tasks_completed += 1
            self.current_task['delivery_status'] = 'completed'
            if 'package_id' in self.current_task and self.current_task['package_id'] is not None:
                package = get_package_by_id(self.current_task['package_id'], packages)
                if package:
                    package.update_status('delivered')
                    print(f"Package {package.package_id} has been delivered.")
                else:
                    print(f"错误: 未找到 package_id 为 {self.current_task['package_id']} 的包裹。")

            self.current_task = None
            self.target_x = None
            self.target_y = None
            self.bus_id = None

        elif self.status == 'direct_transport_to_pickup':
            # 从取货点 -> 送货点的直接飞行
            self.status = 'direct_transport_to_delivery'
            self.target_x = self.current_task['delivery_x']
            self.target_y = self.current_task['delivery_y']
            distance_to_delivery = math.sqrt((self.x - self.target_x)**2 + (self.y - self.target_y)**2)
            self.time_until_idle = int(math.ceil(distance_to_delivery / self.speed))

            if 'package_id' in self.current_task and self.current_task['package_id'] is not None:
                package = get_package_by_id(self.current_task['package_id'], packages)
                if package:
                    package.update_status('on_direct_drone')
                    print(f"Package {package.package_id} picked up for direct transport by Drone {self.drone_id}")
                    package.current_drone_id = self.drone_id  

        elif self.status == 'direct_transport_to_delivery':
            # 无人机到达直接运输的送货点
            self.status = 'idle'
            self.delivery_tasks_completed += 1
            self.pickup_tasks_completed += 1
            self.current_task['pickup_status'] = 'completed'
            self.current_task['delivery_status'] = 'completed'

            if 'package_id' in self.current_task and self.current_task['package_id'] is not None:
                package = get_package_by_id(self.current_task['package_id'], packages)
                if package:
                    package.update_status('directly_delivered')
                    print(f"Package {package.package_id} delivered directly by Drone {self.drone_id}.")

            self.current_task = None
            self.target_x = None
            self.target_y = None
            self.bus_id = None
            print(f"Drone {self.drone_id} is now idle after completing direct transport.")


class DroneFleet:
    """
    Manages a fleet of delivery drones.
    """
    def __init__(self, initial_drone_count=1, bus_stations=None, bus_system=None, packages=None):
        """
        Initialize the drone fleet.
        
        Args:
            initial_drone_count (int, optional): Number of drones to start with. Defaults to 1.
            bus_stations (list, optional): List of bus station objects. Defaults to None.
            bus_system (BusSystem, optional): The bus system object. Defaults to None.
        """
        self.drones = [Drone(i, 'pickup', 35, 35) for i in range(initial_drone_count)]
        self.drone_count = initial_drone_count
        self.pickup_task_queue = deque()
        self.delivery_task_queue = deque()
        self.direct_task_queue = deque()
        # Store bus stations and bus system as class attributes
        self.bus_stations = bus_stations
        self.bus_system = bus_system
        self.packages = packages
        
    def get_idle_drone(self, target_x, target_y):
        """
        Find the nearest idle drone to a target location.
        
        Args:
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            
        Returns:
            Drone or None: The nearest idle drone if found, None otherwise
        """
        nearest_drone = None
        min_distance = float('inf')
        for drone in self.drones:
            if drone.status == 'idle':
                distance = self.calculate_distance(drone, target_x, target_y)
                if distance < min_distance:
                    min_distance = distance
                    nearest_drone = drone
        return nearest_drone
        
    def create_new_drone(self):
        """
        Create a new drone and add it to the fleet.
        
        Returns:
            Drone: The newly created drone
        """
        new_drone = Drone(self.drone_count, 'pickup', 35, 35) # 假设新无人机从(35,35)出发
        self.drones.append(new_drone)
        self.drone_count += 1
        print(f"Created new drone with ID {new_drone.drone_id}.")
        return new_drone
        
    def ensure_delivery_drone_availability(self, task, target_x, target_y):
        """
        Ensure a drone is available for delivery by creating one if needed.
        
        Args:
            task (dict): Delivery task
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
        """
        # 创建一个新的无人机
        new_drone = self.create_new_drone()
        # 找到最近的巴士站，并分配送货任务
        nearest_station = find_nearest_bus_station(target_x, target_y, self.bus_stations, task['line'])
        new_drone.assign_delivery_task_to_station(task, nearest_station.x, nearest_station.y, task['line'])
        
    def calculate_distance(self, drone, x, y):
        """
        Calculate the Euclidean distance between a drone and a point.
        
        Args:
            drone (Drone): The drone
            x (float): X-coordinate of the point
            y (float): Y-coordinate of the point
            
        Returns:
            float: The distance
        """
        return ((drone.x - x) ** 2 + (drone.y - y) ** 2) ** 0.5
        
    def try_allocate_direct_transport(self, task, pickup_x, pickup_y, delivery_x, delivery_y):
        """
        Try to allocate a drone for direct transport (pickup to delivery).
        
        Args:
            task (dict): Task to allocate
            pickup_x (float): Pickup X-coordinate
            pickup_y (float): Pickup Y-coordinate
            delivery_x (float): Delivery X-coordinate
            delivery_y (float): Delivery Y-coordinate
            
        Returns:
            bool: True if allocation was successful, False otherwise
        """
        nearest_drone = self.get_idle_drone(pickup_x, pickup_y)
        if nearest_drone is None:
            print(f"No idle drone available for direct transport task {task['pickup_task']}")
            return False
            
        # ---- 新增：先计算无人机能否完成整个路程 ----
        dist1 = math.sqrt((nearest_drone.x - pickup_x)**2 + (nearest_drone.y - pickup_y)**2)
        dist2 = math.sqrt((pickup_x - delivery_x)**2 + (pickup_y - delivery_y)**2)
        distance_needed = dist1 + dist2
        
        # 如果当前剩余电量不够，就先让它去充电，任务不分配
        if distance_needed > nearest_drone.battery_left:
            print(f"[INFO] Drone {nearest_drone.drone_id} does NOT have enough battery, need {distance_needed}, has {nearest_drone.battery_left}. Going to charge first.")
            nearest_drone.go_to_charge()
            return False
            
        # 分配直接运输任务
        nearest_drone.assign_direct_transport(task, pickup_x, pickup_y, delivery_x, delivery_y)
        task['pickup_status'] = 'assigned'
        task['delivery_status'] = 'assigned'
        
        if 'package_id' in task and task['package_id'] is not None:
            package = get_package_by_id(task['package_id'], packages)
            if package:
                package.update_status('on_direct_drone')
                print(f"Package {package.package_id} assigned for direct transport by Drone {nearest_drone.drone_id}")
                
        print(f"Successfully assigned direct transport task {task['pickup_task']} to Drone {nearest_drone.drone_id}")
        print(f"From: ({pickup_x}, {pickup_y}) To: ({delivery_x}, {delivery_y})")
        return True
        
    def try_allocate_delivery_task(self, task, target_x, target_y, auto_create_drone=False):
        """
        Try to allocate a drone for delivery task.
        
        Args:
            task (dict): Task to allocate
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            auto_create_drone (bool, optional): Whether to create a new drone if none available. Defaults to False.
            
        Returns:
            bool: True if allocation was successful, False otherwise
        """
        nearest_station = find_nearest_bus_station(target_x, target_y, self.bus_stations, task['line'])
        nearest_drone = self.get_idle_drone(nearest_station.x, nearest_station.y)
        
        # 检查是否找到了空闲无人机
        if nearest_drone is None:
            if auto_create_drone:
                # 如果允许自动创建无人机，确保有足够的无人机来处理任务
                self.ensure_delivery_drone_availability(task, target_x, target_y)
                return True
            else:
                # 没有可用无人机且不允许创建新无人机
                print(f"No available drone for delivery task {task['pickup_task']}.")
                return False
                
        # 如果送货点位于巴士站，则直接标记任务完成
        if nearest_station.x == target_x and nearest_station.y == target_y:
            task['delivery_status'] = 'completed'
            nearest_drone.delivery_tasks_completed += 1
            print(f"Delivery task {task['pickup_task']} completed directly at the bus station ({target_x}, {target_y}).")
            return True
            
        # ---------------- 电量检查 ----------------
        # 1) 无人机当前位置 -> nearest_station
        dist_to_station = math.sqrt(
            (nearest_drone.x - nearest_station.x)**2 + (nearest_drone.y - nearest_station.y)**2
        )
        
        # 2) nearest_station -> 最终送货点
        dist_to_delivery = math.sqrt(
            (nearest_station.x - target_x)**2 + (nearest_station.y - target_y)**2
        )
        
        total_needed = dist_to_station + dist_to_delivery
        if total_needed > nearest_drone.battery_left:
            print(f"[INFO] Drone {nearest_drone.drone_id} battery not enough for delivery task {task['pickup_task']}, "
                 f"need {total_needed:.1f}, have {nearest_drone.battery_left:.1f}. Going to charge first.")
            nearest_drone.go_to_charge()
            return False
            
        # 分配送货任务到无人机
        nearest_drone.assign_delivery_task_to_station(task, nearest_station.x, nearest_station.y, task['line'])
        task['delivery_status'] = 'assigned'
        print(f"Assigned delivery task {task['pickup_task']} to Drone {nearest_drone.drone_id} at nearest bus station ({nearest_station.x}, {nearest_station.y}).")
        return True
        
    def add_delivery_task_to_queue(self, task, target_x, target_y):
        """
        Add a delivery task to the waiting queue.
        
        Args:
            task (dict): Task to add
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
        """
        self.delivery_task_queue.append((task, target_x, target_y))
        print(f"Delivery task {task['pickup_task']} added to the queue.")
        
    def process_delivery_task_queue(self):
        """
        Try to allocate drones to tasks in the delivery queue.
        """
        for task, target_x, target_y in list(self.delivery_task_queue): # 遍历等待队列
            success = self.try_allocate_delivery_task(task, target_x, target_y)
            # 如果成功分配，移除出队列
            if success:
                self.delivery_task_queue.remove((task, target_x, target_y))
                print(f"Reassigned delivery task {task['delivery_task']} successfully.")
                
    def try_allocate_pickup_task(self, task, target_x, target_y, station_rank=0):
        """
        Try to allocate a drone for pickup task.
        
        Args:
            task (dict): Task to allocate
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            station_rank (int, optional): Rank of chosen station. Defaults to 0.
            
        Returns:
            bool: True if allocation was successful, False otherwise
        """
        # 获取多个最近的巴士站 - 使用self.bus_stations替代外部变量
        nearest_stations = find_nearest_bus_stations(target_x, target_y, self.bus_stations, task['line'], 3)
        
        # 确保station_rank不超出可用站点数量
        if station_rank >= len(nearest_stations):
            station_rank = len(nearest_stations) - 1
            print(f"Warning: Requested station rank {station_rank} exceeds available stations. Using rank {station_rank} instead.")
            
        # 选择指定排名的站点
        selected_station = nearest_stations[station_rank]
        nearest_drone = self.get_idle_drone(target_x, target_y)
        
        # 检查是否找到空闲无人机
        if nearest_drone is None:
            return False
            
        dist_to_pickup = math.sqrt((nearest_drone.x - target_x) ** 2 + (nearest_drone.y - target_y) ** 2)
        dist_to_station = math.sqrt((target_x - selected_station.x) ** 2 + (target_y - selected_station.y) ** 2)
        total_needed = dist_to_pickup + dist_to_station
        
        if total_needed > nearest_drone.battery_left:
            print(f"[INFO] Drone {nearest_drone.drone_id} battery not enough for pickup task {task['pickup_task']},"
                 f" need {total_needed:.1f}, have {nearest_drone.battery_left:.1f}. Going to charge first.")
            nearest_drone.go_to_charge()
            return False
            
        # 分配取货任务到无人机，传入station_rank参数
        nearest_drone.assign_pickup_task(task, target_x, target_y, task['line'], station_rank)
        task['pickup_status'] = 'assigned'
        
        # 记录选择的巴士站信息
        task['selected_station_x'] = selected_station.x
        task['selected_station_y'] = selected_station.y
        task['station_rank'] = station_rank
        
        print(f"Assigned pickup task {task['pickup_task']} to Drone {nearest_drone.drone_id} at ({target_x}, {target_y}).")
        print(f"Selected station rank: {station_rank}, coordinates: ({selected_station.x}, {selected_station.y})")
        return True
        
    def add_pickup_task_to_queue(self, task, target_x, target_y, station_rank=0):
        """
        Add a pickup task to the waiting queue.
        
        Args:
            task (dict): Task to add
            target_x (float): Target X-coordinate
            target_y (float): Target Y-coordinate
            station_rank (int, optional): Rank of chosen station. Defaults to 0.
        """
        self.pickup_task_queue.append((task, target_x, target_y, station_rank))
        print(f"Pickup task {task['pickup_task']} added to the queue with station rank {station_rank}.")
        
    def process_pickup_task_queue(self):
        """
        Try to allocate drones to tasks in the pickup queue.
        """
        for item in list(self.pickup_task_queue): # 遍历等待队列
            if len(item) == 4: # 新格式：包含station_rank
                task, target_x, target_y, station_rank = item
                success = self.try_allocate_pickup_task(task, target_x, target_y, station_rank)
            else: # 兼容旧格式
                task, target_x, target_y = item
                success = self.try_allocate_pickup_task(task, target_x, target_y)
                
            # 如果成功分配，移除出队列
            if success:
                self.pickup_task_queue.remove(item)
                print(f"Reassigned pickup task {task['pickup_task']} successfully.")
                
    def add_direct_task_to_queue(self, task, pickup_x, pickup_y, delivery_x, delivery_y):
        """
        Add a direct transport task to the queue.
        
        Args:
            task (dict): Task to add
            pickup_x (float): Pickup X-coordinate
            pickup_y (float): Pickup Y-coordinate
            delivery_x (float): Delivery X-coordinate
            delivery_y (float): Delivery Y-coordinate
        """
        self.direct_task_queue.append((task, pickup_x, pickup_y, delivery_x, delivery_y))
        print(f"Direct transport task {task['pickup_task']} added to the direct queue.")
        
    def process_direct_task_queue(self):
        """
        Try to allocate drones to tasks in the direct transport queue.
        """
        # 用 list(...) 遍历拷贝，可以在循环中安全地修改原队列
        for (task, pickup_x, pickup_y, delivery_x, delivery_y) in list(self.direct_task_queue):
            success = self.try_allocate_direct_transport(
                task,
                pickup_x,
                pickup_y,
                delivery_x,
                delivery_y
            )
            # 如果成功分配，移除出队列
            if success:
                self.direct_task_queue.remove((task, pickup_x, pickup_y, delivery_x, delivery_y))
                print(f"Reassigned direct transport task {task['pickup_task']} successfully.")
                
    def get_pickup_queue(self):
        """
        Get the current pickup task queue.
        
        Returns:
            list: The pickup task queue
        """
        return list(self.pickup_task_queue)
        
    def get_delivery_queue(self):
        """
        Get the current delivery task queue.
        
        Returns:
            list: The delivery task queue
        """
        return list(self.delivery_task_queue)
        
    def move_all_drones(self):
        """
        Move all drones in the fleet.
        """
        for drone in self.drones:
            drone.move()