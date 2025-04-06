import math
import numpy as np
from collections import deque

from .package import get_package_by_id
from utils.bus_utils import find_nearest_bus_station, find_nearest_bus_stations

class Drone:
    """
    Represents a delivery drone in the logistics system.
    """
    def __init__(self, drone_id, region, x, y, speed=5):
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

        package = get_package_by_id(task['package_id'], packages)
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
        package = get_package_by_id(task['package_id'], packages)
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
                self._on_arrival_logic()

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

    def _on_arrival_logic(self):
        """
        Logic to execute when the drone arrives at its target.
        """
        if self.status == 'moving_to_pickup':
            package = get_package_by_id(self.current_task['package_id'], packages)
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
                # 如果取货点就是所选车站，那就直接等