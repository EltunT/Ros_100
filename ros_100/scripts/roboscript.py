import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math as math
class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.front_ranges = [0,0]
        self.left_ranges = [0,0]
        self.right_ranges = [0,0]
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 20)
        self.subscriber_front = self.create_subscription(LaserScan, '/lidar', self.front_lidar_callback, 10)
        self.subscriber_left = self.create_subscription(LaserScan, '/lidar', self.left_lidar_callback, 10)
        self.subscriber_right = self.create_subscription(LaserScan, '/lidar', self.right_lidar_callback, 10)
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Параметры для объезда препятствий
        self.safe_distance = 0.8
        self.angle_threshold = 45
        self.get_first_lidar_callback = False
        
        # Параметры движения
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.5
        self.goal_reached = False
        
        # Состояние робота
        self.current_pose = (0, 0, 0)  # x, y, theta
        self.target_pose = (5.5, 0)    # x, y - теперь цель впереди по оси X
        self.initial_pose_set = False
        
        # Состояния объезда препятствия
        self.obstacle_avoidance_state = 'NORMAL'  # NORMAL, TURN_90, MOVE_PARALLEL, TURN_BACK, RETURN_TO_PATH
        self.obstacle_avoidance_start_pose = None
        self.obstacle_avoidance_direction = 0  # 1 для правого объезда, -1 для левого
        self.parallel_movement_distance = 0.0
        self.initial_path_angle = 0.0

    def odom_callback(self, msg):
        """Обновление текущей позиции робота из одометрии"""
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        )
        if not self.initial_pose_set:
            self.initial_pose_set = True
            self.get_logger().info('Initial pose set from odometry')
        self.get_logger().info(f'Current pose: x={self.current_pose[0]:.2f}, y={self.current_pose[1]:.2f}, theta={self.current_pose[2]:.2f}')

    def quaternion_to_yaw(self, orientation):
        """Преобразование кватерниона в угол поворота вокруг оси Z"""
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # Вычисляем угол поворота вокруг оси Z
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def front_lidar_callback(self, msg):
        self.front_ranges = msg.ranges
        self.get_first_lidar_callback = True
        self.get_logger().info(f'Front lidar data received: min={min(msg.ranges):.2f}, max={max(msg.ranges):.2f}')
        
        # Инициализация начального положения робота
        if not self.initial_pose_set:
            self.current_pose = (0, 0, 0)  # Начальное положение
            self.initial_pose_set = True
            self.get_logger().info('Initial pose set to (0, 0, 0)')

    def left_lidar_callback(self, msg):
        self.left_ranges = msg.ranges
        self.get_logger().info(f'Left lidar data received: min={min(msg.ranges):.2f}, max={max(msg.ranges):.2f}')

    def right_lidar_callback(self, msg):
        self.right_ranges = msg.ranges
        self.get_logger().info(f'Right lidar data received: min={min(msg.ranges):.2f}, max={max(msg.ranges):.2f}')

    def check_obstacles(self):
        """Проверка наличия препятствий впереди робота"""
        if not self.get_first_lidar_callback:
            return False
            
        # Проверяем сектор впереди робота
        front_sector = self.front_ranges[180-self.angle_threshold:180+self.angle_threshold]
        min_distance = min(front_sector)
        
        if min_distance < self.safe_distance:
            # Определяем направление объезда
            left_sector = min(self.left_ranges)
            right_sector = min(self.right_ranges)
            self.obstacle_avoidance_direction = 1 if left_sector < right_sector else -1
            return True
        return False

    def check_side_obstacle(self):
        """Проверка препятствия сбоку при параллельном движении"""
        if not self.get_first_lidar_callback:
            return True
            
        # Проверяем соответствующий боковой лидар
        if self.obstacle_avoidance_direction == 1:  # Объезд справа
            side_ranges = self.right_ranges
        else:  # Объезд слева
            side_ranges = self.left_ranges
            
        side_ranges = [x for x in side_ranges if x != float('inf')]
        if not side_ranges:
            return True
            
        return min(side_ranges) < self.safe_distance

    def check_front_clear(self):
        """Проверка, свободен ли путь впереди"""
        if not self.get_first_lidar_callback:
            return False
            
        # Проверяем сектор впереди робота
        front_sector = self.front_ranges[180-self.angle_threshold:180+self.angle_threshold]
        return min(front_sector) > self.safe_distance * 1.5

    def calculate_angle_to_goal(self):
        """Вычисление угла до цели"""
        dx = self.target_pose[0] - self.current_pose[0]
        dy = self.target_pose[1] - self.current_pose[1]
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - self.current_pose[2]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff

    def control_loop(self):
        if not self.initial_pose_set:
            self.get_logger().info('Waiting for initial pose...')
            return
            
        cmd = Twist()
        
        if self.obstacle_avoidance_state == 'NORMAL':
            if self.check_obstacles():
                self.obstacle_avoidance_state = 'TURN_90'
                self.obstacle_avoidance_start_pose = self.current_pose
                self.initial_path_angle = self.calculate_angle_to_goal()
                self.get_logger().info('Obstacle detected, starting avoidance')
            else:
                # Движение к цели
                angle_diff = self.calculate_angle_to_goal()
                self.get_logger().info(f'Angle to goal: {angle_diff:.2f}')
                
                # Пропорциональное управление с порогом
                if abs(angle_diff) > 0.1:  # Если угол больше порога, сначала поворачиваем
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.max_angular_speed * np.sign(angle_diff)
                    self.get_logger().info('Turning to goal')
                else:  # Если угол маленький, двигаемся к цели
                    cmd.linear.x = self.max_linear_speed
                    cmd.angular.z = self.max_angular_speed * angle_diff
                    self.get_logger().info('Moving to goal')
                
                # Проверка достижения цели
                distance_to_goal = math.sqrt((self.target_pose[0] - self.current_pose[0])**2 + 
                                          (self.target_pose[1] - self.current_pose[1])**2)
                self.get_logger().info(f'Distance to goal: {distance_to_goal:.2f}')
                if distance_to_goal < 0.1:
                    self.goal_reached = True
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info('Goal reached!')
                    
        elif self.obstacle_avoidance_state == 'TURN_90':
            # Поворот на 90 градусов
            target_angle = self.initial_path_angle + (math.pi/2 * self.obstacle_avoidance_direction)
            angle_diff = target_angle - self.current_pose[2]
            
            if abs(angle_diff) < 0.1:
                self.obstacle_avoidance_state = 'MOVE_PARALLEL'
                self.parallel_movement_distance = 0.0
                self.get_logger().info('Turned 90 degrees, starting parallel movement')
            else:
                cmd.angular.z = self.max_angular_speed * np.sign(angle_diff)
                cmd.linear.x = 0.0
                
        elif self.obstacle_avoidance_state == 'MOVE_PARALLEL':
            # Проверяем, объехали ли мы препятствие
            if not self.check_side_obstacle() and self.check_front_clear():
                self.obstacle_avoidance_state = 'TURN_BACK'
                self.get_logger().info('Obstacle cleared, turning back')
            else:
                cmd.linear.x = self.max_linear_speed
                cmd.angular.z = 0.0
                
        elif self.obstacle_avoidance_state == 'TURN_BACK':
            # Поворот обратно к изначальному пути
            target_angle = self.initial_path_angle
            angle_diff = target_angle - self.current_pose[2]
            
            if abs(angle_diff) < 0.1:
                self.obstacle_avoidance_state = 'RETURN_TO_PATH'
                self.get_logger().info('Turned back to initial path')
            else:
                cmd.angular.z = self.max_angular_speed * np.sign(angle_diff)
                cmd.linear.x = 0.0
                
        elif self.obstacle_avoidance_state == 'RETURN_TO_PATH':
            # Проверяем, достигли ли мы изначальной траектории
            distance_to_path = abs(self.current_pose[1] - self.obstacle_avoidance_start_pose[1])
            if distance_to_path < 0.1 and self.check_front_clear():
                self.obstacle_avoidance_state = 'NORMAL'
                self.get_logger().info('Returned to normal navigation')
            else:
                # Двигаемся к изначальной траектории
                cmd.linear.x = self.max_linear_speed
                cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







def lidar_to_cartesian(ranges, angle_min = 0, angle_increment =  (3.1416 / 35)):
    points = []
    for i, distance in enumerate(ranges):
        # Пропускаем невалидные значения (inf, nan)
        if not math.isfinite(distance):
            continue
            
        # Вычисляем угол для текущего луча
        angle = angle_min + i * angle_increment
        
        # Переводим в декартовы координаты
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        points.append((x, y))
    
    return np.array(points)  # Возвращаем массив точек shape=(N, 2)


#
#
#
#


def hough_circle_single_radius(points, radius=0.3, delta_ab=0.05, threshold_if_lack=2, threshold=0.8):
    """
    Поиск центра окружности заданного радиуса методом Хафа.
    
    Параметры:
    - points: массив точек shape=(N, 2).
    - radius: известный радиус окружности.
    - delta_ab: шаг дискретизации для a и b (по умолчанию 0.01).
    - threshold_if_lack: минимальное абсолютное количество голосов для обнаружения окружности.
    - threshold: порог для пиков в аккумуляторе (от 0 до 1).
    
    Возвращает:
    - center: координаты центра (a, b) или None, если окружность не найдена.
    """
    if len(points) == 0 or radius <= 0:
        return None

    # Определяем границы аккумулятора
    x_min, y_min = np.min(points, axis=0) - radius
    x_max, y_max = np.max(points, axis=0) + radius

    # Сетка для центров (a, b)
    a_values = np.arange(x_min, x_max, delta_ab)
    b_values = np.arange(y_min, y_max, delta_ab)
    accumulator = np.zeros((len(a_values), len(b_values)))

    # Голосование
    for x, y in points:
        for i, a in enumerate(a_values):
            sqrt_part = radius**2 - (x - a)**2
            if sqrt_part >= 0:
                b1 = y + math.sqrt(sqrt_part)
                b2 = y - math.sqrt(sqrt_part)
                for b in [b1, b2]:
                    j = np.argmin(np.abs(b_values - b))
                    if 0 <= j < len(b_values):
                        accumulator[i, j] += 1

    # Проверка минимального порога голосов
    if np.max(accumulator) < threshold_if_lack:
        return None

    # Нормализация и поиск пиков
    max_accum = np.max(accumulator)
    accumulator_normalized = accumulator / max_accum
    peaks = np.where(accumulator_normalized >= threshold)
    
    if len(peaks[0]) == 0:
        return None

    # Собираем все пики и сортируем по убыванию значений в аккумуляторе
    peak_values = accumulator[peaks]
    sorted_indices = np.argsort(peak_values)[::-1]  # Сортировка по убыванию

    # Берем пик с максимальным значением
    best_idx = sorted_indices[0]
    i, j = peaks[0][best_idx], peaks[1][best_idx]
    
    return (a_values[i], b_values[j])