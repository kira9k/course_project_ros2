import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
import math
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        # Создание паблишера для топика '/goal_pose' 
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('Goal Publisher node initialized')

    def publish_goal(self, x, y, z, yaw):
        """
        Метод для публикации цели в топик '/goal_pose'.
        Аргументы:
            x,y,z: координаты целевой точки (в метрах)
            yaw: угол поворота (в радианах)
        """
        time.sleep(1.0) 
        goal_msg = PoseStamped() # Создание сообщения типа PoseStamped
        goal_msg.header.stamp = self.get_clock().now().to_msg() 
        # Установка временной метки
        goal_msg.header.frame_id = 'map' # Установка системы координат

        # Установка координат целевой точки
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z

        # Преборазование угла в кватернион
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

        # Публикация сообщения в топик '/goal_pose'
        self.publisher_.publish(goal_msg)
        self.get_logger().info(
            f'Published goal: x={x}, y={y}, z={z}, yaw={yaw} rad'
        )

def main():
    # Парсер для аргументов командной строки
    parser = argparse.ArgumentParser(description='Publish a goal pose for Nav2')
    parser.add_argument('--x', type=float, default=0.0, help='X coordinate of the goal')
    parser.add_argument('--y', type=float, default=0.0, help='Y coordinate of the goal')
    parser.add_argument('--z', type=float, default=0.0, help='Z coordinate of the goal (usually 0 for 2D)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw angle in radians')
    args = parser.parse_args()

    # Проверка диапазона угла
    if not (-2 * math.pi <= args.yaw <= 2 * math.pi):
        print('Error: Yaw angle should be between -2π and 2π radians!')
        return

    # Инициализация ROS2
    rclpy.init()

    # Создание экземпляра класса GoalPublisher
    node = GoalPublisher()
    node.publish_goal(args.x, args.y, args.z, args.yaw)

    
    rclpy.spin_once(node, timeout_sec=1.0)

    # Завершение работы
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()