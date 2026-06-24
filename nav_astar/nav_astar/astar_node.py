import rclpy
from rclpy.node import Node
import numpy as np
import math
from .astar import astar, smooth

# Importações de mensagens padrão do ROS 2
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist

# NOTA: Você precisa importar suas funções astar e smooth do seu módulo de planejamento
# from seu_modulo_de_planejamento import astar, smooth

def wrap_to_pi(angle):
    """Garante que o ângulo esteja entre -pi e pi"""
    return math.atan2(math.sin(angle), math.cos(angle))

class AStarNode(Node):
    def __init__(self):
        super().__init__("astar_node")
        
        # Subscrições
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/astar_path", 10)

        # Estado interno
        self.has_map = False
        self.has_odom = False
        self.path = []
        self.current_waypoint = 0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Loop de controle a 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)

    def map_callback(self, msg):
        self.map_msg = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.grid = np.array(msg.data, dtype=np.int8).reshape(self.height, self.width)
        self.has_map = True

    def odom_callback(self, msg):
        # Atualiza a posição do robô
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Converte quaternion para yaw (Euler)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.has_odom = True

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return (row, col)

    def grid_to_world(self, row, col):
        x = (col * self.resolution) + self.origin_x
        y = (row * self.resolution) + self.origin_y
        return x, y

    def goal_callback(self, msg):
        if not self.has_map or not self.has_odom:
            self.get_logger().warn("Aguardando Mapa e Odometria...")
            return

        goal = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        start = self.world_to_grid(self.robot_x, self.robot_y)

        # Chamada às funções externas de pathfinding
        path, explored = astar(self.grid, start, goal)

        if path is None:
            self.get_logger().warn("Nenhum caminho encontrado.")
            return

        self.path = smooth(self.grid, path)
        self.current_waypoint = 0
        self.publish_path()

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg() # Boa prática no ROS 2

        for row, col in self.path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            x, y = self.grid_to_world(row, col)
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def control_loop(self):
        if len(self.path) == 0 or not self.has_odom:
            return

        if self.current_waypoint >= len(self.path):
            self.stop_robot()
            return

        row, col = self.path[self.current_waypoint]
        wx, wy = self.grid_to_world(row, col)

        dx = wx - self.robot_x
        dy = wy - self.robot_y
        distance = math.hypot(dx, dy)

        if distance < 0.20:
            self.current_waypoint += 1
            return

        desired_yaw = math.atan2(dy, dx)
        error = wrap_to_pi(desired_yaw - self.robot_yaw)

        cmd = Twist()
        cmd.angular.z = 2.0 * error
        # Só avança se o robô estiver relativamente alinhado com o alvo
        cmd.linear.x = 0.25 * max(0.0, math.cos(error))

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.path = [] # Limpa o caminho para o robô parar e esperar novo objetivo

def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()