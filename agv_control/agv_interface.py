import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt, pi
import pygame
import sys
import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class AGVInterface(Node):
    def __init__(self):
        super().__init__('agv_interface')
        self.path_pub = self.create_publisher(Path, 'trajectory', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.path = None
        self.current_waypoint_index = 0
        self.current_pose = None
        self.map_data = None
        self.pid = PID(Kp=1.5, Ki=0.01, Kd=0.3)  # Tuned for AGV
        self.linear_velocity = 0.3
        self.waypoint_threshold = 0.2
        self.actual_path = []
        self.waypoints = []
        self.mode = 'trajectory'

        # Pygame setup
        pygame.init()
        self.screen_width = 800
        self.screen_height = 600
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption('AGV Interface')
        self.bg_color = (255, 255, 255)
        self.grid_color = (200, 200, 200)
        self.waypoint_color = (0, 0, 255)
        self.trajectory_color = (0, 0, 0)
        self.agv_color = (255, 0, 0)
        self.actual_path_color = (0, 255, 0)
        self.obstacle_color = (100, 100, 100)
        self.point_radius = 5
        self.agv_radius = 10
        self.scale = 100
        self.clock = pygame.time.Clock()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.current_pose:
            self.actual_path.append((self.current_pose.position.x, self.current_pose.position.y))

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Received map data')

    def publish_trajectory(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish')
            return
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = (wp[0] - self.screen_width / 2) / self.scale
            pose.pose.position.y = (wp[1] - self.screen_height / 2) / self.scale
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path = path.poses
        self.current_waypoint_index = 0
        self.actual_path.clear()
        self.path_pub.publish(path)
        self.get_logger().info(f'Published trajectory with {len(self.waypoints)} waypoints')
        self.waypoints.clear()

    def control_loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN and self.mode == 'trajectory':
                if event.button == 1:
                    x, y = event.pos
                    self.waypoints.append((x, y))
                    self.get_logger().info(f'Added waypoint at ({x}, {y})')
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and self.mode == 'trajectory':
                    self.publish_trajectory()
                elif event.key == pygame.K_c:
                    self.waypoints.clear()
                    self.get_logger().info('Cleared waypoints')
                elif event.key == pygame.K_t:
                    self.mode = 'trajectory'
                    self.get_logger().info('Switched to trajectory mode')
                elif event.key == pygame.K_v:
                    self.mode = 'view'
                    self.get_logger().info('Switched to view mode')
                elif event.key == pygame.K_p:  # Fixed: lowercase
                    self.pid.Kp += 0.1
                    self.get_logger().info(f'Increased Kp to {self.pid.Kp}')
                elif event.key == pygame.K_p and pygame.key.get_mods() & pygame.KMOD_SHIFT:  # Shift+p
                    self.pid.Kp = max(0.1, self.pid.Kp - 0.1)
                    self.get_logger().info(f'Decreased Kp to {self.pid.Kp}')
                elif event.key == pygame.K_i:  # Fixed: lowercase
                    self.pid.Ki += 0.001
                    self.get_logger().info(f'Increased Ki to {self.pid.Ki}')
                elif event.key == pygame.K_i and pygame.key.get_mods() & pygame.KMOD_SHIFT:  # Shift+i
                    self.pid.Ki = max(0.0, self.pid.Ki - 0.001)
                    self.get_logger().info(f'Decreased Ki to {self.pid.Ki}')
                elif event.key == pygame.K_d:  # Fixed: lowercase
                    self.pid.Kd += 0.05
                    self.get_logger().info(f'Increased Kd to {self.pid.Kd}')
                elif event.key == pygame.K_d and pygame.key.get_mods() & pygame.KMOD_SHIFT:  # Shift+d
                    self.pid.Kd = max(0.0, self.pid.Kd - 0.05)
                    self.get_logger().info(f'Decreased Kd to {self.pid.Kd}')

        if self.path is None or self.current_pose is None:
            self.get_logger().warn('Waiting for path or pose data')
            return

        if self.current_waypoint_index >= len(self.path):
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Trajectory completed')
            return

        target_pose = self.path[self.current_waypoint_index].pose
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        target_x = target_pose.position.x
        target_y = target_pose.position.y

        distance = sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        if distance < self.waypoint_threshold:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.path):
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index - 1}')
            return

        desired_heading = atan2(target_y - current_y, target_x - current_x)
        current_heading = self.get_yaw(self.current_pose.orientation)
        error = desired_heading - current_heading
        error = (error + pi) % (2 * pi) - pi

        dt = 0.1
        steering_angle = self.pid.compute(error, dt)
        steering_angle = max(min(steering_angle, 0.7854), -0.7854)  # Match max_steering_angle

        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = steering_angle  # Ackermann plugin expects steering angle
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f'Steering: {steering_angle:.2f}, Velocity: {self.linear_velocity:.2f}')

        self.screen.fill(self.bg_color)

        # Draw grid
        for x in range(0, self.screen_width, 100):
            pygame.draw.line(self.screen, self.grid_color, (x, 0), (x, self.screen_height), 1)
        for y in range(0, self.screen_height, 100):
            pygame.draw.line(self.screen, self.grid_color, (0, y), (self.screen_width, y), 1)

        # Draw obstacles
        if self.map_data:
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y
            for i in range(height):
                for j in range(width):
                    if self.map_data.data[i * width + j] > 50:
                        x = (j * resolution + origin_x) * self.scale + self.screen_width / 2
                        y = (i * resolution + origin_y) * self.scale + self.screen_height / 2
                        size = resolution * self.scale
                        pygame.draw.rect(self.screen, self.obstacle_color, (x, y, size, size))

        # Draw planned trajectory
        if self.path:
            points = []
            for pose in self.path:
                x = pose.pose.position.x * self.scale + self.screen_width / 2
                y = pose.pose.position.y * self.scale + self.screen_height / 2
                points.append((x, y))
                pygame.draw.circle(self.screen, self.waypoint_color, (x, y), self.point_radius)
            if len(points) > 1:
                pygame.draw.lines(self.screen, self.trajectory_color, False, points, 2)

        # Draw waypoints in trajectory mode
        if self.mode == 'trajectory' and self.waypoints:
            if len(self.waypoints) > 1:
                pygame.draw.lines(self.screen, self.trajectory_color, False, self.waypoints, 2)
            for point in self.waypoints:
                pygame.draw.circle(self.screen, self.waypoint_color, point, self.point_radius)

        # Draw actual path
        if len(self.actual_path) > 1:
            actual_points = [(x * self.scale + self.screen_width / 2, y * self.scale + self.screen_height / 2) for x, y in self.actual_path]
            pygame.draw.lines(self.screen, self.actual_path_color, False, actual_points, 1)

        # Draw AGV
        if self.current_pose:
            agv_x = current_x * self.scale + self.screen_width / 2
            agv_y = current_y * self.scale + self.screen_height / 2
            pygame.draw.circle(self.screen, self.agv_color, (agv_x, agv_y), self.agv_radius)

        # Draw labels
        font = pygame.font.SysFont(None, 24)
        mode_text = font.render(f'Mode: {self.mode.capitalize()}', True, (0, 0, 0))
        pid_text = font.render(f'Kp={self.pid.Kp:.2f} Ki={self.pid.Ki:.3f} Kd={self.pid.Kd:.2f}', True, (0, 0, 0))
        self.screen.blit(mode_text, (10, 10))
        self.screen.blit(pid_text, (10, 40))

        pygame.display.flip()
        self.clock.tick(30)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = AGVInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()