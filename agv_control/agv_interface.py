import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from math import atan2, sqrt, pi, cos, sin, isnan, isinf
import pygame
import sys
import time
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class AGVInterface(Node):
    def __init__(self):
        super().__init__('agv_interface')
        self.get_logger().info('Initializing AGVInterface node')

        # Declare parameters
        self.declare_parameter('odom_topic', '/agv/odom')
        self.declare_parameter('config_path', os.path.join(
            get_package_share_directory('agv_control'), 'config', 'config.yaml'))
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # Load configuration
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                self.pid = PID(
                    config.get('pid_kp', 1.5),
                    config.get('pid_ki', 0.0),
                    config.get('pid_kd', 0.3)
                )
                self.linear_velocity = config.get('linear_velocity', 5.0)
                self.lookahead_distance = config.get('lookahead_distance', 0.7)
                self.max_linear_vel = config.get('max_linear_vel', 7.0)
                self.max_angular_vel = config.get('max_angular_vel', 0.5)
                self.get_logger().info(f'Loaded config: {config}')
            else:
                self.get_logger().warn(f'Config file {config_path} not found; using defaults')
                self.pid = PID(Kp=1.5, Ki=0.0, Kd=0.3)
                self.linear_velocity = 5.0
                self.lookahead_distance = 0.7
                self.max_linear_vel = 7.0
                self.max_angular_vel = 0.5
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}; using defaults')
            self.pid = PID(Kp=1.5, Ki=0.0, Kd=0.3)
            self.linear_velocity = 5.0
            self.lookahead_distance = 0.7
            self.max_linear_vel = 7.0
            self.max_angular_vel = 0.5

        # ROS 2 setup
        self.path_pub = self.create_publisher(Path, '/agv/trajectory', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/agv/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/agv/map', self.map_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/agv/joint_states', self.joint_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Initialize state
        self.path = None
        self.current_pose = None
        self.map_data = None
        self.joint_states = None
        self.min_velocity = 0.5
        self.max_accel = 0.5
        self.prev_linear_vel = 0.0
        self.actual_path = []
        self.waypoints = []
        self.mode = 'trajectory'
        self.scale = 100
        self.view_offset_x = 0
        self.view_offset_y = 0
        self.show_grid = True
        self.show_help = False
        self.show_waypoints = True
        self.blink_state = True
        self.last_blink_time = time.time()
        self.last_odom_time = None
        self.odom_timeout = 5.0
        self.path_log_file = 'path_log.txt'
        self.screen = None

        # Pygame setup
        try:
            pygame.init()
            self.get_logger().info('Pygame initialized successfully')
            self.screen_width = 1200
            self.screen_height = 800
            self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
            pygame.display.set_caption('AGV Interface')
            self.bg_color = (255, 255, 255)
            self.grid_color = (200, 200, 200)
            self.axis_color = (0, 0, 0)
            self.waypoint_color = (0, 0, 255)
            self.trajectory_color = (0, 0, 0)
            self.agv_color = (255, 0, 0)
            self.actual_path_color = (0, 255, 0)
            self.obstacle_color = (100, 100, 100)
            self.placeholder_color = (150, 150, 150)
            self.text_bg_color = (255, 255, 255, 180)
            self.warning_color = (255, 100, 100, 200)
            self.waiting_color = (255, 165, 0, 200)
            self.point_radius = 5
            self.agv_radius = 10
            self.clock = pygame.time.Clock()
            self.screen.fill(self.bg_color)
            pygame.display.flip()
            self.get_logger().info('Pygame display set up successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Pygame: {e}')
            self.screen = None  # Continue without interface if Pygame fails

    def odom_callback(self, msg):
        try:
            self.current_pose = msg.pose.pose
            self.last_odom_time = time.time()
            if self.current_pose:
                self.actual_path.append((self.current_pose.position.x, self.current_pose.position.y))
                self.get_logger().info(f'Received /agv/odom: pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')
            else:
                self.get_logger().warn('Received invalid /agv/odom pose')
        except Exception as e:
            self.get_logger().error(f'Odom callback error: {e}')

    def joint_callback(self, msg):
        try:
            self.joint_states = msg
            self.get_logger().debug(f'Received /agv/joint_states: {msg.name}')
            if 'base_to_rear_left' not in msg.name or 'base_to_rear_right' not in msg.name:
                self.get_logger().warn('Missing drive joints in /agv/joint_states')
        except Exception as e:
            self.get_logger().error(f'Joint callback error: {e}')

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Received /agv/map data')

    def save_waypoints(self):
        try:
            with open('waypoints.txt', 'w') as f:
                for x, y in self.waypoints:
                    f.write(f'{x},{y}\n')
            self.get_logger().info('Saved waypoints to waypoints.txt')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')

    def load_waypoints(self):
        try:
            if os.path.exists('waypoints.txt'):
                with open('waypoints.txt', 'r') as f:
                    self.waypoints = []
                    for line in f:
                        x, y = map(float, line.strip().split(','))
                        self.waypoints.append((x, y))
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            else:
                self.get_logger().warn('No waypoints.txt found')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    def publish_trajectory(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish')
            return
        if not self.current_pose:
            self.get_logger().warn('No current pose; cannot publish trajectory')
            return
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        # Prepend current position
        current_pose = PoseStamped()
        current_pose.header.frame_id = 'odom'
        current_pose.header.stamp = path.header.stamp
        current_pose.pose.position.x = self.current_pose.position.x
        current_pose.pose.position.y = self.current_pose.position.y
        current_pose.pose.position.z = 0.0
        current_pose.pose.orientation = self.current_pose.orientation
        path.poses.append(current_pose)

        # Add waypoints
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path.header.stamp
            x = (wp[0] - self.screen_width / 2) / self.scale + self.view_offset_x / self.scale
            y = -(wp[1] - self.screen_height / 2) / self.scale + self.view_offset_y / self.scale
            if abs(x) > 10 or abs(y) > 10:
                self.get_logger().warn(f'Skipping waypoint ({x:.2f}, {y:.2f}) out of bounds')
                continue
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        if len(path.poses) < 2:
            self.get_logger().warn('Insufficient valid waypoints to publish')
            return
        self.path = path.poses
        self.actual_path.clear()
        self.path_pub.publish(path)
        self.get_logger().info(f'Published /agv/trajectory with {len(self.path)} points')
        self.waypoints.clear()

        # Log path
        try:
            with open(self.path_log_file, 'a') as f:
                f.write(f'\nTrajectory at {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
                for pose in self.path:
                    f.write(f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})\n')
        except Exception as e:
            self.get_logger().error(f'Failed to log path: {e}')

    def control_loop(self):
        try:
            if self.screen is None:
                self.get_logger().warn('Pygame interface not available; skipping display update')
                return

            # Update blink state
            current_time = time.time()
            if current_time - self.last_blink_time > 0.5:
                self.blink_state = not self.blink_state
                self.last_blink_time = current_time

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
                    elif event.button == 3 and self.waypoints:
                        self.waypoints.pop()
                        self.get_logger().info('Removed last waypoint')
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
                    elif event.key == pygame.K_p:
                        self.pid.Kp += 0.1
                        self.get_logger().info(f'Increased Kp to {self.pid.Kp:.2f}')
                    elif event.key == pygame.K_p and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.pid.Kp = max(0.1, self.pid.Kp - 0.1)
                        self.get_logger().info(f'Decreased Kp to {self.pid.Kp:.2f}')
                    elif event.key == pygame.K_i:
                        self.pid.Ki += 0.001
                        self.get_logger().info(f'Increased Ki to {self.pid.Ki:.3f}')
                    elif event.key == pygame.K_i and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.pid.Ki = max(0.0, self.pid.Ki - 0.001)
                        self.get_logger().info(f'Decreased Ki to {self.pid.Ki:.3f}')
                    elif event.key == pygame.K_d:
                        self.pid.Kd += 0.05
                        self.get_logger().info(f'Increased Kd to {self.pid.Kd:.2f}')
                    elif event.key == pygame.K_d and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.pid.Kd = max(0.0, self.pid.Kd - 0.05)
                        self.get_logger().info(f'Decreased Kd to {self.pid.Kd:.2f}')
                    elif event.key == pygame.K_u:
                        self.linear_velocity += 0.5
                        self.get_logger().info(f'Increased speed to {self.linear_velocity:.2f} m/s')
                    elif event.key == pygame.K_j:
                        self.linear_velocity = max(self.min_velocity, self.linear_velocity - 0.5)
                        self.get_logger().info(f'Decreased speed to {self.linear_velocity:.2f} m/s')
                    elif event.key == pygame.K_l:
                        self.lookahead_distance += 0.1
                        self.get_logger().info(f'Increased lookahead to {self.lookahead_distance:.2f} m')
                    elif event.key == pygame.K_k:
                        self.lookahead_distance = max(0.3, self.lookahead_distance - 0.1)
                        self.get_logger().info(f'Decreased lookahead to {self.lookahead_distance:.2f} m')
                    elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                        self.scale = min(200, self.scale + 10)
                        self.get_logger().info(f'Zoom in: scale={self.scale}')
                    elif event.key == pygame.K_MINUS:
                        self.scale = max(25, self.scale - 10)
                        self.get_logger().info(f'Zoom out: scale={self.scale}')
                    elif event.key == pygame.K_LEFT:
                        self.view_offset_x = max(-1000, self.view_offset_x - 25)
                        self.get_logger().info(f'Pan left: offset_x={self.view_offset_x}')
                    elif event.key == pygame.K_RIGHT:
                        self.view_offset_x = min(1000, self.view_offset_x + 25)
                        self.get_logger().info(f'Pan right: offset_x={self.view_offset_x}')
                    elif event.key == pygame.K_UP:
                        self.view_offset_y = max(-1000, self.view_offset_y - 25)
                        self.get_logger().info(f'Pan up: offset_y={self.view_offset_y}')
                    elif event.key == pygame.K_DOWN:
                        self.view_offset_y = min(1000, self.view_offset_y + 25)
                        self.get_logger().info(f'Pan down: offset_y={self.view_offset_y}')
                    elif event.key == pygame.K_r:
                        self.scale = 100
                        self.view_offset_x = 0
                        self.view_offset_y = 0
                        self.get_logger().info('Reset view: scale=100, offset=(0,0)')
                    elif event.key == pygame.K_g:
                        self.show_grid = not self.show_grid
                        self.get_logger().info(f'Grid visibility: {self.show_grid}')
                    elif event.key == pygame.K_h:
                        self.show_help = not self.show_help
                        self.get_logger().info(f'Help overlay: {self.show_help}')
                    elif event.key == pygame.K_a and self.current_pose:
                        self.view_offset_x = -self.current_pose.position.x * self.scale
                        self.view_offset_y = -self.current_pose.position.y * self.scale
                        self.get_logger().info(f'Centered view on AGV: offset=({self.view_offset_x}, {self.view_offset_y})')
                    elif event.key == pygame.K_w:
                        self.show_waypoints = not self.show_waypoints
                        self.get_logger().info(f'Waypoint visibility: {self.show_waypoints}')
                    elif event.key == pygame.K_k and self.mode == 'trajectory':
                        self.save_waypoints()
                    elif event.key == pygame.K_l and self.mode == 'trajectory':
                        self.load_waypoints()

            self.screen.fill(self.bg_color)

            # Draw axes
            origin_x = self.screen_width / 2 + self.view_offset_x
            origin_y = self.screen_height / 2 + self.view_offset_y
            pygame.draw.line(self.screen, self.axis_color, (origin_x, origin_y), (origin_x + 50, origin_y), 2)
            pygame.draw.line(self.screen, self.axis_color, (origin_x, origin_y), (origin_x, origin_y - 50), 2)

            # Draw grid
            if self.show_grid:
                grid_step = 100
                for x in range(0, self.screen_width, grid_step):
                    pygame.draw.line(self.screen, self.grid_color, (x, 0), (x, self.screen_height), 1)
                for y in range(0, self.screen_height, grid_step):
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
                            x = (j * resolution + origin_x) * self.scale + self.screen_width / 2 + self.view_offset_x
                            y = (i * resolution + origin_y) * self.scale + self.screen_height / 2 + self.view_offset_y
                            size = resolution * self.scale
                            if 0 <= x < self.screen_width and 0 <= y < self.screen_height:
                                pygame.draw.rect(self.screen, self.obstacle_color, (x, y, size, size))

            # Draw planned trajectory
            if self.path and self.show_waypoints:
                points = []
                for pose in self.path:
                    x = pose.pose.position.x * self.scale + self.screen_width / 2 + self.view_offset_x
                    y = pose.pose.position.y * self.scale + self.screen_height / 2 + self.view_offset_y
                    if 0 <= x < self.screen_width and 0 <= y < self.screen_height:
                        points.append((x, y))
                        pygame.draw.circle(self.screen, self.waypoint_color, (x, y), self.point_radius)
                if len(points) > 1:
                    pygame.draw.lines(self.screen, self.trajectory_color, False, points, 2)

            # Draw waypoints in trajectory mode
            if self.mode == 'trajectory' and self.waypoints and self.show_waypoints:
                if len(self.waypoints) > 1:
                    pygame.draw.lines(self.screen, self.trajectory_color, False, self.waypoints, 2)
                for point in self.waypoints:
                    if 0 <= point[0] < self.screen_width and 0 <= point[1] < self.screen_height:
                        pygame.draw.circle(self.screen, self.waypoint_color, point, self.point_radius)

            # Draw actual path
            if len(self.actual_path) > 1:
                actual_points = []
                for x, y in self.actual_path:
                    px = x * self.scale + self.screen_width / 2 + self.view_offset_x
                    py = y * self.scale + self.screen_height / 2 + self.view_offset_y
                    if 0 <= px < self.screen_width and 0 <= py < self.screen_height:
                        actual_points.append((px, py))
                if len(actual_points) > 1:
                    pygame.draw.lines(self.screen, self.actual_path_color, False, actual_points, 1)

            # Draw AGV
            if self.current_pose:
                agv_x = self.current_pose.position.x * self.scale + self.screen_width / 2 + self.view_offset_x
                agv_y = self.current_pose.position.y * self.scale + self.screen_height / 2 + self.view_offset_y
                if 0 <= agv_x < self.screen_width and 0 <= agv_y < self.screen_height:
                    pygame.draw.circle(self.screen, self.agv_color, (agv_x, agv_y), self.agv_radius)
                    yaw = self.get_yaw(self.current_pose.orientation)
                    arrow_length = self.agv_radius * 2
                    arrow_end_x = agv_x + arrow_length * cos(yaw)
                    arrow_end_y = agv_y - arrow_length * sin(yaw)
                    pygame.draw.line(self.screen, self.agv_color, (agv_x, agv_y), (arrow_end_x, arrow_end_y), 3)
            else:
                placeholder_x = self.screen_width / 2 + self.view_offset_x
                placeholder_y = self.screen_height / 2 + self.view_offset_y
                if self.blink_state:
                    pygame.draw.circle(self.screen, self.placeholder_color, (placeholder_x, placeholder_y), self.agv_radius, 2)

            # Draw labels and status
            font = pygame.font.SysFont(None, 24)
            odom_status = 'Spawned' if self.current_pose else (
                'Waiting for Odometry' if self.last_odom_time and (time.time() - self.last_odom_time < self.odom_timeout)
                else 'Not Spawned'
            )
            joint_status = f'Joints: {len(self.joint_states.name) if self.joint_states else 0}/4'
            texts = [
                f'Mode: {self.mode.capitalize()}',
                f'Kp={self.pid.Kp:.2f} Ki={self.pid.Ki:.3f} Kd={self.pid.Kd:.2f}',
                f'Speed: {self.linear_velocity:.2f} m/s',
                f'Lookahead: {self.lookahead_distance:.2f} m',
                f'Scale: {self.scale} px/m',
                f'Pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})' if self.current_pose else 'Pose: N/A',
                f'AGV: {odom_status}',
                f'{joint_status}',
                f'Last Odom: {time.strftime("%H:%M:%S", time.localtime(self.last_odom_time)) if self.last_odom_time else "N/A"}',
            ]
            text_y = 10
            for text in texts:
                text_surface = font.render(text, True, (0, 0, 0))
                pygame.draw.rect(self.screen, self.text_bg_color, (5, text_y - 2, text_surface.get_width() + 10, text_surface.get_height() + 4), border_radius=3)
                self.screen.blit(text_surface, (10, text_y))
                text_y += 30
            scale_line_start = (self.screen_width - 110, self.screen_height - 20)
            scale_line_end = (self.screen_width - 110 + self.scale, self.screen_height - 20)
            pygame.draw.line(self.screen, (0, 0, 0), scale_line_start, scale_line_end, 2)
            scale_text = font.render('1m', True, (0, 0, 0))
            self.screen.blit(scale_text, (self.screen_width - 110, self.screen_height - 40))

            # Draw warning
            if not self.current_pose:
                warning_texts = [
                    f"AGV {odom_status}!",
                    "Check:",
                    f"- Odom topic: {self.get_parameter('odom_topic').value}",
                    "- Plugin: libgazebo_ros_diff_drive.so",
                    "- Gazebo logs",
                    "- ROS topic: ros2 topic echo /agv/odom",
                    "- ROS/Gazebo versions",
                ]
                warning_surface = pygame.Surface((300, 200), pygame.SRCALPHA)
                warning_surface.fill(self.waiting_color if odom_status == 'Waiting for Odometry' else self.warning_color)
                for i, text in enumerate(warning_texts):
                    text_surface = font.render(text, True, (0, 0, 0))
                    warning_surface.blit(text_surface, (10, 10 + i * 24))
                self.screen.blit(warning_surface, (10, self.screen_height - 210))

            # Draw help
            if self.show_help:
                help_texts = [
                    "Controls:",
                    "t: Trajectory mode",
                    "v: View mode",
                    "s: Send trajectory",
                    "c: Clear waypoints",
                    "p/i/d: Adjust PID gains",
                    "Shift+p/i/d: Decrease PID",
                    "u/j: Speed ±0.5 m/s",
                    "l/k: Lookahead ±0.1 m",
                    "+/-: Zoom",
                    "Arrows: Pan",
                    "r: Reset view",
                    "g: Toggle grid",
                    "h: Toggle help",
                    "a: Center AGV",
                    "w: Toggle waypoints",
                    "k: Save waypoints",
                    "l: Load waypoints",
                    "Troubleshooting:",
                    f"- Check: ros2 topic echo {self.get_parameter('odom_topic').value}",
                    "- Verify plugin",
                    "- Check Gazebo logs",
                ]
                help_surface = pygame.Surface((300, 500), pygame.SRCALPHA)
                help_surface.fill((255, 255, 255, 200))
                for i, text in enumerate(help_texts):
                    text_surface = font.render(text, True, (0, 0, 0))
                    help_surface.blit(text_surface, (10, 10 + i * 24))
                self.screen.blit(help_surface, (self.screen_width - 310, 10))

            pygame.display.flip()
            self.clock.tick(60)

            # Pure pursuit controller
            if not self.current_pose:
                self.get_logger().warn('No pose data; check /agv/odom')
                return
            if not self.joint_states or len(self.joint_states.name) < 4:
                self.get_logger().warn(f'Missing joints; expected 4, got {len(self.joint_states.name) if self.joint_states else 0}')
            if self.path is None:
                self.get_logger().warn('Waiting for path')
                return

            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            current_yaw = self.get_yaw(self.current_pose.orientation)

            # Find closest point on path
            min_dist = float('inf')
            lookahead_point = None
            for i, pose in enumerate(self.path):
                px = pose.pose.position.x
                py = pose.pose.position.y
                dist = sqrt((px - current_x)**2 + (py - current_y)**2)
                if dist < min_dist:
                    min_dist = dist
                if dist >= self.lookahead_distance and min_dist <= dist:
                    lookahead_point = (px, py)
                    break

            if not lookahead_point:
                # Use last point if no lookahead point found
                lookahead_point = (self.path[-1].pose.position.x, self.path[-1].pose.position.y)
                dist = sqrt((lookahead_point[0] - current_x)**2 + (lookahead_point[1] - current_y)**2)
                if dist < 0.3:
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info('Reached end of path; stopping AGV')
                    self.path = None
                    return

            # Compute control commands
            lx, ly = lookahead_point
            desired_heading = atan2(ly - current_y, lx - current_x)
            error = desired_heading - current_yaw
            error = (error + pi) % (2 * pi) - pi  # Normalize to [-π, π]

            # PID control for angular velocity
            dt = 0.1
            try:
                steering_angle = self.pid.compute(error, dt)
                steering_angle = max(min(steering_angle, self.max_angular_vel), -self.max_angular_vel)
            except Exception as e:
                self.get_logger().error(f'PID error: {e}')
                return

            # Linear velocity with smooth scaling
            velocity_scale = 1.0 - pow(2.71828, -min_dist / self.lookahead_distance)
            scaled_velocity = self.linear_velocity * velocity_scale
            scaled_velocity = max(self.min_velocity, scaled_velocity)

            # Rate limit velocity
            twist = Twist()
            target_linear_vel = max(min(scaled_velocity, self.max_linear_vel), 0.0)
            twist.linear.x = max(min(target_linear_vel, self.prev_linear_vel + self.max_accel * dt), 
                               self.prev_linear_vel - self.max_accel * dt)
            twist.angular.z = steering_angle

            # Sanitize cmd_vel
            if any(isnan(v) or isinf(v) for v in [twist.linear.x, twist.angular.z]):
                self.get_logger().error(f'Invalid /agv/cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')
                twist = Twist()
            elif abs(twist.linear.x) > self.max_linear_vel or abs(twist.angular.z) > self.max_angular_vel:
                self.get_logger().warn(f'Clamping /agv/cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')
                twist.linear.x = max(min(twist.linear.x, self.max_linear_vel), -self.max_linear_vel)
                twist.angular.z = max(min(twist.angular.z, self.max_angular_vel), -self.max_angular_vel)

            self.cmd_vel_pub.publish(twist)
            self.prev_linear_vel = twist.linear.x
            self.get_logger().debug(f'Distance: {min_dist:.2f}m, Heading Error: {error:.2f}rad, Velocity: {twist.linear.x:.2f}m/s, Steering: {twist.angular.z:.2f}rad/s')

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def get_yaw(self, q):
        try:
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
            yaw = atan2(siny_cosp, cosy_cosp)
            if isnan(yaw) or isinf(yaw):
                self.get_logger().error(f'Invalid yaw: {yaw}')
                return 0.0
            return yaw
        except Exception:
            self.get_logger().error('Yaw calculation error')
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = AGVInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Node crashed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if pygame.get_init():
            pygame.quit()

if __name__ == '__main__':
    main()