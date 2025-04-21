import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt, pi, cos, sin
import pygame
import sys
import numpy as np
import time

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
        self.pid = PID(Kp=1.5, Ki=0.01, Kd=0.3)
        self.linear_velocity = 0.5
        self.waypoint_threshold = 0.2
        self.actual_path = []
        self.waypoints = []
        self.mode = 'trajectory'
        self.scale = 100
        self.view_offset_x = 0
        self.view_offset_y = 0
        self.show_grid = True
        self.show_help = False
        self.blink_state = True
        self.last_blink_time = time.time()

        # Pygame setup
        pygame.init()
        self.screen_width = 800
        self.screen_height = 600
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
        self.point_radius = 5
        self.agv_radius = 10
        self.clock = pygame.time.Clock()
        self.screen.fill(self.bg_color)
        pygame.display.flip()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.current_pose:
            self.actual_path.append((self.current_pose.position.x, self.current_pose.position.y))
            self.get_logger().info('Received valid odom data')
        else:
            self.get_logger().warn('Received invalid odom data')

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
            x = (wp[0] - self.screen_width / 2 - self.view_offset_x) / self.scale
            y = (wp[1] - self.screen_height / 2 - self.view_offset_y) / self.scale
            if abs(x) > 10 or abs(y) > 10:
                self.get_logger().warn(f'Skipping waypoint ({x}, {y}) out of bounds')
                continue
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        if not path.poses:
            self.get_logger().warn('No valid waypoints to publish')
            return
        self.path = path.poses
        self.current_waypoint_index = 0
        self.actual_path.clear()
        self.path_pub.publish(path)
        self.get_logger().info(f'Published trajectory with {len(self.path)} waypoints')
        self.waypoints.clear()

    def control_loop(self):
        try:
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
                        self.get_logger().info(f'Increased Kp to {self.pid.Kp}')
                    elif event.key == pygame.K_p and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.pid.Kp = max(0.1, self.pid.Kp - 0.1)
                        self.get_logger().info(f'Decreased Kp to {self.pid.Kp}')
                    elif event.key == pygame.K_i:
                        self.pid.Ki += 0.001
                        self.get_logger().info(f'Increased Ki to {self.pid.Ki}')
                    elif event.key == pygame.K_i and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.pid.Ki = max(0.0, self.pid.Ki - 0.001)
                        self.get_logger().info(f'Decreased Ki to {self.pid.Ki}')
                    elif event.key == pygame.K_d:
                        self.pid.Kd += 0.05
                        self.get_logger().info(f'Increased Kd to {self.pid.Kd}')
                    elif event.key == pygame.K_d and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.pid.Kd = max(0.0, self.pid.Kd - 0.05)
                        self.get_logger().info(f'Decreased Kd to {self.pid.Kd}')
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

            self.screen.fill(self.bg_color)

            # Draw axes
            try:
                origin_x = self.screen_width / 2 + self.view_offset_x
                origin_y = self.screen_height / 2 + self.view_offset_y
                pygame.draw.line(self.screen, self.axis_color, (origin_x, origin_y), (origin_x + 50, origin_y), 2)
                pygame.draw.line(self.screen, self.axis_color, (origin_x, origin_y), (origin_x, origin_y - 50), 2)
            except Exception:
                pass

            # Draw grid
            if self.show_grid:
                try:
                    grid_step = 100
                    for x in range(0, self.screen_width, grid_step):
                        pygame.draw.line(self.screen, self.grid_color, (x, 0), (x, self.screen_height), 1)
                    for y in range(0, self.screen_height, grid_step):
                        pygame.draw.line(self.screen, self.grid_color, (0, y), (self.screen_width, y), 1)
                except Exception:
                    pass

            # Draw obstacles
            if self.map_data:
                try:
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
                except Exception:
                    pass

            # Draw planned trajectory
            if self.path:
                try:
                    points = []
                    for pose in self.path:
                        x = pose.pose.position.x * self.scale + self.screen_width / 2 + self.view_offset_x
                        y = pose.pose.position.y * self.scale + self.screen_height / 2 + self.view_offset_y
                        if 0 <= x < self.screen_width and 0 <= y < self.screen_height:
                            points.append((x, y))
                            pygame.draw.circle(self.screen, self.waypoint_color, (x, y), self.point_radius)
                    if len(points) > 1:
                        pygame.draw.lines(self.screen, self.trajectory_color, False, points, 2)
                except Exception:
                    pass

            # Draw waypoints in trajectory mode
            if self.mode == 'trajectory' and self.waypoints:
                try:
                    if len(self.waypoints) > 1:
                        pygame.draw.lines(self.screen, self.trajectory_color, False, self.waypoints, 2)
                    for point in self.waypoints:
                        if 0 <= point[0] < self.screen_width and 0 <= point[1] < self.screen_height:
                            pygame.draw.circle(self.screen, self.waypoint_color, point, self.point_radius)
                except Exception:
                    pass

            # Draw actual path
            if len(self.actual_path) > 1:
                try:
                    actual_points = []
                    for x, y in self.actual_path:
                        px = x * self.scale + self.screen_width / 2 + self.view_offset_x
                        py = y * self.scale + self.screen_height / 2 + self.view_offset_y
                        if 0 <= px < self.screen_width and 0 <= py < self.screen_height:
                            actual_points.append((px, py))
                    if len(actual_points) > 1:
                        pygame.draw.lines(self.screen, self.actual_path_color, False, actual_points, 1)
                except Exception:
                    pass

            # Draw AGV
            if self.current_pose:
                try:
                    agv_x = self.current_pose.position.x * self.scale + self.screen_width / 2 + self.view_offset_x
                    agv_y = self.current_pose.position.y * self.scale + self.screen_height / 2 + self.view_offset_y
                    if 0 <= agv_x < self.screen_width and 0 <= agv_y < self.screen_height:
                        pygame.draw.circle(self.screen, self.agv_color, (agv_x, agv_y), self.agv_radius)
                        yaw = self.get_yaw(self.current_pose.orientation)
                        arrow_length = self.agv_radius * 2
                        arrow_end_x = agv_x + arrow_length * cos(yaw)
                        arrow_end_y = agv_y - arrow_length * sin(yaw)
                        pygame.draw.line(self.screen, self.agv_color, (agv_x, agv_y), (arrow_end_x, arrow_end_y), 3)
                except Exception:
                    pass
            else:
                try:
                    placeholder_x = self.screen_width / 2 + self.view_offset_x
                    placeholder_y = self.screen_height / 2 + self.view_offset_y
                    if self.blink_state:
                        pygame.draw.circle(self.screen, self.placeholder_color, (placeholder_x, placeholder_y), self.agv_radius, 2)
                except Exception:
                    pass

            # Draw labels and status
            try:
                font = pygame.font.SysFont(None, 24)
                texts = [
                    f'Mode: {self.mode.capitalize()}',
                    f'Kp={self.pid.Kp:.2f} Ki={self.pid.Ki:.3f} Kd={self.pid.Kd:.2f}',
                    f'Scale: {self.scale} px/m',
                    f'Waypoint: {self.current_waypoint_index}/{len(self.path) if self.path else 0}',
                    f'Pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})' if self.current_pose else 'Pose: N/A',
                    f'AGV: {"Spawned" if self.current_pose else "Not Spawned"}',
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
            except Exception:
                pass

            # Draw help overlay
            if self.show_help:
                try:
                    help_texts = [
                        "Controls:",
                        "t: Trajectory mode",
                        "v: View mode",
                        "s: Send trajectory",
                        "c: Clear waypoints",
                        "p/i/d: Increase PID gains",
                        "Shift+p/i/d: Decrease PID gains",
                        "+/-: Zoom in/out",
                        "Arrows: Pan view",
                        "r: Reset view",
                        "g: Toggle grid",
                        "h: Toggle help",
                        "a: Center on AGV",
                        "Left click: Add waypoint",
                        "Right click: Remove last waypoint",
                    ]
                    help_surface = pygame.Surface((300, 400), pygame.SRCALPHA)
                    help_surface.fill((255, 255, 255, 200))
                    for i, text in enumerate(help_texts):
                        text_surface = font.render(text, True, (0, 0, 0))
                        help_surface.blit(text_surface, (10, 10 + i * 24))
                    self.screen.blit(help_surface, (self.screen_width - 310, 10))
                except Exception:
                    pass

            pygame.display.flip()
            self.clock.tick(30)

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
            steering_angle = max(min(steering_angle, 0.9), -0.9)

            twist = Twist()
            twist.linear.x = self.linear_velocity
            twist.angular.z = steering_angle
            self.cmd_vel_pub.publish(twist)
            self.get_logger().debug(f'Steering: {steering_angle:.2f}, Velocity: {self.linear_velocity:.2f}')

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def get_yaw(self, q):
        try:
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
            yaw = atan2(siny_cosp, cosy_cosp)
            return yaw
        except Exception:
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = AGVInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()