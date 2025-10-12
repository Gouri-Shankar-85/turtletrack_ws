#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class ObstacleAwarePurePursuitController(Node):
    """
    Enhanced Pure Pursuit Controller with Obstacle Avoidance
    Uses laser scan data to detect and avoid obstacles
    """
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # AMR85 Robot Physical Parameters
        self.declare_parameter('wheelbase', 0.27)
        self.declare_parameter('track_width', 0.265)
        
        # Pure Pursuit Parameters - INCREASED for smoother following
        self.declare_parameter('lookahead_distance', 0.5)  # Increased from 0.35
        
        # Velocity Limits
        self.declare_parameter('target_speed', 0.15)
        self.declare_parameter('max_linear_velocity', 0.20)
        self.declare_parameter('min_linear_velocity', 0.05)  # Increased from 0.03
        self.declare_parameter('max_angular_velocity', 1.0)
        
        # Obstacle Avoidance Parameters - ADJUSTED for cylinders
        self.declare_parameter('obstacle_distance_threshold', 0.35)  # Reduced from 0.5
        self.declare_parameter('warning_distance_threshold', 0.6)    # Reduced from 0.8
        self.declare_parameter('side_clearance', 0.3)                # Reduced from 0.35
        self.declare_parameter('avoidance_gain', 0.8)                # Reduced from 1.5
        
        # Control Parameters
        self.declare_parameter('goal_tolerance', 0.15)  # Slightly increased
        self.declare_parameter('path_completion_threshold', 0.95)  # Increased from 0.92
        
        # NEW: Stuck detection parameters
        self.declare_parameter('stuck_velocity_threshold', 0.02)
        self.declare_parameter('stuck_time_threshold', 3.0)  # seconds
        
        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.path_threshold = self.get_parameter('path_completion_threshold').value
        
        # Obstacle avoidance parameters
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.warning_threshold = self.get_parameter('warning_distance_threshold').value
        self.side_clearance = self.get_parameter('side_clearance').value
        self.avoidance_gain = self.get_parameter('avoidance_gain').value
        
        # Stuck detection parameters
        self.stuck_vel_threshold = self.get_parameter('stuck_velocity_threshold').value
        self.stuck_time_threshold = self.get_parameter('stuck_time_threshold').value
        
        # State variables
        self.current_pose = None
        self.trajectory = []
        self.scan_data = None
        self.goal_reached = False
        self.trajectory_received = False
        self.obstacle_detected = False
        
        # Path tracking
        self.current_waypoint_index = 0
        
        # NEW: Stuck detection variables
        self.stuck_timer = 0.0
        self.last_position = None
        self.position_history = []
        self.max_history_length = 50  # 1 second of history at 50Hz
        
        # NEW: Dynamic lookahead adjustment
        self.base_lookahead = self.lookahead
        self.min_lookahead = 0.3
        self.max_lookahead = 0.8
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.trajectory_sub = self.create_subscription(
            Path, 'time_parameterized_trajectory',
            self.trajectory_callback, qos_profile
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop (50 Hz)
        self.dt = 0.02
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Obstacle-Aware Pure Pursuit Controller Initialized')
        self.get_logger().info(f'Lookahead: {self.lookahead}m | Speed: {self.target_speed} m/s')
        self.get_logger().info(f'Obstacle Detection: {self.obstacle_threshold}m')
        self.get_logger().info('=' * 70)
    
    def odom_callback(self, msg):
        """Store current robot pose"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation),
            'v': math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        }
        
        # Update position history for stuck detection
        self.position_history.append((self.current_pose['x'], self.current_pose['y']))
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
    
    def scan_callback(self, msg):
        """Store laser scan data"""
        self.scan_data = msg
    
    def trajectory_callback(self, msg):
        """Store trajectory - only first time"""
        if self.trajectory_received:
            return
            
        self.trajectory = []
        for pose in msg.poses:
            self.trajectory.append({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            })
        
        self.goal_reached = False
        self.trajectory_received = True
        self.current_waypoint_index = 0
        
        # Reset stuck detection
        self.stuck_timer = 0.0
        self.last_position = None
        self.position_history = []
        
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'✓ Trajectory: {len(self.trajectory)} waypoints')
        
        if len(self.trajectory) >= 2:
            start = self.trajectory[0]
            end = self.trajectory[-1]
            self.get_logger().info(f'  Start: ({start["x"]:.2f}, {start["y"]:.2f})')
            self.get_logger().info(f'  Goal:  ({end["x"]:.2f}, {end["y"]:.2f})')
        
        self.get_logger().info('=' * 70)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def distance(self, x1, y1, x2, y2):
        """Euclidean distance"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def normalize_angle(self, angle):
        """Normalize to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def check_if_stuck(self):
        """
        NEW: Detect if robot is stuck
        Returns True if robot hasn't moved significantly in the past few seconds
        """
        if not self.current_pose or len(self.position_history) < 25:  # Need at least 0.5s of data
            return False
        
        # Check movement in last second
        recent_positions = self.position_history[-50:] if len(self.position_history) >= 50 else self.position_history
        
        if len(recent_positions) < 2:
            return False
        
        # Calculate total distance traveled
        total_distance = 0.0
        for i in range(1, len(recent_positions)):
            total_distance += self.distance(
                recent_positions[i-1][0], recent_positions[i-1][1],
                recent_positions[i][0], recent_positions[i][1]
            )
        
        # If moved less than threshold, might be stuck
        movement_threshold = 0.05  # 5cm in 1 second
        is_stuck = total_distance < movement_threshold
        
        # Also check current velocity
        if self.current_pose['v'] < self.stuck_vel_threshold:
            if is_stuck:
                self.stuck_timer += self.dt
            else:
                self.stuck_timer = 0.0
        else:
            self.stuck_timer = 0.0
        
        return self.stuck_timer > self.stuck_time_threshold
    
    def check_obstacles(self):
        """
        Check for obstacles in front and to the sides
        Returns: (min_front_distance, avoidance_angle, obstacle_severity)
        """
        if not self.scan_data or not self.current_pose:
            return float('inf'), 0.0, 0
        
        ranges = np.array(self.scan_data.ranges)
        ranges[np.isinf(ranges)] = self.scan_data.range_max
        ranges[np.isnan(ranges)] = self.scan_data.range_max
        
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        
        min_front_dist = float('inf')
        
        # Repulsive force components
        repulsive_x = 0.0
        repulsive_y = 0.0
        
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            
            # Skip invalid readings
            if r < 0.05 or r > self.scan_data.range_max - 0.1:
                continue
            
            # Front sector (-30° to +30°)
            if -math.pi/6 <= angle <= math.pi/6:
                min_front_dist = min(min_front_dist, r)
                
                # Add repulsive force if too close
                if r < self.warning_threshold:
                    force_magnitude = self.avoidance_gain * (1.0/r - 1.0/self.warning_threshold)
                    repulsive_x += force_magnitude * math.cos(angle)
                    repulsive_y += force_magnitude * math.sin(angle)
            
            # Left sector (30° to 90°)
            elif math.pi/6 < angle <= math.pi/2:
                if r < self.side_clearance:
                    force_magnitude = self.avoidance_gain * (1.0/r - 1.0/self.side_clearance)
                    repulsive_x += force_magnitude * math.cos(angle)
                    repulsive_y += force_magnitude * math.sin(angle)
            
            # Right sector (-90° to -30°)
            elif -math.pi/2 <= angle < -math.pi/6:
                if r < self.side_clearance:
                    force_magnitude = self.avoidance_gain * (1.0/r - 1.0/self.side_clearance)
                    repulsive_x += force_magnitude * math.cos(angle)
                    repulsive_y += force_magnitude * math.sin(angle)
        
        # Calculate avoidance angle from repulsive forces
        avoidance_angle = math.atan2(repulsive_y, repulsive_x)
        
        # Determine obstacle severity
        # 0: Clear, 1: Warning, 2: Danger, 3: Emergency stop
        if min_front_dist < self.obstacle_threshold:
            severity = 3 if min_front_dist < 0.25 else 2
        elif min_front_dist < self.warning_threshold:
            severity = 1
        else:
            severity = 0
        
        return min_front_dist, avoidance_angle, severity
    
    def adjust_lookahead_distance(self, severity, angular_error):
        """
        NEW: Dynamically adjust lookahead based on situation
        """
        # Reduce lookahead when obstacles are close
        if severity >= 2:
            self.lookahead = self.min_lookahead
        elif severity == 1:
            self.lookahead = (self.min_lookahead + self.base_lookahead) / 2
        else:
            # Adjust based on turning requirement
            if abs(angular_error) > math.radians(30):
                self.lookahead = self.min_lookahead + 0.1
            else:
                self.lookahead = self.base_lookahead
        
        # Clamp
        self.lookahead = np.clip(self.lookahead, self.min_lookahead, self.max_lookahead)
    
    def find_lookahead_point(self):
        """Find lookahead point using forward-only search"""
        if not self.trajectory or not self.current_pose:
            return None, None
        
        current_x = self.current_pose['x']
        current_y = self.current_pose['y']
        
        search_start = self.current_waypoint_index
        
        # Update current waypoint to closest point ahead
        min_dist = float('inf')
        closest_idx = search_start
        
        # INCREASED search window for better tracking
        search_window = min(25, len(self.trajectory) - search_start)  # Increased from 15
        
        for i in range(search_start, min(search_start + search_window, len(self.trajectory))):
            wp = self.trajectory[i]
            dist = self.distance(current_x, current_y, wp['x'], wp['y'])
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx >= self.current_waypoint_index:
            self.current_waypoint_index = closest_idx
        
        # Find lookahead point
        cumulative_dist = 0.0
        
        for i in range(self.current_waypoint_index, len(self.trajectory) - 1):
            p1 = self.trajectory[i]
            p2 = self.trajectory[i + 1]
            
            segment_dist = self.distance(p1['x'], p1['y'], p2['x'], p2['y'])
            
            if cumulative_dist + segment_dist >= self.lookahead:
                remaining = self.lookahead - cumulative_dist
                ratio = remaining / segment_dist if segment_dist > 0 else 0
                
                lookahead_point = {
                    'x': p1['x'] + ratio * (p2['x'] - p1['x']),
                    'y': p1['y'] + ratio * (p2['y'] - p1['y'])
                }
                return lookahead_point, i
            
            cumulative_dist += segment_dist
        
        return self.trajectory[-1], len(self.trajectory) - 1
    
    def compute_steering_angle(self, target_point, avoidance_angle, severity):
        """
        Compute steering angle with obstacle avoidance
        """
        if not self.current_pose or not target_point:
            return 0.0
        
        # Vector from robot to target
        dx = target_point['x'] - self.current_pose['x']
        dy = target_point['y'] - self.current_pose['y']
        
        # Angle to target in global frame
        target_angle = math.atan2(dy, dx)
        
        # Base heading error
        alpha = self.normalize_angle(target_angle - self.current_pose['theta'])
        
        # MODIFIED: Reduced avoidance influence for cylinders
        if severity > 0:
            # Blend path-following and obstacle avoidance
            avoidance_weight = min(severity * 0.2, 0.6)  # Reduced from 0.3 and 0.8
            
            # Convert avoidance angle from robot frame to heading correction
            avoidance_correction = self.normalize_angle(avoidance_angle - math.pi)
            
            # Blend the two angles
            alpha = (1 - avoidance_weight) * alpha + avoidance_weight * avoidance_correction
        
        return alpha
    
    def compute_velocities(self, alpha, distance_to_goal, obstacle_distance, severity):
        """
        Compute linear and angular velocities with obstacle-aware speed control
        """
        # Emergency stop - ONLY if EXTREMELY close (about to collide)
        # Don't stop for cylinders that are on the path plan
        if severity >= 3 and obstacle_distance < 0.15:  # Very close to collision
            return 0.0, 0.0
        
        # Base linear velocity
        if distance_to_goal < 0.5:
            linear_vel = self.target_speed * (distance_to_goal / 0.5)
            linear_vel = max(linear_vel, self.min_linear_vel)
        else:
            linear_vel = self.target_speed
        
        # MODIFIED: Less aggressive speed reduction for obstacles
        if severity == 2:  # Danger
            linear_vel *= 0.5  # Increased from 0.3
        elif severity == 1:  # Warning
            speed_factor = (obstacle_distance - self.obstacle_threshold) / \
                          (self.warning_threshold - self.obstacle_threshold)
            linear_vel *= max(0.6, speed_factor)  # Increased from 0.4
        
        # Reduce speed when turning sharply
        if abs(alpha) > math.radians(25):
            linear_vel *= 0.7  # Increased from 0.6
        if abs(alpha) > math.radians(45):
            linear_vel *= 0.5  # Increased from 0.4
        
        # Angular velocity - REDUCED gain for smoother turns
        k_angular = 1.8  # Reduced from 2.2
        angular_vel = k_angular * alpha
        
        # Limit velocities
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        """Main control loop with obstacle avoidance"""
        if self.goal_reached or not self.trajectory or not self.current_pose:
            return
        
        # NEW: Check if stuck
        if self.check_if_stuck():
            self.get_logger().warning('⚠️ Robot appears stuck! Attempting recovery...')
            # Skip ahead in trajectory to try to find a clearer path
            self.current_waypoint_index = min(
                self.current_waypoint_index + 5,
                len(self.trajectory) - 1
            )
            self.stuck_timer = 0.0
            self.position_history = []
        
        # Check for obstacles
        obstacle_distance, avoidance_angle, severity = self.check_obstacles()
        
        # Check if goal reached
        goal = self.trajectory[-1]
        distance_to_goal = self.distance(
            self.current_pose['x'], self.current_pose['y'],
            goal['x'], goal['y']
        )
        
        progress_ratio = self.current_waypoint_index / len(self.trajectory)
        
        if distance_to_goal < self.goal_tolerance or progress_ratio > self.path_threshold:
            self.goal_reached = True
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('=' * 70)
            self.get_logger().info('🎯 GOAL REACHED!')
            self.get_logger().info(f'   Final error: {distance_to_goal*100:.1f} cm')
            self.get_logger().info(f'   Path completion: {progress_ratio*100:.1f}%')
            self.get_logger().info('=' * 70)
            return
        
        # Find lookahead point
        lookahead_point, lookahead_idx = self.find_lookahead_point()
        
        if lookahead_point is None:
            self.get_logger().warning('No lookahead point found')
            self.publish_velocity(0.0, 0.0)
            return
        
        # Compute steering angle with obstacle avoidance
        alpha = self.compute_steering_angle(lookahead_point, avoidance_angle, severity)
        
        # NEW: Adjust lookahead dynamically
        self.adjust_lookahead_distance(severity, alpha)
        
        # Compute velocities
        linear_vel, angular_vel = self.compute_velocities(
            alpha, distance_to_goal, obstacle_distance, severity
        )
        
        # Publish commands
        self.publish_velocity(linear_vel, angular_vel)
        
        # Calculate cross-track error
        closest_wp = self.trajectory[self.current_waypoint_index]
        cross_track_error = self.distance(
            self.current_pose['x'], self.current_pose['y'],
            closest_wp['x'], closest_wp['y']
        )
        
        # Status indicators
        obstacle_status = ['✓ Clear', '⚠ Warning', '⛔ Danger', '🛑 STOP'][severity]
        
        # Log
        progress = (self.current_waypoint_index / len(self.trajectory)) * 100
        
        self.get_logger().info(
            f'[{progress:5.1f}%] WP:{self.current_waypoint_index:3d}/{len(self.trajectory)} | '
            f'Goal:{distance_to_goal:.2f}m | CTE:{cross_track_error*100:5.1f}cm | '
            f'LA:{self.lookahead:.2f}m | '
            f'Obs:{obstacle_distance:.2f}m {obstacle_status} | '
            f'α:{math.degrees(alpha):+5.1f}° | v={linear_vel:.2f} ω={angular_vel:.2f}',
            throttle_duration_sec=0.3
        )
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAwarePurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user')
    finally:
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()