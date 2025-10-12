#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class TrajectoryGeneration(Node):
    def __init__(self):
        super().__init__('trajectory_generation')

        # Parameters
        self.velocity = 0.2  # meters per second
        self.frame_id = 'odom'

        # Subscribe to smoothed path with TRANSIENT_LOCAL
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.create_subscription(
            Float64MultiArray, 
            'smoothed_path', 
            self.smoothed_path_callback, 
            qos_profile
        )

        # Publisher for trajectory - also use TRANSIENT_LOCAL for latching
        self.trajectory_pub = self.create_publisher(
            Path, 
            'time_parameterized_trajectory', 
            qos_profile
        )
        
        # Publish only once
        self.published = False

    def smoothed_path_callback(self, msg):
        """Process smoothed path and publish trajectory ONCE"""
        if self.published:
            # Silently ignore republished paths to prevent controller reset
            return
            
        points = msg.data
        if len(points) % 2 != 0:
            self.get_logger().warning('Malformed smoothed_path data')
            return

        # Convert to list of tuples
        path_points = [(points[i], points[i + 1]) for i in range(0, len(points), 2)]

        # Generate trajectory
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        t = 0.0
        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id

            if i == 0:
                dt = 0.0
            else:
                dx = x - path_points[i - 1][0]
                dy = y - path_points[i - 1][1]
                dist = math.sqrt(dx * dx + dy * dy)
                dt = dist / self.velocity
            t += dt

            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        # Publish trajectory ONCE
        self.trajectory_pub.publish(path_msg)
        self.get_logger().info(f'✓ Published trajectory with {len(path_msg.poses)} points ONCE')
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()