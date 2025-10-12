#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import yaml
from scipy.interpolate import CubicSpline
import numpy as np

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')

        # Load waypoints from YAML file
        self.declare_parameter('waypoint_file', 'config/waypoints.yaml')
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        waypoints = self.load_waypoints(waypoint_file)

        # Extract x and y coordinates
        self.x_points = np.array([wp[0] for wp in waypoints])
        self.y_points = np.array([wp[1] for wp in waypoints])

        # Number of smoothed points
        self.num_points = 200

        # Publisher for smoothed path - use TRANSIENT_LOCAL for latching
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.smoothed_path_pub = self.create_publisher(
            Float64MultiArray, 
            'smoothed_path', 
            qos_profile
        )

        # Publish ONCE after short delay
        self.published = False
        self.publish_timer = self.create_timer(0.5, self.publish_once)

    def load_waypoints(self, filename):
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return data['waypoints']

    def smooth_path(self):
        t = np.linspace(0, 1, len(self.x_points))
        cs_x = CubicSpline(t, self.x_points)
        cs_y = CubicSpline(t, self.y_points)

        t_new = np.linspace(0, 1, self.num_points)
        x_smooth = cs_x(t_new)
        y_smooth = cs_y(t_new)

        return np.vstack((x_smooth, y_smooth))

    def publish_once(self):
        """Publish smoothed path ONCE at startup with latching"""
        if not self.published:
            smooth_coords = self.smooth_path()
            msg = Float64MultiArray()
            
            combined = []
            for i in range(smooth_coords.shape[1]):
                combined.append(float(smooth_coords[0, i]))
                combined.append(float(smooth_coords[1, i]))
                
            msg.data = combined
            
            # Publish multiple times initially to ensure delivery
            for _ in range(3):
                self.smoothed_path_pub.publish(msg)
                
            self.get_logger().info(f'✓ Published smoothed path with {self.num_points} points ONCE (latched)')
            self.published = True
            self.publish_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()