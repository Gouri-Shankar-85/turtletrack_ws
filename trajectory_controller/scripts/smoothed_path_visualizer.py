#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class SmoothedPathVisualizer(Node):
    def __init__(self):
        super().__init__('smoothed_path_visualizer')

        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Subscribe to smoothed path with TRANSIENT_LOCAL to get latched messages
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.sub = self.create_subscription(
            Float64MultiArray, 
            'smoothed_path', 
            self.callback, 
            qos_profile
        )
        
        # Publisher for visualization - keeps republishing for RViz
        self.pub = self.create_publisher(Path, 'smoothed_path_viz', 10)
        
        # Store the path once received
        self.stored_path = None
        
        # Timer to republish at 2Hz for RViz (visualization only, doesn't affect controller)
        self.viz_timer = self.create_timer(0.5, self.republish_for_rviz)

    def callback(self, msg):
        """Store path when received"""
        points = msg.data
        if len(points) % 2 != 0:
            self.get_logger().warning('Received malformed Float64MultiArray data')
            return
            
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(points), 2):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = points[i]
            pose.pose.position.y = points[i + 1]
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.stored_path = path_msg
        self.pub.publish(path_msg)
        self.get_logger().info(f'✓ Stored path with {len(path_msg.poses)} poses for continuous visualization')
    
    def republish_for_rviz(self):
        """Keep republishing stored path for RViz visualization"""
        if self.stored_path is not None:
            # Update timestamp
            self.stored_path.header.stamp = self.get_clock().now().to_msg()
            for pose in self.stored_path.poses:
                pose.header.stamp = self.stored_path.header.stamp
            
            self.pub.publish(self.stored_path)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothedPathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()