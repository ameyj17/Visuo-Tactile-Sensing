#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import open3d as o3d

class TactileSubscriber(Node):
    def __init__(self):
        super().__init__('tactile_subscriber')
        # Subscribe to the tactile topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'tactile_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Initialize Open3D visualization
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Tactile Force 3D Point Cloud", width=800, height=800)
        
        # Create a point cloud object
        self.point_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.point_cloud)
        
        # Initialize previous frame for temporal filtering
        self.prev_frame = np.zeros((16, 16))
        
        # Initialize previous center for motion tracking
        self.prev_center = None

    def listener_callback(self, msg):
        # --- Data Parsing ---
        # Convert the flattened data back into a 16x16 NumPy array.
        data = np.array(msg.data)
        if data.size != 256:
            self.get_logger().warn("Expected 256 elements but got {}".format(data.size))
            return
        frame = data.reshape((16, 16))
        
        # --- Temporal Smoothing ---
        alpha = 0.5
        smoothed_frame = alpha * frame + (1 - alpha) * self.prev_frame
        self.prev_frame = smoothed_frame
        
        # --- Generate Dense 3D Point Cloud ---
        rows, cols = smoothed_frame.shape
        x, y = np.meshgrid(np.linspace(0, cols - 1, cols), np.linspace(0, rows - 1, rows))
        z = smoothed_frame
        
        # Flatten arrays for Open3D point cloud format
        points = np.vstack((x.flatten(), y.flatten(), z.flatten())).T
        
        # Normalize colors based on pressure values (z-axis)
        colors = z.flatten() / np.max(z) if np.max(z) > 0 else z.flatten()
        colors = np.vstack((colors, colors, colors)).T
        
        # Update point cloud object
        self.point_cloud.points = o3d.utility.Vector3dVector(points)
        self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
        
        # --- Motion Tracking: Compute Center of Pressure ---
        total_pressure = np.sum(smoothed_frame)
        if total_pressure > 0:
            indices = np.indices(smoothed_frame.shape)
            center_row = np.sum(indices[0] * smoothed_frame) / total_pressure
            center_col = np.sum(indices[1] * smoothed_frame) / total_pressure
            center_of_pressure = (center_row, center_col)
            
            # Draw motion vector if previous center exists
            if self.prev_center is not None:
                motion_vector_start = [self.prev_center[1], self.prev_center[0], np.max(z)]
                motion_vector_end = [center_col, center_row, np.max(z)]
                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector([motion_vector_start, motion_vector_end]),
                    lines=o3d.utility.Vector2iVector([[0, 1]])
                )
                line_set.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0]])  # Red color for motion vector
                self.vis.add_geometry(line_set)
            
            self.prev_center = center_of_pressure
        
        # --- Update Visualization ---
        self.vis.update_geometry(self.point_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)
    node = TactileSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
