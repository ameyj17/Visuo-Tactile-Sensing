# #!usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

def temporal_filter(new_frame, prev_frame, alpha=1):
    """
    Apply temporal smoothing filter.
    'alpha' determines the blending factor.
    A higher alpha gives more weight to the current frame, while a lower alpha gives more weight to the previous frame.
    """
    return alpha * new_frame + (1 - alpha) * prev_frame

class TactilePublisher(Node):
    def __init__(self):
        super().__init__('tactile_processor')
        # Publisher for the processed tactile data (flattened 16x16 frame)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'tactile_topic', 10)
        # Timer simulating one row read every ~33ms (approx. 30Hz per row)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # Frame simulation variables
        self.total_frames = 0    # Number of complete frames processed so far
        self.max_frames = 25     # Total frames to simulate
        self.current_row = 0     # Current row index in the ongoing frame (0 to 15)
        self.current_frame = []  # Buffer to accumulate rows
        
        # For baseline computation (using the first few frames)
        self.init_frames = []
        self.init_frames_needed = 3
        self.baseline_computed = False
        self.baseline = None
        
        # For temporal filtering: initialize previous frame as zeros
        self.alpha = 0.5
        self.prev_frame = np.zeros((16, 16))
        
    def timer_callback(self):
        # Stop if we've reached the max number of frames.
        if self.total_frames >= self.max_frames:
            self.get_logger().info("Reached maximum of {} frames.".format(self.max_frames))
            return
        
        # --- Step A: Generate Raw Row (simulate 8-bit ASCII values 0–255) ---
        # Simulate a grasp event for frames 5–9 and 15–19:
        if (5 <= self.total_frames < 10) or (15 <= self.total_frames < 20):
            # During grasp events, sensor values are higher (simulate with values between 150 and 200)
            raw_row = np.random.randint(150, 201, size=16)
        else:
            # Otherwise, baseline readings (simulate around 100 with a small noise range)
            raw_row = np.random.randint(95, 106, size=16)
        
        # Log the raw row
        self.get_logger().info("Frame {} Row {}: Raw row: {}".format(self.total_frames, self.current_row, raw_row.tolist()))
        self.current_frame.append(raw_row)
        self.current_row += 1
        
        # If we have collected 16 rows, we have a complete frame.
        if self.current_row == 16:
            # Convert the accumulated rows to a NumPy array (16x16)
            raw_frame = np.array(self.current_frame, dtype=float)
            self.get_logger().info("Frame {}: Raw Frame:\n{}".format(self.total_frames, raw_frame))
            
            # --- Step B: Baseline Subtract ---
            # For the first few frames, collect data to compute the baseline.
            if not self.baseline_computed:
                self.init_frames.append(raw_frame)
                if len(self.init_frames) >= self.init_frames_needed:
                    # Compute the median across the initialization frames.
                    self.baseline = np.median(np.array(self.init_frames), axis=0)
                    self.baseline_computed = True
                    self.get_logger().info("Computed Baseline (median of first {} frames):\n{}".format(self.init_frames_needed, self.baseline))
                # If baseline not yet computed, use 0 for subtraction.
                baseline_subtracted = raw_frame - (self.baseline if self.baseline is not None else 0) - 12
            else:
                baseline_subtracted = raw_frame - self.baseline - 12
            self.get_logger().info("Frame {}: After Baseline Subtract:\n{}".format(self.total_frames, baseline_subtracted))
            
            # --- Step C: Clipping ---
            clipped = np.clip(baseline_subtracted, 0, 100)
            self.get_logger().info("Frame {}: After Clipping:\n{}".format(self.total_frames, clipped))
            
            # --- Step D: Normalize ---
            max_val = np.max(clipped)
            if max_val > 0:
                normalized = clipped / max_val
            else:
                normalized = clipped
            self.get_logger().info("Frame {}: After Normalization:\n{}".format(self.total_frames, normalized))
            
            # --- Step E: Temporal Filter ---
            filtered = temporal_filter(normalized, self.prev_frame, alpha=self.alpha)
            self.prev_frame = filtered
            self.get_logger().info("Frame {}: After Temporal Filter:\n{}".format(self.total_frames, filtered))
            
            # --- Step F: ROS Publish ---
            msg = Float64MultiArray()
            msg.data = filtered.flatten().tolist()
            self.publisher_.publish(msg)
            self.get_logger().info("Frame {}: Published processed tactile data.".format(self.total_frames))
            
            # Reset for the next frame
            self.current_frame = []
            self.current_row = 0
            self.total_frames += 1

def main(args=None):
    rclpy.init(args=args)
    node = TactilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

