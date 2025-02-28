import numpy as np
import serial
import threading
import cv2
import time
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from mpl_toolkits.mplot3d import Axes3D

class TactileSensorVisualizer:
    def __init__(self, port='/dev/ttyACM0', baud=115200, rows=16, cols=16, upscale_factor=4):
        # Configuration
        self.port = port
        self.baud = baud
        self.rows = rows
        self.cols = cols
        self.upscale_factor = upscale_factor
        self.upscaled_rows = rows * upscale_factor
        self.upscaled_cols = cols * upscale_factor
        
        # Sensitivity parameters - tuned for the observed range (0-150)
        self.threshold = 15      # Noise threshold
        self.min_pressure = 20   # Minimum pressure to consider as contact
        self.max_pressure = 150  # Maximum expected pressure value
        self.alpha_temporal = 0.6  # Higher value for faster response to new data
        
        # Data structures
        self.contact_data_norm = np.zeros((rows, cols))
        self.prev_frame = np.zeros((rows, cols))
        self.baseline = None
        self.calibration_samples = []
        self.calibration_complete = False
        self.frame_count = 0
        self.last_update_time = time.time()
        self.fps = 0
        
        # Frame management
        self.last_valid_frame = None
        self.last_frame_time = 0
        self.frame_timeout = 0.5  # Time in seconds before considering a frame stale
        
        # Visualization settings - high resolution
        self.scale_factor = 40
        self.window_width = cols * self.scale_factor
        self.window_height = rows * self.scale_factor
        
        # Create visualization windows
        cv2.namedWindow("Tactile Pressure Map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Tactile Pressure Map", self.window_width, self.window_height)
        
        cv2.namedWindow("3D Pressure Visualization", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("3D Pressure Visualization", self.window_width, self.window_height)
        
        # Visualization buffers to prevent flickering
        self.pressure_map_buffer = None
        self.plot_3d_buffer = None
        
        # Initialize mapping matrices based on physical model
        self.Q_matrix = self._initialize_mapping_matrices()
        self.W_matrix = self._initialize_pressure_mapping()
        
        # Set up threading for non-blocking serial reads
        self.data_lock = threading.Lock()
        self.running = True
        self.latest_frame = None
        self.frame_buffer = []  # Buffer to store recent frames
        self.max_buffer_size = 5  # Maximum number of frames to buffer
        
        # Connect to serial port
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.ser.flush()
            print(f"Connected to {port} at {baud} baud")
            
            # Start data acquisition thread
            self.data_thread = threading.Thread(target=self._data_acquisition_thread)
            self.data_thread.daemon = True
            self.data_thread.start()
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            self.ser = None

    def _initialize_mapping_matrices(self):
        """Initialize voltage-to-conductivity mapping matrix based on physical model equation"""
        # Create Jacobian matrix based on the equation δσ = (J^T J + α^2 R^T R)^-1 J^T δV
        J = np.eye(self.rows * self.cols)
        alpha = 0.1  # Regularization parameter
        R = np.eye(self.rows * self.cols)
        
        # Calculate Q matrix according to the equation in the image
        JtJ = J.T @ J
        RtR = R.T @ R
        return np.linalg.inv(JtJ + alpha**2 * RtR) @ J.T

    def _initialize_pressure_mapping(self):
        """Initialize conductivity-to-pressure mapping matrix"""
        # This would typically be derived from experimental data as shown in the image
        # The graph shows a logarithmic relationship between tactile reading and force
        W = np.eye(self.rows * self.cols)
        return W

    def _data_acquisition_thread(self):
        """Thread for continuous data acquisition from serial port"""
        current_frame = []
        
        while self.running:
            if self.ser is None:
                time.sleep(0.1)
                continue
                
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    # Skip empty or short lines
                    if len(line) < 3:
                        if len(current_frame) == self.rows:
                            # We have a complete frame
                            with self.data_lock:
                                frame_data = np.array(current_frame)
                                self.latest_frame = frame_data
                                
                                # Add to frame buffer for smoother visualization
                                self.frame_buffer.append(frame_data)
                                if len(self.frame_buffer) > self.max_buffer_size:
                                    self.frame_buffer.pop(0)
                                    
                            current_frame = []
                        continue
                    
                    # Parse row data
                    try:
                        values = [int(val) for val in line.split()]
                        if len(values) == self.cols:
                            current_frame.append(values)
                    except ValueError:
                        # Skip lines that can't be parsed
                        continue
            except Exception as e:
                print(f"Error in data acquisition: {e}")
                time.sleep(0.1)
    
    def calibrate(self, num_samples=30):
        """Perform initial calibration to establish baseline"""
        if self.ser is None:
            print("Serial connection not available")
            return False
            
        print("Starting calibration... Keep sensor unloaded")
        self.calibration_samples = []
        
        # Clear any pending data
        self.ser.reset_input_buffer()
        
        # Collect calibration samples
        for i in range(num_samples):
            # Initialize frame as None before attempting to read
            frame = None
            
            # Wait for a new frame from the acquisition thread
            start_time = time.time()
            while True:
                with self.data_lock:
                    if self.latest_frame is not None:
                        frame = self.latest_frame.copy()
                        self.latest_frame = None
                        break
                
                # Check for timeout
                if time.time() - start_time > 1.0:
                    print(f"Timeout waiting for frame {i+1}")
                    break
                    
                time.sleep(0.05)
            
            if frame is not None and frame.shape == (self.rows, self.cols):
                self.calibration_samples.append(frame)
                print(f"Calibration: {len(self.calibration_samples)}/{num_samples}")
            
            time.sleep(0.1)
        
        if len(self.calibration_samples) > 0:
            self.baseline = np.median(np.array(self.calibration_samples), axis=0)
            self.calibration_complete = True
            print("Calibration complete")
            return True
        else:
            print("Calibration failed - no data received")
            return False

    def process_frame(self, raw_frame):
        """Process a raw frame with enhanced sensitivity and noise removal"""
        if self.baseline is None or raw_frame is None:
            return None
            
        # Step 1: Subtract baseline and threshold
        delta_V = raw_frame - self.baseline - self.threshold
        delta_V = np.clip(delta_V, 0, None)  # Ensure non-negative values
        
        # Step 2: Apply voltage-to-conductivity mapping (Q matrix) from physical model
        delta_V_vec = delta_V.flatten()
        delta_sigma_vec = self.Q_matrix @ delta_V_vec
        delta_sigma = delta_sigma_vec.reshape(self.rows, self.cols)
        
        # Step 3: Apply non-linear transformation based on the graph in the image
        # The graph shows logarithmic relationship between tactile reading and force
        epsilon = 1e-6  # Small value to avoid log(0)
        pressure = np.log(delta_sigma + epsilon) * 20  # Scale factor
        
        # Step 4: Apply spatial filtering (Gaussian blur)
        pressure_filtered = gaussian_filter(pressure, sigma=0.8)
        
        # Step 5: Apply temporal filtering with faster response
        pressure_temp_filtered = self._temporal_filter(pressure_filtered)
        
        # Step 6: Normalize based on expected pressure range from the graph
        normalized = np.clip(pressure_temp_filtered / self.max_pressure, 0, 1)
        
        # Update current data
        self.contact_data_norm = normalized
        self.last_valid_frame = normalized
        self.last_frame_time = time.time()
        
        return normalized

    def _temporal_filter(self, new_frame):
        """Apply temporal smoothing with faster response to new data"""
        filtered = self.alpha_temporal * new_frame + (1 - self.alpha_temporal) * self.prev_frame
        self.prev_frame = filtered
        return filtered

    def upsample_data(self, data):
        """Upsample the data for higher resolution visualization"""
        # Use bicubic interpolation for smoother results
        upsampled = cv2.resize(data, (self.upscaled_cols, self.upscaled_rows), 
                              interpolation=cv2.INTER_CUBIC)
        return upsampled

    def create_3d_visualization(self, data):
        """Create a 3D surface plot visualization"""
        fig = plt.figure(figsize=(8, 6), dpi=100)
        ax = fig.add_subplot(111, projection='3d')
        
        # Create coordinate grid
        x = np.arange(0, data.shape[1])
        y = np.arange(0, data.shape[0])
        X, Y = np.meshgrid(x, y)
        
        # Plot surface with custom colormap
        surf = ax.plot_surface(X, Y, data, cmap='jet', 
                              linewidth=0, antialiased=True)
        
        # Set labels and remove ticks for cleaner look
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Pressure')
        ax.set_title('3D Pressure Distribution')
        ax.set_xticks([])
        ax.set_yticks([])
        
        # Adjust view angle for better visualization
        ax.view_init(elev=30, azim=45)
        
        # Convert matplotlib figure to OpenCV image
        fig.tight_layout(pad=0)
        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        plot_data = np.asarray(buf)
        plot_data = cv2.cvtColor(plot_data, cv2.COLOR_RGBA2BGR)
        plt.close(fig)
        
        return plot_data

    def visualize(self):
        """Display the current tactile pressure map with enhanced visualization"""
        if not self.calibration_complete:
            return False
        
        # Check if we have a valid frame or need to use the last valid one
        current_time = time.time()
        frame_data = self.contact_data_norm
        
        # Calculate FPS
        self.frame_count += 1
        if current_time - self.last_update_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_update_time)
            self.frame_count = 0
            self.last_update_time = current_time
        
        # 1. Basic visualization - original resolution with enhanced color mapping
        scaled_data = (frame_data * 255).astype(np.uint8)
        colormap = cv2.applyColorMap(scaled_data, cv2.COLORMAP_JET)
        
        # Resize for better visibility
        colormap_resized = cv2.resize(colormap, (self.window_width, self.window_height), 
                                     interpolation=cv2.INTER_NEAREST)
        
        # Add FPS counter and max pressure
        max_pressure = np.max(frame_data)
        cv2.putText(colormap_resized, f"FPS: {self.fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(colormap_resized, f"Max Pressure: {max_pressure:.2f}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Add frame status indicator
        frame_status = "Live Data" if current_time - self.last_frame_time < self.frame_timeout else "Buffered Data"
        cv2.putText(colormap_resized, frame_status, (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, 
                   (0, 255, 0) if frame_status == "Live Data" else (0, 165, 255), 2)
        
        # Store in buffer to prevent flickering
        self.pressure_map_buffer = colormap_resized
        
        # 2. Create 3D visualization only if we have new data (to save CPU)
        if current_time - self.last_frame_time < self.frame_timeout or self.plot_3d_buffer is None:
            # Upsample for smoother 3D visualization
            upsampled_data = self.upsample_data(frame_data)
            plot_3d = self.create_3d_visualization(upsampled_data)
            self.plot_3d_buffer = plot_3d
        
        # Display visualizations using the buffers
        cv2.imshow("Tactile Pressure Map", self.pressure_map_buffer)
        cv2.imshow("3D Pressure Visualization", self.plot_3d_buffer)
        
        # Check for key press (return True if 'q' is pressed to exit)
        key = cv2.waitKey(1) & 0xFF
        return key == ord('q')

    def get_frame_from_buffer(self):
        """Get a frame from the buffer to ensure smooth visualization"""
        with self.data_lock:
            if len(self.frame_buffer) > 0:
                # Return the most recent frame
                return self.frame_buffer[-1].copy()
            elif self.last_valid_frame is not None:
                # If buffer is empty but we have a last valid frame, use that
                return self.last_valid_frame.copy()
            else:
                # No frames available
                return None

    def visualize_fallback(self):
        """Fallback visualization when no data is available"""
        # Create a placeholder image
        placeholder = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
        
        # Add text explaining status
        cv2.putText(placeholder, "Waiting for data...", (self.window_width//4, self.window_height//2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        
        # Display the image
        cv2.imshow("Tactile Pressure Map", placeholder)
        cv2.imshow("3D Pressure Visualization", placeholder)
        
        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        return key == ord('q')

    def close(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'data_thread') and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
            
        if self.ser is not None:
            self.ser.close()
            
        cv2.destroyAllWindows()

def main():
    # Initialize tactile sensor visualizer with higher baud rate
    sensor = TactileSensorVisualizer(port='/dev/ttyACM0', baud=115200)
    
    try:
        # Perform initial calibration
        if sensor.calibrate(num_samples=30):
            print("Processing tactile data in real-time...")
            
            # Main processing loop
            while True:
                # Get the latest frame from the acquisition thread
                frame = sensor.get_frame_from_buffer()
                
                if frame is not None:
                    # Process and visualize the frame
                    sensor.process_frame(frame)
                    if sensor.visualize():
                        break  # Exit if 'q' is pressed
                else:
                    # Use fallback visualization if no data
                    if sensor.visualize_fallback():
                        break  # Exit if 'q' is pressed
                
                # Small delay to prevent high CPU usage
                time.sleep(0.01)
                
        else:
            print("Calibration failed. Check sensor connection.")
            
    except KeyboardInterrupt:
        print("\nStopping tactile sensor processing...")
    finally:
        sensor.close()
        print("Resources released")

if __name__ == "__main__":
    main()
