

# Importing necessary libraries
import rclpy  # ROS 2 client library
from rclpy.node import Node  # Base class for creating a ROS 2 node
from std_msgs.msg import Float32  # Standard message type for floating-point numbers
import numpy as np  # Library for numerical computations
import time  # Library to track time

# Create a node that generates a sinusoidal signal at a frequency of 10Hz
class WaveEmitter(Node):
    def __init__(self):
        super().__init__('wave_emitter')  # Node name changed to 'wave_emitter'

        # Publishers to send the wave signal and time reference
        self.wave_pub = self.create_publisher(Float32, '/input_wave', 10)  
        self.time_ref_pub = self.create_publisher(Float32, '/time_ref', 10)  

        # Timer to publish messages at a rate of 10Hz (0.1s interval)
        self.timer = self.create_timer(0.1, self.transmit_wave)  
        
        # Store the initial time reference
        self.start_timestamp = time.time()

    # Function to generate and publish the wave signal
    def transmit_wave(self):
        elapsed_time = time.time() - self.start_timestamp  # Compute elapsed time
        wave_value = np.sin(elapsed_time)  # Generate a sine wave value
        
        # Publish the generated wave and time reference to corresponding topics
        self.wave_pub.publish(Float32(data=wave_value))
        self.time_ref_pub.publish(Float32(data=elapsed_time))

        # Log the wave data for debugging
        self.get_logger().info(f'Timestamp: {elapsed_time:.2f}, Wave Value: {wave_value:.2f}')

# Main function to initialize and run the node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    wave_node = WaveEmitter()  # Create an instance of the node
    rclpy.spin(wave_node)  # Keep the node running
    wave_node.destroy_node()  # Clean up before shutting down
    rclpy.shutdown()  # Shut down ROS 2 properly

# Run the main function when executed
if __name__ == '__main__':
    main()
