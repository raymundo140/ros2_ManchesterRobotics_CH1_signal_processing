# Import necessary libraries
import rclpy  # ROS 2 Python library
from rclpy.node import Node  # Base class for nodes
from std_msgs.msg import Float32  # Standard message type for floating-point numbers
import numpy as np  # Numerical computing library

# Node class for processing a mathematical signal
class SignalTransformer(Node):  
    def __init__(self):
        super().__init__('process') # Node name set as 'process'

        # Subscribing to two topics: one for input signals and one for time updates
        self.input_signal_sub = self.create_subscription(Float32, '/input_wave', self.modify_wave, 10)
        self.time_ref_sub = self.create_subscription(Float32, '/time_ref', self.update_time, 10)

        # Publisher to send out the processed signal
        self.transformed_wave_pub = self.create_publisher(Float32, '/transformed_wave', 10)

        # Parameters for modifying the signal
        self.phase_offset = np.pi / 3  # Adjusting phase shift (60 degrees instead of 45)
        self.vertical_shift = 1.2  # Vertical shift to ensure positivity
        self.scaling_factor = 0.4  # Scaling amplitude reduction factor
        self.current_time = 0.0  # Variable to store latest time received

    # Callback function to update the time variable
    def update_time(self, msg):
        self.current_time = msg.data  # Store received time value

    # Callback function to modify the input signal
    def modify_wave(self, msg):
        raw_wave = msg.data  # Retrieve original signal value
        
        # Apply transformation: phase shift, scaling, and offset adjustment
        altered_wave = np.sin(self.current_time + self.phase_offset)  
        transformed_signal = self.scaling_factor * (altered_wave + self.vertical_shift)  

        # Publish the modified wave to the new topic
        self.transformed_wave_pub.publish(Float32(data=transformed_signal))

        # Log the modified signal value
        self.get_logger().info(f'Altered Wave Output: {transformed_signal:.2f}')

# Main function to initialize and run the node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    processor_node = SignalTransformer()  # Create an instance of the node
    rclpy.spin(processor_node)  # Keep the node running and processing messages
    processor_node.destroy_node()  # Clean up before shutting down
    rclpy.shutdown()  # Shut down ROS 2 properly

# Run the main function when executed
if __name__ == '__main__':
    main()
