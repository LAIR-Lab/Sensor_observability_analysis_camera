
# src/ft_sensor_reader.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np

class FTSensorReader(Node):
    def __init__(self):
        super().__init__('ft_sensor_reader')
        
        # Subscribe to F/T sensor topic
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/ft_sensor',  # Topic name from your sensor config
            self.ft_callback,
            10  # QoS queue size
        )
        
        self.get_logger().info('F/T Sensor Reader initialized')
        self.force_data = None
        self.torque_data = None
    
    def ft_callback(self, msg: WrenchStamped):
        """Callback for F/T sensor data"""
        # Extract force data
        force = msg.wrench.force
        self.force_data = np.array([force.x, force.y, force.z])
        
        # Extract torque data
        torque = msg.wrench.torque
        self.torque_data = np.array([torque.x, torque.y, torque.z])
        
        # Log the data
        self.get_logger().info(
            f'Force: [{force.x:.3f}, {force.y:.3f}, {force.z:.3f}] N\n'
            f'Torque: [{torque.x:.3f}, {torque.y:.3f}, {torque.z:.3f}] N·m\n'
            f'Frame: {msg.header.frame_id}'
        )
        
        # Calculate magnitude
        force_magnitude = np.linalg.norm(self.force_data)
        torque_magnitude = np.linalg.norm(self.torque_data)
        self.get_logger().info(
            f'Force Magnitude: {force_magnitude:.3f} N | '
            f'Torque Magnitude: {torque_magnitude:.3f} N·m'
        )

def main(args=None):
    rclpy.init(args=args)
    reader = FTSensorReader()
    
    try:
        rclpy.spin(reader)
    except KeyboardInterrupt:
        pass
    finally:
        reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()