#!/usr/bin/env python3

# ROS 2 module imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

# Constrain control commands
def constrain(input, low_bound, high_bound):
    if input < low_bound:
        input = low_bound
    elif input > high_bound:
        input = high_bound
    return input

def bound_steer(steer_cmd):
    return constrain(steer_cmd, -1.0, 1.0)

def bound_drive(drive_cmd):
    return constrain(drive_cmd, -0.05, 0.05)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        qos = QoSProfile(depth=1)
        
        # Publishers for throttle and steering commands
        self.pub_steering_command = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', qos)
        self.pub_throttle_command = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', qos)
        
        # Subscriber to /drive topic
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, qos)
        
        self.get_logger().info('Node has been initialized and waiting for /drive messages.')

    def drive_callback(self, msg):
        # Extract throttle (speed) and steering angle from /drive message
        throttle = bound_drive(msg.drive.speed)
        steering = bound_steer(msg.drive.steering_angle)

        if steering > 0.0:
            steering = -steering
        else:
            steering = abs(steering)
        # Create Float32 messages
        throttle_msg = Float32()
        steering_msg = Float32()

        # Assign extracted values to the messages
        throttle_msg.data = float(throttle)
        steering_msg.data = float(steering)

        # Publish to the respective topics
        self.pub_throttle_command.publish(throttle_msg)
        self.pub_steering_command.publish(steering_msg)

        # Log info
        self.get_logger().info(f'Published throttle: {throttle}, steering: {steering}')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
