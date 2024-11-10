import rclpy
from rclpy.node import Node

import sensor_msgs.msg
from std_msgs.msg import Float32

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(sensor_msgs.msg.LaserScan,'/autodrive/f1tenth_1/lidar',self.lidar_scans,1)
        self.subscription  # prevent unused variable warning
        self.safe_distance = 1.0
        self.p=0.01
        self.i=0.0
        self.d=0
        self.dt = 1
        # PID controller variables
        self.error_prev = 0.0     # Previous error
        self.integral = 0.0       # Integral component
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 1)

    def lidar_scans(self, msg):
        # self.get_logger().info('0th value:- %s' % (msg.angle_min*180)/3.14)
        # self.get_logger().info('Max Value:-  %s' % (msg.angle_max*180)/3.14)
        # self.get_logger().info('Max Value:-  %s' % msg.ranges[540])
        # self.get_logger().info('Time now:- %s' %self.get_clock().now().to_msg())

        # Distance to maintain = 2.0m
        # Making a PID controller to keep the distance 2m.
        # If the distance is 'inf', set it to a high value
        distance = msg.ranges[540]
        self.get_logger().info(f"Distance: {distance}")

        if distance == float('inf'):
            # self.get_logger().info("hit")
            throttle = 1.0
            error=0.0
        else:
            # PID control
            error = self.safe_distance - distance
            self.integral += error * self.dt
            derivative = (error - self.error_prev) / self.dt
            throttle = self.p * error + self.i * self.integral + self.d * derivative
        

        # if throttle < 0:
        #     throttle=0.0
        
        # Publish throttle command
        throttle_msg = Float32()
        throttle_msg.data = throttle
        self.throttle_publisher.publish(throttle_msg)
        
        # Update previous error
        self.error_prev = error

        # Log information
        self.get_logger().info(f"Distance: {distance}, Throttle Command: {throttle}")




        



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()