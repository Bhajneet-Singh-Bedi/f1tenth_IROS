#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
import tf2_ros # ROS bindings for tf2 library to handle transforms
from std_msgs.msg import Int32, Float32, Header # Int32, Float32 and Header message classes
from geometry_msgs.msg import Point, TransformStamped # Point and TransformStamped message classes
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes
from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation
from ament_index_python.packages import get_package_share_directory # Access package's shared directory path
from threading import Thread
from nav_msgs.msg import Odometry

# Python mudule imports
from cv_bridge import CvBridge, CvBridgeError # ROS bridge for opencv library to handle images
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from geventwebsocket.handler import WebSocketHandler # Handler for WebSocket messages and lifecycle events
import socketio # Socket.IO realtime client and server
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
from io import BytesIO # Manipulate bytes data in memory
from PIL import Image # Python Imaging Library's (PIL's) Image module
import configparser # Parsing shared configuration file(s)
import autodrive_f1tenth.config as config # AutoDRIVE Ecosystem ROS 2 configuration for F1TENTH vehicle
import math
################################################################################

# Global declarations
global autodrive_bridge, cv_bridge, publishers
global throttle_command, steering_command

# Initialize vehicle control commands
throttle_command = config.throttle_command
steering_command = config.steering_command

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_int_msg(val):
    i = Int32()
    i.data = int(val)
    return i

def create_float_msg(val):
    f = Float32()
    f.data = float(val)
    return f

def create_joint_state_msg(joint_angle, joint_name, frame_id):
    js = JointState()
    js.header = Header()
    js.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    js.header.frame_id = frame_id
    js.name = [joint_name]
    js.position = [joint_angle]
    js.velocity = []
    js.effort = []
    return js

def create_point_msg(position):
    p = Point()
    p.x = position[0]
    p.y = position[1]
    p.z = position[2]
    return p

def create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration):
    imu = Imu()
    imu.header = Header()
    imu.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    imu.header.frame_id = 'imu'
    imu.orientation.x = orientation_quaternion[0]
    imu.orientation.y = orientation_quaternion[1]
    imu.orientation.z = orientation_quaternion[2]
    imu.orientation.w = orientation_quaternion[3]
    imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.angular_velocity.x = angular_velocity[0]
    imu.angular_velocity.y = angular_velocity[1]
    imu.angular_velocity.z = angular_velocity[2]
    imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.linear_acceleration.x = linear_acceleration[0]
    imu.linear_acceleration.y = linear_acceleration[1]
    imu.linear_acceleration.z = linear_acceleration[2]
    imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    return imu

def create_laser_scan_msg(lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    ls = LaserScan()
    ls.header = Header()
    ls.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    ls.header.frame_id = 'lidar'
    ls.angle_min = -2.35619 # Minimum angle of laser scan (0 degrees)
    ls.angle_max = 2.35619 # Maximum angle of laser scan (270 degrees)
    ls.angle_increment = 0.004363323 # Angular resolution of laser scan (0.25 degree)
    ls.time_increment = (1 / lidar_scan_rate) / 360 # Time required to scan 1 degree
    ls.scan_time = ls.time_increment * 360 # Time required to complete a scan of 360 degrees
    ls.range_min = 0.06 # Minimum sensor range (in meters)
    ls.range_max = 10.0 # Maximum sensor range (in meters)
    ls.ranges = lidar_range_array
    ls.intensities = lidar_intensity_array
    return ls

def create_image_msg(image_array, frame_id):
    img = cv_bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
    img.header = Header()
    img.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    img.header.frame_id = frame_id
    return img
######################
def create_odom_msg(current_time, position, linear_velocity, angular_velocity, theta):
    # Publish the odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time.to_msg()
    odom_msg.header.frame_id = 'world'  # Use world as the global frame
    odom_msg.child_frame_id = 'f1tenth_1'  # Robot base link

    # Set the position
    odom_msg.pose.pose.position.x = position[0]
    odom_msg.pose.pose.position.y = position[1]
    odom_msg.pose.pose.position.z = 0.0  # Assuming a 2D plane
    odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
    odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

    # Set the velocity
    odom_msg.twist.twist.linear.x = linear_velocity
    odom_msg.twist.twist.angular.z = angular_velocity
    return odom_msg
#####################
def broadcast_transform(child_frame_id, parent_frame_id, position_tf, orientation_tf):
    tb = tf2_ros.TransformBroadcaster(autodrive_bridge)
    tf = TransformStamped()
    tf.header.stamp = autodrive_bridge.get_clock().now().to_msg()
    tf.header.frame_id = parent_frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = position_tf[0] # Pos X
    tf.transform.translation.y = position_tf[1] # Pos Y
    tf.transform.translation.z = position_tf[2] # Pos Z
    tf.transform.rotation.x = orientation_tf[0] # Quat X
    tf.transform.rotation.y = orientation_tf[1] # Quat Y
    tf.transform.rotation.z = orientation_tf[2] # Quat Z
    tf.transform.rotation.w = orientation_tf[3] # Quat W
    tb.sendTransform(tf)

#########################################################
# ROS 2 PUBLISHER FUNCTIONS
#########################################################

# VEHICLE DATA PUBLISHER FUNCTIONS

def publish_actuator_feedbacks(throttle, steering):
    publishers['pub_throttle'].publish(create_float_msg(throttle))
    publishers['pub_steering'].publish(create_float_msg(steering))

def publish_speed_data(speed):
    publishers['pub_speed'].publish(create_float_msg(speed))

def publish_encoder_data(encoder_angles):
    publishers['pub_left_encoder'].publish(create_joint_state_msg(encoder_angles[0], "left_encoder", "left_encoder"))
    publishers['pub_right_encoder'].publish(create_joint_state_msg(encoder_angles[1], "right_encoder", "right_encoder"))

def publish_ips_data(position):
    publishers['pub_ips'].publish(create_point_msg(position))

def publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration):
    publishers['pub_imu'].publish(create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration))

def publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array):
    publishers['pub_lidar'].publish(create_laser_scan_msg(lidar_scan_rate, lidar_range_array.tolist(), lidar_intensity_array.tolist()))

def publish_camera_images(front_camera_image):
    publishers['pub_front_camera'].publish(create_image_msg(front_camera_image, "front_camera"))

def publish_lap_count_data(lap_count):
    publishers['pub_lap_count'].publish(create_int_msg(lap_count))

def publish_lap_time_data(lap_time):
    publishers['pub_lap_time'].publish(create_float_msg(lap_time))

def publish_last_lap_time_data(last_lap_time):
    publishers['pub_last_lap_time'].publish(create_float_msg(last_lap_time))

def publish_best_lap_time_data(best_lap_time):
    publishers['pub_best_lap_time'].publish(create_float_msg(best_lap_time))

def publish_collision_count_data(collision_count):
    publishers['pub_collision_count'].publish(create_int_msg(collision_count))

#################
def publish_odom_data(current_time, position, linear_velocity, angular_velocity, theta):
    publishers['pub_odom'].publish(create_odom_msg(current_time, position, linear_velocity, angular_velocity, theta))
#################
    

#########################################################
# ROS 2 SUBSCRIBER CALLBACKS
#########################################################

# VEHICLE DATA SUBSCRIBER CALLBACKS

def callback_throttle_command(throttle_command_msg):
    global throttle_command
    throttle_command = float(np.round(throttle_command_msg.data, 3))

def callback_steering_command(steering_command_msg):
    global steering_command
    steering_command = float(np.round(steering_command_msg.data, 3))

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print("Connected!")

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    # Global declarations
    global autodrive_bridge, cv_bridge, publishers
    global throttle_command, steering_command
    global theta, prev_time, prev_x, prev_y, wheel_base
    # Get package's shared directory path
    package_share_directory = get_package_share_directory('autodrive_f1tenth')

    # Wait for data to become available
    if data:
        ########################################################################
        # VEHICLE DATA
        ########################################################################
        # Actuator feedbacks
        throttle = float(data["V1 Throttle"])
        steering = float(data["V1 Steering"])
        publish_actuator_feedbacks(throttle, steering)
        # Speed
        speed = float(data["V1 Speed"])
        publish_speed_data(speed)
        # Wheel encoders
        encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        publish_encoder_data(encoder_angles)
        # IPS
        position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        publish_ips_data(position)
        # IMU
        orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration)

        ######################ODOM
        # current_time = autodrive_bridge.get_clock().now()
        # delta_time = (current_time - prev_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        # if prev_x is not None and prev_y is not None:
        #     # Calculate the change in position
        #     delta_x = position[0] - prev_x
        #     delta_y = position[1] - prev_y

        #     # Calculate the heading (theta) based on the change in position
        #     theta = math.atan2(delta_y, delta_x) if delta_x != 0 or delta_y != 0 else theta

        #     # Calculate velocity based on the distance traveled
        #     linear_velocity = math.sqrt(delta_x**2 + delta_y**2) / delta_time if delta_time > 0 else 0.0

        #     # Calculate the angular velocity from the difference in wheel encoders
        #     left_wheel_velocity = encoder_angles[0] / delta_time
        #     right_wheel_velocity = encoder_angles[1] / delta_time
        #     angular_velocity = (right_wheel_velocity - left_wheel_velocity) / wheel_base

        # else:
        #     linear_velocity = 0.0
        #     angular_velocity = 0.0

        # Publish odometry
        # publish_odom_data(current_time, position, linear_velocity, angular_velocity, theta)
        
        # Store the current position and time for the next loop
        # prev_x = position[0]
        # prev_y = position[1]
        # prev_time = current_time
        ############################

        # Cooordinate transforms
        broadcast_transform("f1tenth_1", "world", position, orientation_quaternion) # Vehicle frame defined at center of rear axle
        broadcast_transform("left_encoder", "f1tenth_1", np.asarray([0.0, 0.12, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[0]%6.283, 0.0))
        broadcast_transform("right_encoder", "f1tenth_1", np.asarray([0.0, -0.12, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[1]%6.283, 0.0))
        broadcast_transform("ips", "f1tenth_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform("imu", "f1tenth_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform("lidar", "f1tenth_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform("front_camera", "f1tenth_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]))
        broadcast_transform("front_left_wheel", "f1tenth_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537-2*0.0765*np.tan(steering)))))
        broadcast_transform("front_right_wheel", "f1tenth_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537+2*0.0765*np.tan(steering)))))
        broadcast_transform("rear_left_wheel", "f1tenth_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[0]%6.283, 0.0))
        broadcast_transform("rear_right_wheel", "f1tenth_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[1]%6.283, 0.0))

        # LIDAR
        lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])
        lidar_range_array = np.fromstring(data["V1 LIDAR Range Array"], dtype=float, sep=' ')
        lidar_intensity_array = np.fromstring(data["V1 LIDAR Intensity Array"], dtype=float, sep=' ')
        publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array)
        # Cameras
        front_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
        publish_camera_images(front_camera_image)
        # Lap data
        lap_count = int(float(data["V1 Lap Count"]))
        lap_time = float(data["V1 Lap Time"])
        last_lap_time = float(data["V1 Last Lap Time"])
        best_lap_time = float(data["V1 Best Lap Time"])
        collision_count = int(float(data["V1 Collisions"]))
        publish_lap_count_data(lap_count)
        publish_lap_time_data(lap_time)
        publish_last_lap_time_data(last_lap_time)
        publish_best_lap_time_data(best_lap_time)
        publish_collision_count_data(collision_count)

        ########################################################################
        # CONTROL COMMANDS
        ########################################################################
        # Vehicle and traffic light control commands
        sio.emit('Bridge', data={'V1 Throttle': str(throttle_command), 'V1 Steering': str(steering_command)})

#########################################################
# AUTODRIVE ROS 2 BRIDGE INFRASTRUCTURE
#########################################################

def main():
    # Global declarations
    global autodrive_bridge, cv_bridge, publishers
    global throttle_command, steering_command
    ################ Globalization
    global theta, prev_time, prev_x, prev_y, wheel_base
    ################
    # ROS 2 infrastructure
    rclpy.init() # Initialize ROS 2 communication for this context
    autodrive_bridge = rclpy.create_node('autodrive_bridge') # Create ROS 2 node
    ########### Initialization
    prev_x = None
    prev_y = None
    theta = 0.0
    prev_time = autodrive_bridge.get_clock().now()
    wheel_base = 0.3240
    ##################
    qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
    cv_bridge = CvBridge() # ROS bridge object for opencv library to handle image data
    publishers = {e.name: autodrive_bridge.create_publisher(e.type, e.topic, qos_profile)
                  for e in config.pub_sub_dict.publishers} # Publishers
    callbacks = {
        # Vehicle data subscriber callbacks
        '/autodrive/f1tenth_1/throttle_command': callback_throttle_command,
        '/autodrive/f1tenth_1/steering_command': callback_steering_command,
    } # Subscriber callback functions
    subscribers = [autodrive_bridge.create_subscription(e.type, e.topic, callbacks[e.topic], qos_profile)
                   for e in config.pub_sub_dict.subscribers] # Subscribers
    subscribers # Avoid unused variable warning

    executor = rclpy.executors.SingleThreadedExecutor() # Create executor to control which threads callbacks get executed in
    executor.add_node(autodrive_bridge) # Add a node whose callbacks should be managed by this executor

    process = Thread(target=executor.spin, daemon=True) # Runs callbacks in the thread
    process.start() # Activate the thread as demon (background process) and prompt it to the target function (spin the executor)

    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as a gevent WSGI server
    
    autodrive_bridge.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # Shutdown this context

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE ROS 2 bridge