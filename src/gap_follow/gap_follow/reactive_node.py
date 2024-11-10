#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from queue import PriorityQueue

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class GapFollower(Node):
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Publishers, Subscribers
        lidarscan_topic = '/autodrive/f1tenth_1/lidar'
        steering_topic = '/autodrive/f1tenth_1/steering_command'
        throttle_topic = '/autodrive/f1tenth_1/throttle_command'

        # Initialize publishers for steering and throttle
        self.steering_publisher = self.create_publisher(Float32, steering_topic, 10)
        self.throttle_publisher = self.create_publisher(Float32, throttle_topic, 10)

        # Subscribe to Lidar topic
        self.scan_subscriber = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10
        )

        # Constants
        self.blur_size = 10
        self.safe_distance = 2.0
        self.danger_distance = 1.0
        self.jump_threshold = 2.0
        self.max_velocity = 0.05 
        self.min_velocity = 0.05
        self.max_gap_size = 300
        self.min_gap_size = 50
        self.smoothing_window = 20

    def filter_lidar_readings(self, ranges):
        smoothed_ranges = []
        window = self.smoothing_window
        for i in range(0, len(ranges), window):
            current_mean = round(sum(ranges[i:i + window]) / window, 5)
            smoothed_ranges.extend([current_mean] * window)
        return np.array(smoothed_ranges)

    def mask_danger_zones(self, start_idx, end_idx, ranges):
        idx = start_idx
        while idx < end_idx:
            if ranges[idx] <= self.danger_distance:
                ranges[max(0, idx - self.blur_size): idx + self.blur_size] = 0
                idx += self.blur_size
            else:
                idx += 1
        return ranges

    def select_best_heading(self, start_idx, end_idx, closest_idx, ranges):
        idx = start_idx
        safe_ranges = PriorityQueue()
        while idx < end_idx:
            if ranges[idx] >= self.safe_distance:
                safe_start = idx
                idx += 1
                while (idx < end_idx and ranges[idx] >= self.safe_distance
                       and idx - safe_start <= self.max_gap_size
                       and abs(ranges[idx] - ranges[max(0, idx - 1)]) < self.jump_threshold):
                    idx += 1
                safe_end = max(0, idx - 1)
                if safe_end != safe_start:
                    safe_ranges.put((-(np.max(ranges[safe_start:safe_end])), (safe_start, safe_end)))
            else:
                idx += 1
        if safe_ranges.empty():
            print('No safe ranges found')
            return np.argmax(ranges)
        else:
            while not safe_ranges.empty():
                safe_start, safe_end = safe_ranges.get()[1]
                target_idx = (safe_start + safe_end) // 2
                if 179 <= target_idx <= 900 and safe_end - safe_start > self.min_gap_size:
                    print(f'left: {safe_start}, right: {safe_end}')
                    return target_idx
            return target_idx

    def lidar_callback(self, scan_msg):
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min        
        ranges = scan_msg.ranges

        processed_ranges = self.filter_lidar_readings(ranges)
        closest_point_idx = np.argmin(processed_ranges)

        processed_ranges = self.mask_danger_zones(0, len(processed_ranges), processed_ranges)

        farthest_point_idx = self.select_best_heading(0, len(processed_ranges), closest_point_idx, processed_ranges)
        steering_angle = angle_min + farthest_point_idx * angle_increment

        # Set initial velocity to max speed
        velocity = self.max_velocity

        print(f'Farthest point index: {farthest_point_idx}')
        print(f'Range at farthest point: {processed_ranges[farthest_point_idx]}')
        print(f'Steering angle: {math.degrees(steering_angle)}')
        
        # Adjust velocity based on steering angle
        if abs(steering_angle) >= 0.3:
            velocity = self.min_velocity

        # Publish throttle (velocity) and steering angle
        throttle_msg = Float32()
        throttle_msg.data = velocity
        self.throttle_publisher.publish(throttle_msg)

        steering_msg = Float32()
        steering_msg.data = steering_angle
        self.steering_publisher.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    print("GapFollower Node Initialized")
    reactive_node = GapFollower()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
