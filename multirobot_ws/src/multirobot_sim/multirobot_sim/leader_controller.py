#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion
import time


class PIDController(Node):
    def __init__(self):
        super().__init__('leader_controller')
        
        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.traj_sub = self.create_subscription(Pose2D, '/desired_trajectory', self.traj_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Timer for control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # PID gains
        self.kvp = 2.25
        self.kvi = 0.0
        self.kvd = 0.1
        self.kwp = 1.5
        self.kwi = 0.0
        self.kwd = 0.05
        
        # Robot state
        self.robot_pose = Pose2D()
        self.desired_pose = Pose2D()
        self.odom_received = False
        self.traj_received = False
        
        # PID state variables
        self.ev_prev = 0.0
        self.ew_prev = 0.0
        self.integral_ev = 0.0
        self.integral_ew = 0.0
        self.dt = 0.1
        self.stop_threshold = 0.05
        
        # Obstacle avoidance parameters
        self.safe_distance = 0.5
        self.avoidance_distance = 1.2
        
        # Persistent avoidance state
        self.avoidance_active = False
        self.avoidance_start_pose = None
        self.avoidance_target = None
        self.avoidance_min_distance = 1.5  # minimum distance to travel during avoidance
        
        self.get_logger().info('Leader Controller with Persistent Avoidance started')

    def odom_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose.theta = yaw
        self.odom_received = True

    def traj_callback(self, msg):
        self.desired_pose = msg
        self.traj_received = True

    def laser_callback(self, msg):
        self.laser_data = msg

    def check_front_obstacle(self):
        if not hasattr(self, 'laser_data') or self.laser_data is None:
            return False, float('inf')
        
        total_rays = len(self.laser_data.ranges)
        front_rays = total_rays // 6
        front_readings = self.laser_data.ranges[-front_rays//2:] + self.laser_data.ranges[:front_rays//2]
        valid_readings = [r for r in front_readings if not math.isinf(r) and not math.isnan(r)]
        
        if not valid_readings:
            return False, float('inf')
        
        min_dist = min(valid_readings)
        return min_dist < self.avoidance_distance, min_dist

    def find_clear_direction(self):
        if not hasattr(self, 'laser_data') or self.laser_data is None:
            return 0.0
        
        total_rays = len(self.laser_data.ranges)
        quarter = total_rays // 4
        
        # Left side
        left_readings = self.laser_data.ranges[:quarter]
        left_valid = [r for r in left_readings if not math.isinf(r) and not math.isnan(r)]
        left_avg = sum(left_valid) / len(left_valid) if left_valid else float('inf')
        
        # Right side
        right_readings = self.laser_data.ranges[-quarter:]
        right_valid = [r for r in right_readings if not math.isinf(r) and not math.isnan(r)]
        right_avg = sum(right_valid) / len(right_valid) if right_valid else float('inf')

        return math.pi / 6 if left_avg > right_avg else -math.pi / 6

    def start_avoidance_maneuver(self):
        """Initialize a new avoidance maneuver"""
        avoid_angle = self.robot_pose.theta + self.find_clear_direction()
        avoid_distance = 2.0
        
        # Store avoidance state
        self.avoidance_active = True
        self.avoidance_start_pose = Pose2D()
        self.avoidance_start_pose.x = self.robot_pose.x
        self.avoidance_start_pose.y = self.robot_pose.y
        self.avoidance_start_pose.theta = self.robot_pose.theta
        
        # Set target for avoidance
        self.avoidance_target = Pose2D()
        self.avoidance_target.x = self.robot_pose.x + avoid_distance * math.cos(avoid_angle)
        self.avoidance_target.y = self.robot_pose.y + avoid_distance * math.sin(avoid_angle)
        
        self.get_logger().info(f'Starting avoidance maneuver - direction: {avoid_angle:.2f} rad')

    def should_continue_avoidance(self):
        """Check if avoidance should continue based on distance criteria"""
                
        # Calculate distance traveled since avoidance started
        dx = self.robot_pose.x - self.avoidance_start_pose.x
        dy = self.robot_pose.y - self.avoidance_start_pose.y
        distance_traveled = math.sqrt(dx*dx + dy*dy)
        
        # Continue if distance threshold hasn't been met
        should_continue = (distance_traveled < self.avoidance_min_distance)  
        
        if not should_continue:
            self.get_logger().info(f'Ending avoidance - Distance: {distance_traveled:.1f}m')
        
        return should_continue

    def end_avoidance_maneuver(self):
        self.avoidance_active = False
        self.avoidance_start_pose = None
        self.avoidance_target = None

    def control_loop(self):
        if not self.odom_received or not self.traj_received:
            return
        
        # Check for obstacles
        obstacle_ahead, dist = self.check_front_obstacle()
        
        # Emergency stop if too close
        if obstacle_ahead and dist < self.safe_distance:
            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            self.get_logger().warn(f'Emergency stop! Obstacle at {dist:.2f}m')
            # End any active avoidance
            if self.avoidance_active:
                self.end_avoidance_maneuver()
            return
        
        # Avoidance state machine
        if not self.avoidance_active and obstacle_ahead and dist > self.safe_distance:
            # Start new avoidance maneuver
            self.start_avoidance_maneuver()
        elif self.avoidance_active:
            # Check if we should continue avoidance
            if not self.should_continue_avoidance():
                self.end_avoidance_maneuver()
        
        # Determine target based on avoidance state
        if self.avoidance_active and self.avoidance_target is not None:
            # Use avoidance target
            dx = self.avoidance_target.x - self.robot_pose.x
            dy = self.avoidance_target.y - self.robot_pose.y
            speed_factor = 0.3
            self.get_logger().debug(f'Following avoidance target: ({self.avoidance_target.x:.2f}, {self.avoidance_target.y:.2f})')
        else:
            # Use normal trajectory
            dx = self.desired_pose.x - self.robot_pose.x
            dy = self.desired_pose.y - self.robot_pose.y
            speed_factor = 1.0
        
        # Position error
        ev = math.sqrt(dx*dx + dy*dy)
        
        if ev < self.stop_threshold and not self.avoidance_active:
            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            return
        
        # Angular error
        ew = math.atan2(dy, dx) - self.robot_pose.theta
        ew = math.atan2(math.sin(ew), math.cos(ew))
        
        # Derivatives
        dev_dt = (ev - self.ev_prev) / self.dt
        dew_dt = (ew - self.ew_prev) / self.dt
        
        # Integrals
        self.integral_ev += ev * self.dt
        self.integral_ew += ew * self.dt
        self.integral_ev = max(-5.0, min(5.0, self.integral_ev))
        self.integral_ew = max(-5.0, min(5.0, self.integral_ew))
        
        # PID control
        vr = self.kvp * ev + self.kvi * self.integral_ev + self.kvd * dev_dt
        wr = self.kwp * ew + self.kwi * self.integral_ew + self.kwd * dew_dt
        
        # Store errors
        self.ev_prev = ev
        self.ew_prev = ew
        
        # Apply speed factor and limits
        vr = max(-0.8, min(0.8, vr))
        wr = max(-2.0, min(2.0, wr))
        vr *= speed_factor

        # Publish
        vel_msg = Twist()
        vel_msg.linear.x = vr
        vel_msg.angular.z = wr
        self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        vel_msg = Twist()
        node.vel_pub.publish(vel_msg)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()