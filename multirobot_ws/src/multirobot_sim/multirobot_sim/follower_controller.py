#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion


class FormationFollower(Node):
    def __init__(self):
        super().__init__('formation_follower')
        
        # Parameters
        self.declare_parameter('robot_name', 'follower_1')
        self.declare_parameter('offset_x', -2.0)
        self.declare_parameter('offset_y', 0.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        
        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.leader_sub = self.create_subscription(Odometry, '/odom', self.leader_callback, 10)
        self.robot_sub = self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.robot_callback, 10)
        self.ultrasonic_sub = self.create_subscription(LaserScan, f'/{self.robot_name}/ultrasonic_sensor', self.ultrasonic_callback, 10)
        
        # Timer for control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # PID gains (same as leader)
        self.kvp = 2.25  # Proportional gain for linear velocity
        self.kvi = 0.0   # Integral gain for linear velocity
        self.kvd = 0.1   # Derivative gain for linear velocity
        
        self.kwp = 1.5   # Proportional gain for angular velocity
        self.kwi = 0.0   # Integral gain for angular velocity
        self.kwd = 0.05  # Derivative gain for angular velocity
        
        # Robot state
        self.leader_pose = [0, 0, 0]  # [x, y, theta]
        self.robot_pose = [0, 0, 0]
        self.leader_received = False
        self.robot_received = False
        
        # PID state variables
        self.ev_prev = 0.0
        self.ew_prev = 0.0
        self.integral_ev = 0.0
        self.integral_ew = 0.0
        self.dt = 0.1
        
        # Stop threshold
        self.stop_threshold = 0.05  # meters
        
        # Obstacle avoidance parameters
        self.safe_distance = 0.3  # Smaller for ultrasonic sensor
        self.avoidance_distance = 0.8  # Smaller detection range
        
        # Persistent avoidance state
        self.avoidance_active = False
        self.avoidance_start_pose = None
        self.avoidance_target = None
        self.avoidance_min_distance = 1.0  # Smaller avoidance distance
        
        self.get_logger().info(f'Formation Follower {self.robot_name} with Obstacle Avoidance started')

    def leader_callback(self, msg):
        self.leader_pose[0] = msg.pose.pose.position.x
        self.leader_pose[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.leader_pose[2] = yaw
        
        self.leader_received = True

    def robot_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose[2] = yaw
        
        self.robot_received = True

    def ultrasonic_callback(self, msg):
        self.ultrasonic_data = msg

    def check_front_obstacle(self):
        """Check for obstacles using ultrasonic sensor"""
        if not hasattr(self, 'ultrasonic_data') or self.ultrasonic_data is None:
            return False, float('inf')
        
        # Ultrasonic sensor has narrow beam, check all readings
        valid_readings = [r for r in self.ultrasonic_data.ranges if not math.isinf(r) and not math.isnan(r)]
        
        if not valid_readings:
            return False, float('inf')
        
        min_dist = min(valid_readings)
        return min_dist < self.avoidance_distance, min_dist

    def find_clear_direction(self):
        """Find clear direction for ultrasonic sensor"""
        if not hasattr(self, 'ultrasonic_data') or self.ultrasonic_data is None:
            return math.pi / 4  # Default left turn
        
        # For ultrasonic sensor with narrow beam, analyze left vs right readings
        total_readings = len(self.ultrasonic_data.ranges)
        mid_point = total_readings // 2
        
        # Left side readings
        left_readings = self.ultrasonic_data.ranges[-mid_point:]
        left_valid = [r for r in left_readings if not math.isinf(r) and not math.isnan(r)]
        left_avg = sum(left_valid) / len(left_valid) if left_valid else float('inf')
        
        # Right side readings
        right_readings = self.ultrasonic_data.ranges[:mid_point]
        right_valid = [r for r in right_readings if not math.isinf(r) and not math.isnan(r)]
        right_avg = sum(right_valid) / len(right_valid) if right_valid else float('inf')
        
        # Choose direction with more clearance
        return math.pi / 4 if left_avg > right_avg else -math.pi / 4

    def start_avoidance_maneuver(self):
        """Initialize avoidance maneuver for follower"""
        avoid_angle = self.robot_pose[2] + self.find_clear_direction()
        avoid_distance = 1.5  # Smaller avoidance distance for followers
        
        # Store avoidance state
        self.avoidance_active = True
        self.avoidance_start_pose = [self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]]
        
        # Set avoidance target
        self.avoidance_target = [
            self.robot_pose[0] + avoid_distance * math.cos(avoid_angle),
            self.robot_pose[1] + avoid_distance * math.sin(avoid_angle)
        ]
        
        self.get_logger().info(f'{self.robot_name}: Starting avoidance - direction: {avoid_angle:.2f} rad')

    def should_continue_avoidance(self):
        """Check if avoidance should continue"""
        if self.avoidance_start_pose is None:
            return False
        
        # Calculate distance traveled since avoidance started
        dx = self.robot_pose[0] - self.avoidance_start_pose[0]
        dy = self.robot_pose[1] - self.avoidance_start_pose[1]
        distance_traveled = math.sqrt(dx*dx + dy*dy)
        
        # Continue if distance threshold hasn't been met
        should_continue = (distance_traveled < self.avoidance_min_distance)
        
        if not should_continue:
            self.get_logger().info(f'{self.robot_name}: Ending avoidance - Distance: {distance_traveled:.1f}m')
        
        return should_continue

    def end_avoidance_maneuver(self):
        """End avoidance maneuver"""
        self.avoidance_active = False
        self.avoidance_start_pose = None
        self.avoidance_target = None

    def control_loop(self):
        if not self.leader_received or not self.robot_received:
            return
        
        # Check for obstacles using ultrasonic sensor
        obstacle_ahead, dist = self.check_front_obstacle()
        
        # Emergency stop if too close
        if obstacle_ahead and dist < self.safe_distance:
            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            self.get_logger().warn(f'{self.robot_name}: Emergency stop! Obstacle at {dist:.2f}m')
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
            target_x = self.avoidance_target[0]
            target_y = self.avoidance_target[1]
            speed_factor = 0.4  # Reduced speed during avoidance
            self.get_logger().debug(f'{self.robot_name}: Following avoidance target: ({target_x:.2f}, {target_y:.2f})')
        else:
            # Use formation target (virtual leader position)
            cos_t, sin_t = math.cos(self.leader_pose[2]), math.sin(self.leader_pose[2])
            target_x = self.leader_pose[0] + self.offset_x*cos_t - self.offset_y*sin_t
            target_y = self.leader_pose[1] + self.offset_x*sin_t + self.offset_y*cos_t
            speed_factor = 1.0
        
        # Calculate errors
        dx = target_x - self.robot_pose[0]
        dy = target_y - self.robot_pose[1]
        
        # Position error magnitude
        ev = math.sqrt(dx*dx + dy*dy)
        
        # If error is very small and not avoiding obstacles, stop
        if ev < self.stop_threshold and not self.avoidance_active:
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            return
        
        # Angular error (pointing toward target)
        ew = math.atan2(dy, dx) - self.robot_pose[2]
        ew = math.atan2(math.sin(ew), math.cos(ew))  # Normalize
        
        # Calculate derivative terms
        dev_dt = (ev - self.ev_prev) / self.dt
        dew_dt = (ew - self.ew_prev) / self.dt
        
        # Update integral terms
        self.integral_ev += ev * self.dt
        self.integral_ew += ew * self.dt
        
        # Anti-windup: limit integral terms
        max_integral = 5.0
        self.integral_ev = max(-max_integral, min(max_integral, self.integral_ev))
        self.integral_ew = max(-max_integral, min(max_integral, self.integral_ew))
        
        # PID control
        # Linear velocity PID
        p_linear = self.kvp * ev
        i_linear = self.kvi * self.integral_ev
        d_linear = self.kvd * dev_dt
        vr = p_linear + i_linear + d_linear
        
        # Angular velocity PID
        p_angular = self.kwp * ew
        i_angular = self.kwi * self.integral_ew
        d_angular = self.kwd * dew_dt
        wr = p_angular + i_angular + d_angular
        
        # Store current errors for next iteration
        self.ev_prev = ev
        self.ew_prev = ew
        
        # Apply speed factor and limits
        vr = max(-0.5, min(0.5, vr))
        wr = max(-2.5, min(2.5, wr))
        vr *= speed_factor
        
        # Publish velocity
        vel_msg = Twist()
        vel_msg.linear.x = vr
        vel_msg.angular.z = wr
        self.vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = FormationFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        vel_msg = Twist()
        node.vel_pub.publish(vel_msg)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()