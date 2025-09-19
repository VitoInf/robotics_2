#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
import numpy as np


class CubicTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('cubic_trajectory_generator')
        
        # Publisher
        self.traj_pub = self.create_publisher(Pose2D, '/desired_trajectory', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        
        # Current Time for trajectory
        self.t = 0.0
        
        # Setup trajectory
        self.setup_spline()
        self.get_logger().info('Cubic spline trajectory started')

    def setup_spline(self):
        """Setup cubic spline through waypoints"""
        self.waypoints = np.array([[9.5, 0.0], [6.0, 2.0], [4.0, 3.0], [2.0, 0.0]])
        self.times = np.array([0.0, 15.0, 30.0, 40.0])
        
        # Calculate coefficients for each segment
        self.coeffs_x = []
        self.coeffs_y = []
        
        for i in range(len(self.waypoints) - 1):
            dt = self.times[i+1] - self.times[i]
            x0, y0 = self.waypoints[i]
            x1, y1 = self.waypoints[i+1]
            
            # Cubic: p(t) = a*t^3 + b*t^2 + c*t + d
            # Simple cubic trajectory with zero derivatives (velocity) at endpoints
            a_x = (2*x0 - 2*x1) / (dt**3)
            b_x = (3*x1 - 3*x0) / (dt**2)
            c_x = 0.0
            d_x = x0
            
            a_y = (2*y0 - 2*y1) / (dt**3)
            b_y = (3*y1 - 3*y0) / (dt**2)
            c_y = 0.0
            d_y = y0
            
            self.coeffs_x.append([a_x, b_x, c_x, d_x])
            self.coeffs_y.append([a_y, b_y, c_y, d_y])

    def evaluate_spline(self, t):
        """Evaluate spline at time t"""
        # Find segment
        segment = 0
        for i in range(len(self.times) - 1):
            if t <= self.times[i+1]:
                segment = i
                break
        
        t_local = t - self.times[segment]
        
        # Evaluate cubic
        cx, cy = self.coeffs_x[segment], self.coeffs_y[segment]
        x = cx[0]*t_local**3 + cx[1]*t_local**2 + cx[2]*t_local + cx[3]
        y = cy[0]*t_local**3 + cy[1]*t_local**2 + cy[2]*t_local + cy[3]
        
        # Calculate heading
        dx = 3*cx[0]*t_local**2 + 2*cx[1]*t_local + cx[2]
        dy = 3*cy[0]*t_local**2 + 2*cy[1]*t_local + cy[2]
        theta = math.atan2(dy, dx)
        
        return x, y, theta

    def publish_trajectory(self):
        if self.t > 40.0:
            return
        
        # Generate trajectory point
        x, y, theta = self.evaluate_spline(self.t)
        
        # Publish
        traj = Pose2D()
        traj.x = x
        traj.y = y
        traj.theta = theta
        self.traj_pub.publish(traj)
        
        self.t += 0.1


def main():
    rclpy.init()
    node = CubicTrajectoryGenerator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()