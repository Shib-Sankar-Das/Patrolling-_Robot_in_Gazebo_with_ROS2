#!/usr/bin/env python3
"""
Obstacle Avoidance Safety Node for ROS2
========================================
This node provides a safety layer for the patrol robot by:
1. Monitoring LIDAR scan data for obstacles
2. Publishing emergency stop commands when obstacles are too close
3. Providing sector-based obstacle detection (front, left, right, back)
4. Integrating with Nav2 by clearing costmaps when stuck

Inspired by:
- https://github.com/vinay06vinay/Turtlebot3-Obstacle-Avoidance-ROS2
- https://medium.com/@kabilankb2003/building-a-simple-ros2-object-avoidance-robot-using-python

Author: Patrol Robot Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
import numpy as np
import math


class ObstacleAvoidanceNode(Node):
    """
    A safety node that monitors LIDAR data and provides obstacle avoidance
    capabilities as a backup to Nav2's built-in obstacle handling.
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Declare parameters with default values
        # Note: Lower thresholds for indoor navigation where walls are close
        self.declare_parameter('safe_distance', 0.25)  # Reduced from 0.35 - walls are typically at ~0.3m
        self.declare_parameter('warning_distance', 0.4)  # Reduced from 0.6 - less sensitive
        self.declare_parameter('critical_distance', 0.15)  # Reduced from 0.25 - only immediate collision
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enabled', True)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('avoidance_mode', 'monitor')  # 'monitor', 'active', 'autonomous'
        
        # Get parameters
        self.safe_distance = self.get_parameter('safe_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enabled = self.get_parameter('enabled').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.avoidance_mode = self.get_parameter('avoidance_mode').value
        
        # State variables
        self.obstacle_detected = False
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.back_clear = True
        self.min_distance_front = float('inf')
        self.min_distance_left = float('inf')
        self.min_distance_right = float('inf')
        self.emergency_stop_active = False
        self.last_scan = None
        
        # Define sector angles (in degrees, 0 is forward)
        # LIDAR typically has 360 readings for 360 degrees
        self.front_sector = (-30, 30)    # Front: -30° to +30°
        self.left_sector = (30, 90)      # Left: 30° to 90°
        self.right_sector = (-90, -30)   # Right: -90° to -30°
        self.back_sector = (150, 210)    # Back: 150° to 210° (or -150° to -210°)
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.lidar_callback,
            sensor_qos
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.obstacle_status_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_info_pub = self.create_publisher(String, '/obstacle_info', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Service clients for costmap clearing
        self.clear_global_costmap_client = self.create_client(
            Empty, '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.create_client(
            Empty, '/local_costmap/clear_entirely_local_costmap'
        )
        
        # Timer for periodic status publishing
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        # Timer for stuck detection (if robot hasn't moved in a while with obstacles)
        self.stuck_timer = None
        self.stuck_counter = 0
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Obstacle Avoidance Node Started')
        self.get_logger().info(f'  Mode: {self.avoidance_mode}')
        self.get_logger().info(f'  Safe distance: {self.safe_distance}m')
        self.get_logger().info(f'  Warning distance: {self.warning_distance}m')
        self.get_logger().info(f'  Critical distance: {self.critical_distance}m')
        self.get_logger().info('=' * 50)
    
    def angle_to_index(self, angle, num_readings, angle_min, angle_increment):
        """Convert an angle (in degrees) to a LIDAR reading index."""
        angle_rad = math.radians(angle)
        # Normalize angle to LIDAR frame
        index = int((angle_rad - angle_min) / angle_increment)
        return index % num_readings
    
    def get_sector_min_distance(self, ranges, start_angle, end_angle, angle_min, angle_increment, num_readings):
        """Get the minimum distance in a sector."""
        start_idx = self.angle_to_index(start_angle, num_readings, angle_min, angle_increment)
        end_idx = self.angle_to_index(end_angle, num_readings, angle_min, angle_increment)
        
        # Handle wrap-around
        if start_idx <= end_idx:
            sector_ranges = ranges[start_idx:end_idx+1]
        else:
            sector_ranges = list(ranges[start_idx:]) + list(ranges[:end_idx+1])
        
        # Filter out invalid readings (inf, nan, 0)
        valid_ranges = [r for r in sector_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        
        if valid_ranges:
            return min(valid_ranges)
        return float('inf')
    
    def lidar_callback(self, msg):
        """Process incoming LIDAR data and detect obstacles."""
        if not self.enabled:
            return
        
        self.last_scan = msg
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Get minimum distances for each sector
        self.min_distance_front = self.get_sector_min_distance(
            ranges, self.front_sector[0], self.front_sector[1],
            angle_min, angle_increment, num_readings
        )
        
        self.min_distance_left = self.get_sector_min_distance(
            ranges, self.left_sector[0], self.left_sector[1],
            angle_min, angle_increment, num_readings
        )
        
        self.min_distance_right = self.get_sector_min_distance(
            ranges, self.right_sector[0], self.right_sector[1],
            angle_min, angle_increment, num_readings
        )
        
        # Update sector status (convert to Python bool to avoid numpy bool issues)
        self.front_clear = bool(self.min_distance_front > self.safe_distance)
        self.left_clear = bool(self.min_distance_left > self.safe_distance)
        self.right_clear = bool(self.min_distance_right > self.safe_distance)
        
        # Overall obstacle detection
        min_overall = min(self.min_distance_front, self.min_distance_left, self.min_distance_right)
        self.obstacle_detected = bool(min_overall < self.warning_distance)
        
        # Check for critical distance (emergency stop)
        if self.min_distance_front < self.critical_distance:
            if not self.emergency_stop_active:
                self.get_logger().warn(f'EMERGENCY STOP! Obstacle at {self.min_distance_front:.2f}m')
                self.emergency_stop_active = True
            self.send_emergency_stop()
        else:
            if self.emergency_stop_active:
                self.get_logger().info('Emergency stop cleared - path is safe')
            self.emergency_stop_active = False
        
        # If in active avoidance mode, take control of robot movement
        if self.avoidance_mode == 'active' and not self.front_clear:
            self.avoid_obstacle()
        elif self.avoidance_mode == 'autonomous':
            self.autonomous_navigation()
    
    def send_emergency_stop(self):
        """Send emergency stop command."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Publish emergency stop status
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
    
    def avoid_obstacle(self):
        """Active obstacle avoidance - turn away from obstacles."""
        twist = Twist()
        
        if self.min_distance_front < self.safe_distance:
            # Stop forward motion
            twist.linear.x = 0.0
            
            # Decide which way to turn based on which side has more space
            if self.min_distance_left > self.min_distance_right:
                # Turn left
                twist.angular.z = self.angular_speed
                self.get_logger().info('Avoiding obstacle: turning left')
            else:
                # Turn right  
                twist.angular.z = -self.angular_speed
                self.get_logger().info('Avoiding obstacle: turning right')
            
            self.cmd_vel_pub.publish(twist)
    
    def autonomous_navigation(self):
        """Fully autonomous obstacle avoidance mode."""
        twist = Twist()
        
        if self.min_distance_front < self.safe_distance:
            # Obstacle ahead - stop and turn
            twist.linear.x = 0.0
            
            if self.min_distance_left > self.min_distance_right:
                twist.angular.z = self.angular_speed
            else:
                twist.angular.z = -self.angular_speed
        elif self.min_distance_front < self.warning_distance:
            # Obstacle nearby - slow down and prepare to turn
            twist.linear.x = self.linear_speed * 0.5
            
            if self.min_distance_left > self.min_distance_right:
                twist.angular.z = self.angular_speed * 0.3
            else:
                twist.angular.z = -self.angular_speed * 0.3
        else:
            # Path is clear - move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
    
    def publish_status(self):
        """Publish obstacle status periodically."""
        # Publish boolean obstacle detection status (ensure Python bool type)
        status_msg = Bool()
        status_msg.data = bool(self.obstacle_detected)
        self.obstacle_status_pub.publish(status_msg)
        
        # Publish detailed obstacle info
        info_msg = String()
        info_msg.data = (
            f"front:{self.min_distance_front:.2f}m,"
            f"left:{self.min_distance_left:.2f}m,"
            f"right:{self.min_distance_right:.2f}m,"
            f"clear:{'F' if self.front_clear else 'X'}"
            f"{'L' if self.left_clear else 'X'}"
            f"{'R' if self.right_clear else 'X'}"
        )
        self.obstacle_info_pub.publish(info_msg)
        
        # Publish emergency stop status (ensure Python bool type)
        estop_msg = Bool()
        estop_msg.data = bool(self.emergency_stop_active)
        self.emergency_stop_pub.publish(estop_msg)
        
        # Log periodic status if obstacles are detected
        if self.obstacle_detected and not self.emergency_stop_active:
            self.get_logger().debug(
                f'Obstacle status - Front: {self.min_distance_front:.2f}m, '
                f'Left: {self.min_distance_left:.2f}m, '
                f'Right: {self.min_distance_right:.2f}m'
            )
    
    def clear_costmaps(self):
        """Clear both global and local costmaps."""
        self.get_logger().info('Clearing costmaps...')
        
        # Clear global costmap
        if self.clear_global_costmap_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.clear_global_costmap_client.call_async(req)
        
        # Clear local costmap
        if self.clear_local_costmap_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.clear_local_costmap_client.call_async(req)
    
    def get_obstacle_status(self):
        """Return current obstacle status for external queries."""
        return {
            'detected': self.obstacle_detected,
            'front_clear': self.front_clear,
            'left_clear': self.left_clear,
            'right_clear': self.right_clear,
            'min_front': self.min_distance_front,
            'min_left': self.min_distance_left,
            'min_right': self.min_distance_right,
            'emergency_stop': self.emergency_stop_active
        }


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Obstacle Avoidance Node shutting down...')
    finally:
        # Send stop command before shutting down
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
