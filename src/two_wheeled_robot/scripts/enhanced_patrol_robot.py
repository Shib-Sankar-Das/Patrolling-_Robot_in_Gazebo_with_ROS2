#!/usr/bin/env python3
"""
Enhanced Patrol Robot with Dynamic Obstacle Avoidance
=====================================================
This script implements an intelligent patrol robot that:
1. Navigates through predefined waypoints using Nav2
2. Monitors for obstacles via the obstacle_avoidance node
3. Implements recovery behaviors when stuck
4. Provides detailed logging and status reporting
5. Gracefully handles failures and restarts navigation

Improvements from reference projects:
- Better error handling and recovery (inspired by Patrol-Robot)
- Obstacle awareness integration (inspired by Turtlebot3-Obstacle-Avoidance)
- Smoother velocity control and waypoint transitions
- Robust state machine for patrol management

Author: Patrol Robot Team
"""

import time
import logging
import os
from datetime import datetime
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Import the BasicNavigator and NavigationResult from robot_navigator
from robot_navigator import BasicNavigator, NavigationResult


class PatrolState(Enum):
    """States for the patrol state machine."""
    INITIALIZING = "initializing"
    WAITING_FOR_NAV2 = "waiting_for_nav2"
    PATROLLING = "patrolling"
    NAVIGATING_TO_WAYPOINT = "navigating_to_waypoint"
    OBSTACLE_DETECTED = "obstacle_detected"
    RECOVERING = "recovering"
    RETURNING_HOME = "returning_home"
    COMPLETED = "completed"
    ERROR = "error"


class EnhancedPatrolRobot(Node):
    """
    Enhanced patrol robot with obstacle avoidance integration and recovery behaviors.
    """
    
    def __init__(self, navigator):
        super().__init__('enhanced_patrol_robot')
        
        self.navigator = navigator
        self.state = PatrolState.INITIALIZING
        
        # Configuration parameters
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('waypoint_timeout', 120.0)  # seconds
        self.declare_parameter('patrol_cycles', 1)
        self.declare_parameter('pause_at_waypoint', 2.0)  # seconds
        
        self.max_recovery_attempts = self.get_parameter('max_recovery_attempts').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.patrol_cycles = self.get_parameter('patrol_cycles').value
        self.pause_at_waypoint = self.get_parameter('pause_at_waypoint').value
        
        # Obstacle monitoring
        self.obstacle_detected = False
        self.emergency_stop = False
        self.obstacle_info = ""
        
        # QoS for obstacle topics
        qos = QoSProfile(depth=10)
        
        # Subscribe to obstacle avoidance node outputs
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/obstacle_detected',
            self.obstacle_callback,
            qos
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            qos
        )
        
        self.obstacle_info_sub = self.create_subscription(
            String,
            '/obstacle_info',
            self.obstacle_info_callback,
            qos
        )
        
        # Velocity publisher for recovery maneuvers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Statistics
        self.waypoints_visited = 0
        self.recovery_count = 0
        self.failed_waypoints = []
        self.start_time = None
        
        # Setup logging
        self.logger, self.log_file = self.setup_logging()
        
    def setup_logging(self):
        """Setup logging to file and console."""
        log_dir = os.path.expanduser("~/ros2_ws")
        log_file = os.path.join(log_dir, "enhanced_patrol_log.txt")
        
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        
        file_handler = logging.FileHandler(log_file, mode='a')
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.INFO)
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        console_handler.setLevel(logging.INFO)
        
        logger = logging.getLogger('EnhancedPatrolRobot')
        logger.setLevel(logging.INFO)
        
        # Clear existing handlers
        logger.handlers = []
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger, log_file
    
    def obstacle_callback(self, msg):
        """Handle obstacle detection updates."""
        self.obstacle_detected = msg.data
    
    def emergency_stop_callback(self, msg):
        """Handle emergency stop updates."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.logger.warning("Emergency stop activated!")
    
    def obstacle_info_callback(self, msg):
        """Handle detailed obstacle info updates."""
        self.obstacle_info = msg.data
    
    def create_pose(self, x, y, z=0.0, w=1.0):
        """Helper function to create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose
    
    def backup_maneuver(self, duration=2.0, speed=-0.15):
        """Execute a backup maneuver to clear obstacle."""
        self.logger.info(f"Executing backup maneuver for {duration}s")
        
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.emergency_stop:  # Only if safe to move
                self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
    
    def spin_maneuver(self, duration=3.0, angular_speed=0.5):
        """Execute a spin maneuver to scan surroundings."""
        self.logger.info(f"Executing spin maneuver for {duration}s")
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.emergency_stop:
                self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
    
    def recovery_behavior(self):
        """Execute recovery behaviors when stuck."""
        self.state = PatrolState.RECOVERING
        self.recovery_count += 1
        
        self.logger.warning(f"Executing recovery behavior (attempt {self.recovery_count})")
        
        # Cancel current navigation
        self.navigator.cancelNav()
        time.sleep(1.0)
        
        # Clear costmaps
        self.logger.info("Clearing costmaps...")
        self.navigator.clearAllCostmaps()
        time.sleep(1.0)
        
        # Backup a bit
        self.backup_maneuver(duration=1.5, speed=-0.1)
        
        # Spin to clear and re-scan
        self.spin_maneuver(duration=2.0, angular_speed=0.4)
        
        # Clear costmaps again after moving
        self.navigator.clearAllCostmaps()
        time.sleep(1.0)
        
        self.logger.info("Recovery behavior completed")
    
    def wait_for_clear_path(self, timeout=10.0):
        """Wait for obstacle to clear with timeout."""
        start_time = time.time()
        
        while self.obstacle_detected and (time.time() - start_time) < timeout:
            self.logger.info(f"Waiting for obstacle to clear... {self.obstacle_info}")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return not self.obstacle_detected
    
    def navigate_to_waypoint(self, waypoint, waypoint_index, total_waypoints):
        """Navigate to a single waypoint with obstacle handling."""
        name = waypoint["name"]
        x, y = waypoint["x"], waypoint["y"]
        
        self.logger.info(f"Navigating to waypoint {waypoint_index + 1}/{total_waypoints}: {name} ({x}, {y})")
        print(f"\n[NAV] Going to: {name} ({x}, {y})")
        
        pose = self.create_pose(x, y)
        
        # Check if path is clear before starting
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.emergency_stop:
            self.logger.warning("Emergency stop active - waiting for clear path")
            if not self.wait_for_clear_path(timeout=15.0):
                return False
        
        # Start navigation
        success = self.navigator.goToPose(pose)
        if not success:
            self.logger.error(f"Failed to send goal to {name}")
            return False
        
        # Monitor navigation progress
        start_time = time.time()
        last_progress_time = start_time
        recovery_attempts = 0
        
        while not self.navigator.isTaskComplete():
            # Check for timeout
            elapsed = time.time() - start_time
            if elapsed > self.waypoint_timeout:
                self.logger.error(f"Timeout navigating to {name}")
                self.navigator.cancelNav()
                return False
            
            # Process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check for emergency stop
            if self.emergency_stop:
                self.logger.warning(f"Emergency stop during navigation to {name}")
                self.navigator.cancelNav()
                time.sleep(0.5)
                
                if recovery_attempts < self.max_recovery_attempts:
                    recovery_attempts += 1
                    self.recovery_behavior()
                    
                    # Retry navigation
                    if not self.wait_for_clear_path(timeout=10.0):
                        return False
                    
                    pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    success = self.navigator.goToPose(pose)
                    if not success:
                        return False
                    last_progress_time = time.time()
                else:
                    return False
            
            # Get feedback
            feedback = self.navigator.getFeedback()
            if feedback:
                # Check for progress (to detect stuck situations)
                if hasattr(feedback, 'distance_remaining'):
                    distance = feedback.distance_remaining
                    if distance > 0:
                        last_progress_time = time.time()
            
            # Check if stuck (no progress for 30 seconds)
            if time.time() - last_progress_time > 30.0:
                self.logger.warning(f"Robot may be stuck navigating to {name}")
                if recovery_attempts < self.max_recovery_attempts:
                    recovery_attempts += 1
                    self.recovery_behavior()
                    
                    pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    success = self.navigator.goToPose(pose)
                    if not success:
                        return False
                    last_progress_time = time.time()
                else:
                    self.navigator.cancelNav()
                    return False
            
            time.sleep(0.2)
        
        # Check result
        result = self.navigator.getResult()
        
        if result == NavigationResult.SUCCEEDED:
            self.logger.info(f"Reached waypoint: {name}")
            print(f"[OK] Reached: {name}")
            self.waypoints_visited += 1
            return True
        elif result == NavigationResult.CANCELED:
            self.logger.warning(f"Navigation to {name} was canceled")
            return False
        else:
            self.logger.error(f"Failed to reach {name} - result: {result}")
            return False
    
    def patrol(self, waypoints):
        """Execute the patrol mission."""
        self.state = PatrolState.PATROLLING
        self.start_time = time.time()
        
        self.logger.info("=" * 60)
        self.logger.info("ENHANCED PATROL ROBOT STARTING")
        self.logger.info(f"Waypoints: {len(waypoints)}, Cycles: {self.patrol_cycles}")
        self.logger.info("=" * 60)
        
        for cycle in range(self.patrol_cycles):
            self.logger.info(f"\n--- Starting Patrol Cycle {cycle + 1}/{self.patrol_cycles} ---")
            print(f"\n{'='*50}")
            print(f"PATROL CYCLE {cycle + 1}/{self.patrol_cycles}")
            print(f"{'='*50}")
            
            # Clear costmaps before each cycle
            self.navigator.clearAllCostmaps()
            time.sleep(1.0)
            
            for i, waypoint in enumerate(waypoints):
                self.state = PatrolState.NAVIGATING_TO_WAYPOINT
                
                success = self.navigate_to_waypoint(waypoint, i, len(waypoints))
                
                if success:
                    # Pause at waypoint
                    if self.pause_at_waypoint > 0:
                        self.logger.info(f"Pausing at {waypoint['name']} for {self.pause_at_waypoint}s")
                        time.sleep(self.pause_at_waypoint)
                else:
                    self.failed_waypoints.append((cycle, waypoint['name']))
                    self.logger.warning(f"Failed waypoint: {waypoint['name']} - continuing to next")
                    
                    # Try to recover and continue
                    if len(self.failed_waypoints) > len(waypoints) // 2:
                        self.logger.error("Too many failed waypoints - aborting patrol")
                        self.state = PatrolState.ERROR
                        return False
            
            self.logger.info(f"Patrol Cycle {cycle + 1} completed")
        
        self.state = PatrolState.COMPLETED
        return True
    
    def return_home(self, home_pose):
        """Return to starting position."""
        self.state = PatrolState.RETURNING_HOME
        self.logger.info("Returning to home position...")
        print("\n[INFO] Returning home...")
        
        home_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(home_pose)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)
        
        result = self.navigator.getResult()
        return result == NavigationResult.SUCCEEDED
    
    def print_summary(self):
        """Print patrol mission summary."""
        duration = time.time() - self.start_time if self.start_time else 0
        
        summary = f"""
{'='*60}
PATROL MISSION SUMMARY
{'='*60}
Status: {self.state.value}
Duration: {duration:.1f} seconds ({duration/60:.1f} minutes)
Waypoints visited: {self.waypoints_visited}
Recovery attempts: {self.recovery_count}
Failed waypoints: {len(self.failed_waypoints)}
"""
        if self.failed_waypoints:
            summary += f"Failed locations: {[wp[1] for wp in self.failed_waypoints]}\n"
        
        summary += f"Log file: {self.log_file}\n"
        summary += "=" * 60
        
        print(summary)
        self.logger.info(summary)


def define_patrol_waypoints():
    """
    Define the patrol waypoints for the house map.
    These waypoints are based on the working security_demo.py and run_inspection.py
    with positions verified to avoid walls and obstacles.
    """
    return [
        # === SAFE PATROL ROUTE (Tested coordinates) ===
        # Based on security_demo.py and run_inspection.py
        
        # Start from center
        {"name": "Start Position", "x": 0.0, "y": 0.0},
        
        # Move to kitchen area (east)
        {"name": "Kitchen Entrance", "x": 4.0, "y": 0.0},
        {"name": "Kitchen Corner", "x": 7.0, "y": -2.5},
        
        # Move to south entrance
        {"name": "South Entrance", "x": 0.0, "y": -4.0},
        
        # Move to west hallway
        {"name": "West Hallway", "x": -2.5, "y": -1.5},
        
        # Move to southwest corner
        {"name": "Southwest Corner", "x": -5.0, "y": -4.0},
        
        # Return via west hallway
        {"name": "West Return", "x": -2.5, "y": -1.5},
        
        # Move to bedroom area (northwest)
        {"name": "Bedroom Area", "x": -8.0, "y": -0.25},
        
        # Additional safe waypoints for better coverage
        {"name": "North Central", "x": 0.0, "y": 2.0},
        {"name": "Fitness Area", "x": 2.0, "y": 2.0},
        
        # Return to near center (not exactly at 0,0 to avoid nav issues)
        {"name": "Return Center", "x": 0.5, "y": 0.5},
    ]


def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create navigator
    navigator = BasicNavigator()
    
    # Create enhanced patrol robot
    patrol_robot = EnhancedPatrolRobot(navigator)
    
    print("\n" + "=" * 60)
    print("ENHANCED PATROL ROBOT WITH OBSTACLE AVOIDANCE")
    print("=" * 60)
    print(f"Log file: {patrol_robot.log_file}")
    print("=" * 60 + "\n")
    
    # Set initial pose
    initial_pose = patrol_robot.create_pose(0.0, 0.0, 0.0, 1.0)
    
    print("[INFO] Setting initial pose...")
    patrol_robot.logger.info("Setting initial pose: x=0.0, y=0.0")
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2 to become active
    print("[INFO] Waiting for Nav2 to become active...")
    print("[INFO] This may take 30-60 seconds on first run...")
    patrol_robot.logger.info("Waiting for Nav2 stack to activate...")
    
    navigator.waitUntilNav2Active()
    
    print("[INFO] Nav2 is active and ready!")
    patrol_robot.logger.info("Nav2 stack is active and ready")
    
    # Define waypoints
    waypoints = define_patrol_waypoints()
    
    print(f"\n[INFO] Patrol route with {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. {wp['name']}: ({wp['x']}, {wp['y']})")
    
    # Execute patrol
    print("\n[INFO] Starting patrol mission...")
    patrol_robot.logger.info("Starting patrol mission")
    
    try:
        success = patrol_robot.patrol(waypoints)
        
        if success:
            # Return home
            patrol_robot.return_home(initial_pose)
        
    except KeyboardInterrupt:
        print("\n[INFO] Patrol interrupted by user")
        patrol_robot.logger.info("Patrol interrupted by user")
        try:
            navigator.cancelNav()
        except:
            pass
    except Exception as e:
        print(f"\n[ERROR] Patrol failed: {e}")
        patrol_robot.logger.error(f"Patrol failed: {e}")
    finally:
        # Print summary
        patrol_robot.print_summary()
        
        # Clean shutdown - check if already shutdown
        try:
            patrol_robot.destroy_node()
        except:
            pass
        try:
            navigator.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
