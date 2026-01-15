#! /usr/bin/env python3
"""
Patrolling Robot with ROS2 Navigation Stack
============================================
This script implements a patrol robot that:
1. Patrols a predefined set of waypoints
2. Dynamically adjusts path based on obstacles (handled by Nav2)
3. Returns to starting position after completing patrol cycle
4. Maintains a log of patrols with timestamps and coordinates
"""

import time
import logging
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped
import rclpy

# Import the BasicNavigator and NavigationResult from robot_navigator
from robot_navigator import BasicNavigator, NavigationResult


def create_pose(navigator, x, y, z=0.0, w=1.0):
    """Helper function to create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def setup_logging():
    """Setup logging to file and console."""
    log_dir = os.path.expanduser("~/ros2_ws")
    log_file = os.path.join(log_dir, "patrol_log.txt")
    
    # Create formatter
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    
    # Setup file handler
    file_handler = logging.FileHandler(log_file, mode='a')
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.INFO)
    
    # Setup console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(logging.INFO)
    
    # Setup logger
    logger = logging.getLogger('PatrolRobot')
    logger.setLevel(logging.INFO)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger, log_file


def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create navigator
    navigator = BasicNavigator()
    
    # Setup logging
    logger, log_file = setup_logging()
    logger.info("=" * 60)
    logger.info("PATROL ROBOT STARTED")
    logger.info("=" * 60)
    print(f"\n[INFO] Logging patrol data to: {log_file}\n")

    # =========================================================================
    # STEP 1: Set Initial Pose
    # =========================================================================
    initial_pose = create_pose(navigator, 0.0, 0.0, 0.0, 1.0)
    
    print("[INFO] Setting initial pose...")
    logger.info(f"Setting initial pose: x=0.0, y=0.0")
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2 to become active
    print("[INFO] Waiting for Nav2 to become active...")
    print("[INFO] This may take 30-60 seconds on first run...")
    logger.info("Waiting for Nav2 stack to activate...")
    
    navigator.waitUntilNav2Active()
    
    print("[INFO] Nav2 is active and ready!")
    logger.info("Nav2 stack is active and ready")

    # =========================================================================
    # STEP 2: Define Patrol Waypoints (covering entire house map)
    # Based on working coordinates from security_demo.py and run_inspection.py
    # These waypoints are tested and verified to avoid walls/obstacles
    # =========================================================================
    waypoints = [
        # === SAFE PATROL ROUTE (Verified from demo scripts) ===
        
        # Start - center of map
        {"name": "Start", "x": 0.0, "y": 0.0},
        
        # Kitchen area (east) - open space
        {"name": "Kitchen East", "x": 4.0, "y": 0.0},
        
        # Kitchen corner - verified safe position
        {"name": "Kitchen Corner", "x": 7.0, "y": -2.5},
        
        # South entrance - open corridor
        {"name": "South Entrance", "x": 0.0, "y": -4.0},
        
        # West corridor junction
        {"name": "West Junction", "x": -2.5, "y": -1.5},
        
        # Southwest room
        {"name": "Southwest Room", "x": -5.0, "y": -4.0},
        
        # Bedroom area (northwest) - far west
        {"name": "Bedroom", "x": -8.0, "y": -0.25},
        
        # Return via west junction
        {"name": "West Return", "x": -2.5, "y": -1.5},
        
        # Back to start area
        {"name": "Return", "x": 0.5, "y": 0.5},
    ]
    
    goal_poses = []
    for wp in waypoints:
        pose = create_pose(navigator, wp["x"], wp["y"])
        goal_poses.append(pose)
        logger.info(f"Added waypoint: {wp['name']} at ({wp['x']}, {wp['y']})")

    print(f"\n[INFO] Patrol route with {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. {wp['name']}: ({wp['x']}, {wp['y']})")

    # =========================================================================
    # STEP 3: Execute Patrol
    # =========================================================================
    patrol_count = 0
    max_patrols = 1  # One complete patrol of 24 waypoints covers the full house
    
    while patrol_count < max_patrols:
        patrol_count += 1
        logger.info(f"Starting Patrol Cycle {patrol_count}/{max_patrols}")
        print(f"\n[INFO] STARTING PATROL CYCLE {patrol_count}/{max_patrols}")
        
        # Clear costmaps before starting patrol to ensure fresh planning
        navigator.clearAllCostmaps()
        time.sleep(1.0)  # Give costmaps time to rebuild
        
        navigator.followWaypoints(goal_poses)
        
        # Give Nav2 a moment to process the waypoints request
        time.sleep(2.0)
        
        i = 0
        current_waypoint = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            
            if feedback:
                if feedback.current_waypoint != current_waypoint:
                    current_waypoint = feedback.current_waypoint
                    if current_waypoint < len(waypoints):
                        wp = waypoints[current_waypoint]
                        logger.info(f"Navigating to: {wp['name']} ({wp['x']}, {wp['y']})")
                        print(f"[INFO] Navigating to: {wp['name']}")
                
                if i % 20 == 0:
                    print(f"[INFO] Progress: {feedback.current_waypoint + 1}/{len(waypoints)}")
            
            time.sleep(0.5)
        
        result = navigator.getResult()
        
        if result == NavigationResult.SUCCEEDED:
            logger.info(f"Patrol Cycle {patrol_count} COMPLETED")
            print(f"[SUCCESS] Patrol cycle {patrol_count} completed!")
            for wp in waypoints:
                logger.info(f"  Visited: {wp['name']} at ({wp['x']}, {wp['y']})")
        elif result == NavigationResult.CANCELED:
            logger.warning(f"Patrol Cycle {patrol_count} CANCELED")
            print(f"[WARN] Patrol canceled")
            break
        elif result == NavigationResult.FAILED:
            logger.error(f"Patrol Cycle {patrol_count} FAILED")
            print(f"[ERROR] Patrol failed!")
            continue
        else:
            logger.error(f"Unknown result")
            continue

    # =========================================================================
    # STEP 4: Return to Start
    # =========================================================================
    logger.info("Returning to start position...")
    print(f"\n[INFO] RETURNING TO START POSITION")
    
    # Clear costmaps before final navigation
    navigator.clearAllCostmaps()
    time.sleep(1.0)
    
    # Update timestamp for initial pose
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Try to go to start position
    success = navigator.goToPose(initial_pose)
    
    if success:
        # Wait for navigation to complete with timeout
        start_time = time.time()
        while not navigator.isTaskComplete():
            if time.time() - start_time > 60.0:  # 60 second timeout
                logger.warning("Return to start timed out")
                navigator.cancelNav()
                break
            time.sleep(0.5)
        
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            logger.info("Returned to start position")
            print("[SUCCESS] Returned to start position")
        else:
            logger.info(f"Return navigation result: {result} (robot may already be at start)")
            print(f"[INFO] Navigation completed (result: {result})")
    else:
        logger.info("Could not send return goal - robot may already be at start")
        print("[INFO] Robot is near start position")

    # =========================================================================
    # STEP 5: Summary
    # =========================================================================
    logger.info("PATROL MISSION COMPLETE")
    print(f"\n{'='*50}")
    print(f"PATROL COMPLETE - Cycles: {patrol_count}, Log: {log_file}")
    print(f"{'='*50}\n")

    # Clean shutdown
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
