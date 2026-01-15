#! /usr/bin/env python3
import time
import logging
from datetime import datetime
from enum import Enum  # Added this for the fix
from geometry_msgs.msg import PoseStamped
import rclpy

# --- FIX: Only import BasicNavigator, not TaskResult ---
from robot_navigator import BasicNavigator 

# --- FIX: Define TaskResult locally since it's missing in the helper file ---
class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

def main():
    # 1. Initialize ROS and the Navigator
    rclpy.init()
    navigator = BasicNavigator()

    # 2. Setup Logging
    log_file = "patrol_log.txt"
    logging.basicConfig(filename=log_file, level=logging.INFO, format='%(message)s')
    print(f"Logging patrol data to {log_file}...")
    logging.info(f"--- Patrol Started at {datetime.now()} ---")

    # 3. Set Initial Pose (Using the standard Gazebo start position)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # 4. Define Waypoints 
    goals = []
    
    # Waypoint 1
    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.header.stamp = navigator.get_clock().now().to_msg()
    wp1.pose.position.x = 2.0
    wp1.pose.position.y = 1.0
    wp1.pose.orientation.w = 1.0
    goals.append(wp1)

    # Waypoint 2
    wp2 = PoseStamped()
    wp2.header.frame_id = 'map'
    wp2.header.stamp = navigator.get_clock().now().to_msg()
    wp2.pose.position.x = 2.0
    wp2.pose.position.y = -1.0
    wp2.pose.orientation.w = 1.0
    goals.append(wp2)

    # 5. Start Patrolling
    print("Starting patrol...")
    navigator.followWaypoints(goals)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Executing current waypoint: {feedback.current_waypoint}')
            
            # Log periodic status
            now = datetime.now().strftime("%H:%M:%S")
            logging.info(f"[{now}] En route to waypoint {feedback.current_waypoint}")
            
        # Small sleep to prevent 100% CPU usage
        time.sleep(0.1)

    # 6. Check Result
    result = navigator.getResult()
    
    # Convert the raw result to our local Enum if needed, or compare directly
    if result == TaskResult.SUCCEEDED:
        print('Patrol complete! Returning to Start...')
        logging.info("Patrol waypoints complete. Returning to start.")
        
        # RETURN TO START 
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete():
            pass
        print("Returned to start position.")
        logging.info("Returned to start position.")
        
    elif result == TaskResult.CANCELED:
        print('Patrol was canceled')
    elif result == TaskResult.FAILED:
        print('Patrol failed!')
    else:
        print('Goal result unknown/failed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
