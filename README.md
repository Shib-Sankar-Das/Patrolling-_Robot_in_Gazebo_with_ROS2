# Patrolling Robot in Gazebo with ROS2

A comprehensive patrolling robot project using ROS2 Humble and Gazebo simulation. The robot autonomously patrols predefined waypoints, dynamically avoids obstacles, and maintains a log of visited locations.

## Project Features

✅ **Patrol Waypoints**: Robot navigates through predefined waypoints  
✅ **Dynamic Obstacle Avoidance**: Nav2 stack handles real-time path planning  
✅ **Return to Start**: Robot returns to starting position after patrol  
✅ **Patrol Logging**: Timestamps and coordinates logged to file  
✅ **Multiple Patrol Cycles**: Configurable number of patrol rounds

---

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic
- Nav2 Navigation Stack
- CycloneDDS (for reliable communication)

### Install Required Packages

```bash
# Install Nav2 and dependencies
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
sudo apt install -y ros-humble-xacro
```

---

## Quick Start Guide

### Step 1: Source the Environment

**IMPORTANT**: You must run this in EVERY new terminal you open!

```bash
source /home/shib/ros2_ws/setup_patrol_robot.sh
```

This script:
- Sources ROS2 Humble
- Sources your workspace
- Switches to CycloneDDS (fixes communication issues)
- Sets up Gazebo model paths

### Step 2: Launch the Simulation (Terminal 1)

```bash
# Source environment first!
source /home/shib/ros2_ws/setup_patrol_robot.sh

# Launch Gazebo + Nav2 + RViz
ros2 launch two_wheeled_robot house_world_v1.launch.py
```

**Wait for:**
- Gazebo window to open with the house and robot
- RViz window to show the map and robot
- Terminal shows "Activating..." messages for Nav2 nodes

⏱️ **This takes 30-60 seconds on first launch**

### Step 3: Run the Patrol Robot (Terminal 2)

Open a **NEW terminal**, then:

```bash
# CRITICAL: Source environment in this terminal too!
source /home/shib/ros2_ws/setup_patrol_robot.sh

# Run the patrol script
ros2 run two_wheeled_robot my_patrol_robot.py
```

**You should see:**
1. "Setting initial pose..."
2. "Waiting for Nav2 to become active..."
3. "Nav2 is active and ready!"
4. "STARTING PATROL CYCLE 1/2"
5. Robot moving to waypoints in Gazebo

---

## Troubleshooting

### Issue: "Publishing Initial Pose" hangs forever

**Cause**: DDS communication failure  
**Solution**: Make sure you sourced `setup_patrol_robot.sh` in BOTH terminals

```bash
# Verify CycloneDDS is active
echo $RMW_IMPLEMENTATION
# Should print: rmw_cyclonedds_cpp
```

### Issue: RViz shows "Fixed Frame [map] does not exist"

**Cause**: AMCL hasn't started or isn't localized  
**Solution**: Wait longer (30-60 seconds) after launch. The map frame appears after AMCL initializes.

### Issue: Robot not moving in Gazebo

**Cause**: Nav2 not fully activated  
**Solution**: 
1. Check if map is visible in RViz
2. Try setting initial pose manually in RViz (2D Pose Estimate button)
3. Wait for all Nav2 nodes to become active

### Issue: "Discarding message because queue is full"

**Cause**: FastDDS buffer overflow  
**Solution**: This is fixed by using CycloneDDS. Verify:
```bash
echo $CYCLONEDDS_URI
# Should show the path to cyclonedds.xml
```

### Issue: Gazebo crashes or runs slowly

**Solution**: 
```bash
# Kill any zombie processes
killall -9 gzserver gzclient ruby

# Reduce Gazebo physics update rate if needed
# or close other heavy applications
```

---

## File Structure

```
ros2_ws/
├── setup_patrol_robot.sh          # Environment setup script
├── patrol_log.txt                 # Patrol logs (created at runtime)
└── src/two_wheeled_robot/
    ├── config/
    │   ├── cyclonedds.xml         # DDS configuration
    │   └── ekf.yaml               # EKF localization config
    ├── launch/house_world/
    │   └── house_world_v1.launch.py
    ├── maps/house_world/
    │   └── house_world.yaml       # Map file
    ├── params/house_world/
    │   └── nav2_params.yaml       # Navigation parameters
    ├── scripts/
    │   ├── my_patrol_robot.py     # Main patrol script
    │   └── robot_navigator.py     # Navigation helper
    └── worlds/
        └── house.world            # Gazebo world file
```

---

## Customizing Waypoints

Edit the waypoints in `my_patrol_robot.py`:

```python
waypoints = [
    {"name": "Kitchen", "x": 2.0, "y": 1.0},
    {"name": "Living Room", "x": 3.0, "y": -1.0},
    {"name": "Bedroom", "x": 1.0, "y": -2.0},
    {"name": "Entrance", "x": -1.0, "y": 0.0},
]
```

To find valid coordinates:
1. Launch the simulation
2. In RViz, use "Publish Point" tool to click on the map
3. Check the terminal for coordinates
4. Update the waypoints list

---

## Recording a Demo Video

### Option 1: Screen Recording

```bash
# Install OBS Studio
sudo apt install obs-studio

# Or use built-in screen recorder
gnome-screenshot -w  # For screenshots
```

### Option 2: ROS2 Bag Recording

```bash
# Record all topics
ros2 bag record -a -o patrol_demo

# Or record specific topics
ros2 bag record /cmd_vel /scan /map /amcl_pose -o patrol_demo
```

---

## Submitting Your Project

Include these files in your submission:

1. **Video**: Screen recording of the patrol robot in action
2. **Workspace files**:
   - `src/two_wheeled_robot/` folder
   - `setup_patrol_robot.sh`
   - `patrol_log.txt` (generated after running)

### Create a ZIP archive:

```bash
cd ~/ros2_ws
zip -r patrol_robot_submission.zip \
    src/two_wheeled_robot \
    setup_patrol_robot.sh \
    patrol_log.txt
```

---

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Automatic Addison Tutorial](https://automaticaddison.com/how-to-run-an-inspection-with-a-robot-ros-2-navigation/)

---

## Common Commands Reference

```bash
# Check ROS2 topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames

# Check Nav2 node states
ros2 lifecycle list /amcl
ros2 lifecycle list /controller_server

# Manual goal (via terminal)
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."

# Kill everything
killall -9 gzserver gzclient ruby python3
```

---

## License

This project is for educational purposes.
