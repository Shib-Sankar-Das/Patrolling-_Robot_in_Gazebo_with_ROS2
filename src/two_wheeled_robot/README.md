# 🤖 Patrolling Robot in Gazebo with ROS2

A fully autonomous patrolling robot simulation using **ROS2 Humble** and **Gazebo Classic**. The robot navigates through 24 predefined waypoints covering an entire house environment, dynamically avoids obstacles using Nav2, and maintains detailed patrol logs.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-purple)
![License](https://img.shields.io/badge/License-Educational-green)

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🗺️ **Full Map Coverage** | 24 waypoints covering Living Room, Kitchen, Bedroom, Fitness Room, Hallways, and more |
| 🚧 **Dynamic Obstacle Avoidance** | Nav2 stack with AMCL localization and DWB local planner |
| �️ **LIDAR Safety Layer** | Real-time obstacle detection with emergency stop capability |
| 🔄 **Recovery Behaviors** | Automatic backup, spin, and costmap clearing when stuck |
| 📝 **Patrol Logging** | Timestamps, coordinates, and visited waypoints logged to file |
| ⚡ **CycloneDDS** | Reliable communication replacing FastDDS (fixes queue overflow issues) |
| 🎮 **RViz Visualization** | Real-time visualization of robot, map, costmaps, and planned paths |

---

## 📋 Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS2 Humble Hawksbill**
- **Gazebo Classic 11**
- **Nav2 Navigation Stack**

### Install Required Packages

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Navigation and dependencies
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher
```

### Configure Network Buffers (One-time setup)

```bash
# Create sysctl config for larger socket buffers
sudo tee /etc/sysctl.d/10-cyclonedds.conf << EOF
net.core.rmem_max=26214400
net.core.wmem_max=26214400
EOF

# Apply changes
sudo sysctl --system
```

---

## 🚀 Quick Start

### Step 1: Clone and Build

```bash
# Navigate to workspace
cd ~/ros2_ws

# Build the package
source /opt/ros/humble/setup.bash
colcon build --packages-select two_wheeled_robot --symlink-install

# Source the workspace
source install/setup.bash
```

### Step 2: Source Environment (Every Terminal!)

```bash
source ~/ros2_ws/setup_patrol_robot.sh
```

This script:
- ✅ Sources ROS2 Humble
- ✅ Sources workspace
- ✅ Switches to CycloneDDS
- ✅ Sets Gazebo model paths

### Step 3: Launch Simulation (Terminal 1)

```bash
source ~/ros2_ws/setup_patrol_robot.sh
ros2 launch two_wheeled_robot house_world_v1.launch.py
```

**Wait for:**
- Gazebo window with house environment and robot
- RViz showing map, robot, and TF frames
- Terminal showing "Managed nodes are active"

⏱️ **Initial launch takes 30-60 seconds**

### Step 4: Run Patrol Robot (Terminal 2)

**Option A: Basic Patrol (Original)**
```bash
source ~/ros2_ws/setup_patrol_robot.sh
ros2 run two_wheeled_robot my_patrol_robot.py
```

**Option B: Enhanced Patrol with Obstacle Avoidance (Recommended)**
```bash
# Terminal 2: Start obstacle avoidance safety layer
source ~/ros2_ws/setup_patrol_robot.sh
ros2 launch two_wheeled_robot patrol_with_obstacle_avoidance.launch.py

# Terminal 3: Run enhanced patrol robot
source ~/ros2_ws/setup_patrol_robot.sh
ros2 run two_wheeled_robot enhanced_patrol_robot.py
```

**Expected Output:**
```
[INFO] Setting initial pose...
[INFO] Waiting for Nav2 to become active...
[INFO] Nav2 is active and ready!
[INFO] Patrol route with 24 waypoints:
  1. Living Room Center: (1.0, -1.5)
  2. Living Room East: (3.0, -1.0)
  ...
[INFO] STARTING PATROL CYCLE 1/1
[INFO] Navigating to: Living Room Center
```

---

## 🗺️ Patrol Waypoints

The robot patrols 24 waypoints covering the entire house:

| Area | Waypoints |
|------|-----------|
| **Living Room** | Center (1.0, -1.5), East (3.0, -1.0) |
| **Kitchen** | Entrance (5.0, 0.5), Table (6.5, 1.0), Corner (7.5, -2.5) |
| **Entrance** | Main Door (5.0, -4.0), Hallway (3.0, -4.0) |
| **Central Hall** | South (0.0, -3.5), Center (0.0, 0.0), North (0.0, 2.0) |
| **Fitness Room** | Main (3.0, 3.0), Corner (2.5, 2.5) |
| **Dining/Balcony** | Dining (-0.5, 3.5), Balcony (1.0, 4.0) |
| **Bedroom** | Door (-3.0, 1.0), Center (-5.5, 1.5), Corner (-7.0, 2.5) |
| **West Hallway** | North (-3.0, -1.0), Center (-5.0, -1.0), South (-6.5, -3.5) |
| **Southwest** | Corner (-7.5, -4.0), Room (-6.0, -4.5) |
| **Return Path** | West (-3.0, -2.0), Center (-1.0, -1.0) |

---

## 📁 Project Structure

```
ros2_ws/
├── setup_patrol_robot.sh              # Environment setup script
├── patrol_log.txt                     # Runtime patrol logs
└── src/two_wheeled_robot/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── cyclonedds.xml             # DDS configuration (1MB buffers)
    │   └── ekf.yaml                   # Extended Kalman Filter config
    ├── launch/house_world/
    │   ├── house_world_v1.launch.py   # Main simulation launch
    │   └── patrol_with_obstacle_avoidance.launch.py  # Obstacle avoidance launch
    ├── maps/house_world/
    │   ├── house_world.pgm            # Occupancy grid map
    │   └── house_world.yaml           # Map metadata
    ├── params/house_world/
    │   └── nav2_params.yaml           # Navigation parameters (A* planner)
    ├── scripts/
    │   ├── my_patrol_robot.py         # Basic patrol script (24 waypoints)
    │   ├── enhanced_patrol_robot.py   # Advanced patrol with recovery behaviors
    │   ├── obstacle_avoidance.py      # LIDAR-based obstacle safety layer
    │   └── robot_navigator.py         # Navigation helper class
    ├── urdf/
    │   └── two_wheeled_robot.urdf     # Robot description
    ├── worlds/
    │   └── house.world                # Gazebo world file
    └── models/
        └── two_wheeled_robot_description/
```

---

## 🛡️ Obstacle Avoidance System

The project includes a sophisticated obstacle avoidance safety layer inspired by:
- [AjaySurya-018/Patrol-Robot](https://github.com/AjaySurya-018/Patrol-Robot)
- [vinay06vinay/Turtlebot3-Obstacle-Avoidance-ROS2](https://github.com/vinay06vinay/Turtlebot3-Obstacle-Avoidance-ROS2)

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    LIDAR Sensor (/scan)                     │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│              Obstacle Avoidance Node                         │
│  ┌──────────────┬──────────────┬──────────────┐             │
│  │ Front Sector │ Left Sector  │ Right Sector │             │
│  │  (±30°)      │  (30°-90°)   │  (-30°--90°) │             │
│  └──────────────┴──────────────┴──────────────┘             │
│                                                              │
│  Distance Thresholds:                                        │
│  • Critical: 0.25m → Emergency Stop                         │
│  • Safe: 0.35m → Warning                                    │
│  • Warning: 0.6m → Slow Down                                │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│/emergency_stop│ │/obstacle_info│ │/cmd_vel      │
│   (Bool)     │ │  (String)    │ │  (Twist)     │
└──────────────┘ └──────────────┘ └──────────────┘
        │                               │
        ▼                               ▼
┌─────────────────────────────────────────────────────────────┐
│              Enhanced Patrol Robot                          │
│  • Subscribes to obstacle topics                            │
│  • Implements recovery behaviors (backup, spin)             │
│  • Clears costmaps when stuck                               │
└─────────────────────────────────────────────────────────────┘
```

### Operating Modes

| Mode | Description |
|------|-------------|
| **monitor** | Publishes obstacle info, no velocity override |
| **active** | Stops robot on obstacles, manual navigation resumes |
| **autonomous** | Full autonomous avoidance with turn commands |

### Launch Parameters

```bash
ros2 launch two_wheeled_robot patrol_with_obstacle_avoidance.launch.py \
    avoidance_mode:=active \
    safe_distance:=0.35 \
    warning_distance:=0.6
```

### Recovery Behaviors

The enhanced patrol robot includes automatic recovery:

1. **Backup Maneuver**: Reverse 0.3m when stuck
2. **Spin Recovery**: Rotate 180° to find clear path
3. **Costmap Clearing**: Clear local costmap around robot
4. **Wait and Retry**: Pause before retrying failed navigation

---

## ⚙️ Configuration

### Customize Waypoints

Edit `scripts/my_patrol_robot.py`:

```python
waypoints = [
    {"name": "Custom Point 1", "x": 2.0, "y": 1.5},
    {"name": "Custom Point 2", "x": -3.0, "y": 2.0},
    # Add more waypoints...
]
```

### Find Valid Coordinates

1. Launch the simulation
2. In RViz, click the "Publish Point" tool
3. Click on the map - coordinates appear in terminal
4. Add coordinates to waypoints list

### Adjust Navigation Parameters

Edit `params/house_world/nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    max_vel_x: 0.26          # Max forward velocity
    max_vel_theta: 1.0       # Max rotational velocity
    
amcl:
  ros__parameters:
    max_particles: 2000      # Particle filter size
```

---

## 🔧 Troubleshooting

### "Publishing Initial Pose" hangs
**Cause:** DDS communication failure  
**Solution:** Verify CycloneDDS is active:
```bash
echo $RMW_IMPLEMENTATION  # Should show: rmw_cyclonedds_cpp
```

### "Fixed Frame [map] does not exist" in RViz
**Cause:** AMCL not initialized  
**Solution:** Wait 30-60 seconds for Nav2 lifecycle manager to activate all nodes

### Robot not moving
**Cause:** Nav2 not fully active  
**Solution:** Check terminal for "Managed nodes are active" message

### "queue is full" warnings
**Cause:** FastDDS buffer overflow  
**Solution:** This project uses CycloneDDS which fixes this. Ensure you sourced `setup_patrol_robot.sh`

### Gazebo crashes
**Solution:**
```bash
# Kill zombie processes
killall -9 gzserver gzclient ruby

# Restart simulation
ros2 launch two_wheeled_robot house_world_v1.launch.py
```

---

## 📊 Patrol Log Format

Logs are saved to `~/ros2_ws/patrol_log.txt`:

```
2026-01-15 10:40:49,454 - INFO - PATROL ROBOT STARTED
2026-01-15 10:40:49,454 - INFO - Added waypoint: Living Room Center at (1.0, -1.5)
2026-01-15 10:40:49,474 - INFO - Starting Patrol Cycle 1/1
2026-01-15 10:42:06,701 - INFO - Navigating to: Living Room East (3.0, -1.0)
2026-01-15 10:42:29,590 - INFO - Navigating to: Kitchen Entrance (5.0, 0.5)
...
2026-01-15 10:50:00,000 - INFO - PATROL MISSION COMPLETE
```

---

## 🛠️ Technical Details

### Navigation Stack Components

| Component | Purpose |
|-----------|---------|
| **AMCL** | Adaptive Monte Carlo Localization |
| **NavFn** | Global path planner (A* algorithm enabled) |
| **DWB** | Dynamic Window local planner |
| **Costmap2D** | Obstacle representation (inflation: 0.45m) |
| **BT Navigator** | Behavior tree for navigation |
| **Waypoint Follower** | Sequential waypoint navigation |
| **Obstacle Avoidance** | LIDAR-based safety layer |

### DDS Configuration

Using CycloneDDS with optimized settings:
- Socket buffer: 1MB (prevents overflow)
- Max message size: 65.5KB
- Multicast: enabled

### Robot Specifications

- **Type:** Differential drive (2-wheeled)
- **Sensors:** 2D LiDAR, IMU
- **Localization:** EKF (odometry + IMU fusion)

---

## 📚 References

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [CycloneDDS Configuration](https://cyclonedds.io/docs/)
- [Gazebo Classic Tutorials](http://gazebosim.org/tutorials)
- [Patrol-Robot by AjaySurya-018](https://github.com/AjaySurya-018/Patrol-Robot)
- [Turtlebot3 Obstacle Avoidance](https://github.com/vinay06vinay/Turtlebot3-Obstacle-Avoidance-ROS2)

---

## 📜 License

This project is for educational purposes.

---

## 👤 Author

**Shib Sankar Das**

---

## 🙏 Acknowledgments

- ROS2 Navigation Stack team
- Eclipse CycloneDDS project
- Automatic Addison tutorials
