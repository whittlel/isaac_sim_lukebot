# Isaac ROS vSLAM Integration Research Summary

## Overview

This document summarizes research findings for integrating Isaac ROS Visual SLAM (vSLAM) with Isaac Sim for Lukebot autonomous navigation.

**Date:** 2025-10-17
**Target Hardware:** Jetson Orin Nano + OAK-D IOT 75
**Development Environment:** Isaac Sim 5.1.0 + Isaac ROS

---

## What is Isaac ROS vSLAM?

### Description
- **Full Name:** Isaac ROS Visual SLAM (cuVSLAM)
- **Type:** GPU-accelerated stereo visual-inertial odometry (SVIO)
- **Purpose:** Real-time localization and mapping for mobile robots
- **License:** Apache 2.0 (Open Source)

### Key Features
- ✅ GPU-accelerated using CUDA (NVIDIA GPUs only)
- ✅ Stereo camera + optional IMU fusion
- ✅ Real-time performance: 20-30 FPS on Jetson Orin Nano
- ✅ Best-in-class accuracy on KITTI benchmark
- ✅ Native Isaac Sim integration
- ✅ Part of Isaac ROS ecosystem

### Performance Metrics
- **Runtime:** 0.007 seconds/frame on Jetson AGX Xavier
- **Translation Error:** 0.94% (KITTI benchmark)
- **Rotation Error:** 0.0019 deg/m
- **Frame Rate:** Up to 30 FPS on Jetson Orin Nano

---

## ROS 2 Topics and Data Flow

### Input Topics (Required)

#### Stereo Camera Images
```
/stereo_camera/left/image          (sensor_msgs/Image - grayscale)
/stereo_camera/left/camera_info    (sensor_msgs/CameraInfo)
/stereo_camera/right/image         (sensor_msgs/Image - grayscale)
/stereo_camera/right/camera_info   (sensor_msgs/CameraInfo)
```

#### IMU (Optional but Recommended)
```
/imu                               (sensor_msgs/Imu)
```

### Output Topics

#### Odometry and Pose
```
/visual_slam/tracking/odometry           (nav_msgs/Odometry)
/visual_slam/tracking/vo_pose            (geometry_msgs/PoseStamped)
/visual_slam/tracking/vo_pose_covariance (geometry_msgs/PoseWithCovarianceStamped)
/visual_slam/tracking/slam_path          (nav_msgs/Path)
/visual_slam/tracking/vo_path            (nav_msgs/Path)
```

#### Visualization and Diagnostics
```
/visual_slam/vis/loop_closure_cloud      (sensor_msgs/PointCloud2)
/visual_slam/vis/landmarks_cloud         (sensor_msgs/PointCloud2)
/visual_slam/vis/pose_graph_nodes        (visualization_msgs/MarkerArray)
/visual_slam/vis/pose_graph_edges        (visualization_msgs/MarkerArray)
```

---

## Configuration Parameters

### Essential Parameters

#### IMU Configuration
```yaml
enable_imu: true                    # Enable IMU fusion (default: false)
enable_imu_fusion: true             # Use IMU for better accuracy
```

#### Image Processing
```yaml
rectified_images: true              # Set to true if images are pre-rectified
denoise_input_images: false         # Enable for low-light conditions
image_jitter_threshold_ms: 0.5      # Timestamp tolerance
```

#### Coordinate Frames
```yaml
input_base_frame: "base_link"       # Robot base frame
input_left_camera_frame: "camera_left"
input_right_camera_frame: "camera_right"
input_imu_frame: "imu_link"
```

#### Performance Tuning
```yaml
enable_localization_n_mapping: true  # Full SLAM vs odometry only
enable_slam_visualization: true      # Publish visualization topics
enable_observations_view: true       # Publish feature observations
enable_landmarks_view: true          # Publish 3D landmarks
```

---

## Integration with Isaac Sim

### Four-Terminal Workflow

#### Terminal 1: Launch Isaac Sim
```bash
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

#### Terminal 2: Isaac ROS Docker + vSLAM Node
```bash
# Enter Isaac ROS Docker container
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container - install vSLAM
sudo apt-get install -y ros-humble-isaac-ros-visual-slam

# Launch vSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
```

#### Terminal 3: Visualization (RViz2)
```bash
rviz2 -d <path_to_config>.rviz
```

#### Terminal 4: Robot Control
```bash
# Test movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

### Isaac Sim ROS 2 Bridge Setup

1. **Enable ROS 2 Bridge Extension**
   - Window → Extensions
   - Search for "ROS2 Bridge"
   - Toggle ENABLED

2. **Create Stereo Camera Action Graph**
   - Tools → Robotics → ROS 2 OmniGraphs → Camera
   - Configure left camera prim path
   - Configure right camera prim path
   - Set frameId for each camera
   - Enable publishing: Image + CameraInfo

3. **Configure Camera Parameters**
   - Set resolution: 1280x800 (OAK-D IOT 75)
   - Set horizontal FOV: 75°
   - Enable stereo offset calculation
   - Set baseline distance between cameras

4. **Add IMU Publisher (Optional)**
   - Add IMU sensor to Lukebot
   - Create action graph for IMU
   - Publish on `/imu` topic

---

## OAK-D IOT 75 Specific Considerations

### Camera Configuration for vSLAM

The OAK-D IOT 75 has **onboard stereo processing**, which gives us two options:

#### Option 1: Use OAK-D's Stereo Output (Recommended for Testing)
- OAK-D computes depth onboard
- Publish RGB + Depth to ROS 2
- vSLAM can work with RGB-D instead of stereo
- **Pros:** Less processing on Jetson
- **Cons:** vSLAM prefers raw stereo for best accuracy

#### Option 2: Use OAK-D's Raw Stereo Streams (Best for Production)
- Access left/right raw camera streams from OAK-D
- Bypass OAK-D's onboard depth processing
- Let vSLAM do stereo matching (GPU-accelerated)
- **Pros:** Maximum vSLAM accuracy
- **Cons:** Requires proper calibration

### OAK-D Camera Specifications
```yaml
Resolution: 1280x800
FOV: 75° (horizontal)
Baseline: ~7.5cm (between left/right cameras)
Depth Range: 0.35m - 10m
Frame Rate: 30 FPS
IMU: Yes (BMI270 - 6-axis)
```

### Recommended vSLAM Configuration for OAK-D
```yaml
# Use raw stereo from OAK-D
camera_width: 1280
camera_height: 800
camera_fps: 30

# Enable IMU from OAK-D
enable_imu: true
imu_frequency: 100  # BMI270 can do up to 400 Hz

# OAK-D provides pre-rectified images
rectified_images: true
```

---

## Implementation Steps for Lukebot

### Phase 1: Isaac Sim Development

#### Step 1.1: Update Lukebot Camera Setup
- [ ] Modify `lukebot_camera.py` to create stereo camera setup
- [ ] Add left/right camera prims to scene
- [ ] Configure stereo baseline (~7.5cm to match OAK-D)
- [ ] Set resolution to 1280x800, FOV to 75°

#### Step 1.2: Create ROS 2 Bridge Action Graph
- [ ] Enable ROS 2 Bridge extension in Isaac Sim
- [ ] Create stereo camera action graph
- [ ] Publish left/right images and camera_info
- [ ] Add IMU publisher from Lukebot's IMU sensor

#### Step 1.3: Create vSLAM Launch File
- [ ] Create `lukebot_vslam.launch.py` launch file
- [ ] Configure topic remapping for Lukebot
- [ ] Set parameters for OAK-D camera specs
- [ ] Enable IMU fusion

#### Step 1.4: Test in Isaac Sim
- [ ] Launch Isaac Sim with Lukebot
- [ ] Start ROS 2 bridge
- [ ] Launch vSLAM node
- [ ] Drive robot and verify odometry
- [ ] Visualize SLAM in RViz2

### Phase 2: Navigation Integration

#### Step 2.1: Add Nav2 Stack
- [ ] Install Nav2 packages
- [ ] Create navigation launch file
- [ ] Configure costmaps (using depth + vSLAM)
- [ ] Set up local and global planners

#### Step 2.2: Combine vSLAM + Depth Sensing
- [ ] Use vSLAM for localization (slow but accurate)
- [ ] Use live depth for obstacle avoidance (fast)
- [ ] Configure sensor fusion

#### Step 2.3: Create Autonomous Navigation Demo
- [ ] Set goal positions
- [ ] Plan paths using Nav2
- [ ] Execute with holonomic controller
- [ ] Test with yellow target detection

### Phase 3: Jetson Deployment

#### Step 3.1: Jetson Setup
- [ ] Install JetPack SDK
- [ ] Install Isaac ROS on Jetson
- [ ] Configure Docker environment
- [ ] Install OAK-D drivers (DepthAI)

#### Step 3.2: OAK-D Configuration
- [ ] Install depthai-ros package
- [ ] Configure stereo camera pipeline
- [ ] Calibrate cameras (if needed)
- [ ] Test stereo streaming

#### Step 3.3: Deploy and Test
- [ ] Transfer launch files from Isaac Sim
- [ ] Run vSLAM on Jetson
- [ ] Benchmark performance (FPS, accuracy)
- [ ] Tune parameters for real-world performance

---

## Expected Results

### In Isaac Sim (Development)
- Stereo camera streaming at 30 FPS
- vSLAM processing at 20-30 FPS
- Accurate odometry during robot movement
- Loop closure detection in repeated areas
- 3D point cloud map generation

### On Jetson Orin Nano (Deployment)
- vSLAM processing at 20-30 FPS (15W mode)
- Translation error < 1% over distance traveled
- Rotation drift < 0.002 deg/m
- Real-time navigation response
- 4-6 hours battery life (estimated)

---

## Key Resources

### Official Documentation
- [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [Isaac Sim ROS 2 Tutorials](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)

### Community Examples
- [ros-vslam Quickstart](https://github.com/KhaledSharif/ros-vslam) - Isaac Sim + vSLAM example
- [Isaac ROS Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/isaac-ros/656)

### OAK-D Resources
- [DepthAI ROS 2](https://github.com/luxonis/depthai-ros) - Official ROS 2 driver
- [OAK-D IOT 75 Datasheet](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9095/)

---

## Next Steps

1. ✅ **Research Complete** - This document summarizes findings
2. ⏭️ **Implementation** - Create vSLAM demo in Isaac Sim
3. ⏭️ **Testing** - Validate with Lukebot in simulation
4. ⏭️ **Deployment** - Transfer to Jetson Orin Nano

---

## Notes and Considerations

### Advantages of This Approach
- ✅ GPU-accelerated on Jetson (uses hardware efficiently)
- ✅ Seamless Sim → Real transition (same ecosystem)
- ✅ Professional-grade accuracy (KITTI benchmark leader)
- ✅ Active NVIDIA support and development
- ✅ Works with our existing hardware stack

### Potential Challenges
- ⚠️ Requires good stereo calibration
- ⚠️ Performance depends on visual features (struggles in featureless environments)
- ⚠️ No built-in loop closure (yet) - may drift over long distances
- ⚠️ NVIDIA hardware dependency

### Mitigation Strategies
- Use IMU fusion for better accuracy
- Combine with depth-based obstacle avoidance (reactive)
- Consider adding RTAB-Map for loop closure refinement (optional)
- Test extensively in Isaac Sim before hardware deployment

---

**End of Research Document**
