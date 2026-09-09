# Carry My Luggage | ROS 2 Human-Robot Interaction 🧳 

> A ROS 2 robot system inspired by the RoboCup@Home *Carry My Luggage* challenge. Point at your bag, and the robot recognizes what you want and follows you around!

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)
![Nav2](https://img.shields.io/badge/Nav2-Enabled-brightgreen.svg)
![YOLO](https://img.shields.io/badge/YOLO-v11%20Pose%20%7C%20v12%20Detection-orange.svg)
![LiDAR](https://img.shields.io/badge/LiDAR-Leg%20Detection-purple.svg)
![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)

---

## What is this project?

In service robotics (and specifically in the RoboCup@Home competition), a classic challenge is having a domestic robot assist someone with their luggage. This repository implements the two core capabilities needed for that task:

1. **Luggage Identification (`pointing_ros`)**: You point at a suitcase or bag, and the robot figures out *which* object you're gesturing towards using real-time pose estimation and 2D raycasting.
2. **Person Following (`follow_ros`)**: Once the luggage is identified, the robot tracks your legs using a 2D LiDAR scanner and follows you smoothly at a safe social distance using Nav2.

Everything is tested and simulated in **Gazebo** using a Robotnik RB-1 mobile base.

---

## Demos

### 1. Pointing Gesture Recognition
Detecting the user's arm direction, projecting a visual ray forward, and matching it with the bounding box of the target luggage:

![Pointing Detection Demo](carry_my_luggage_1.gif)

### 2. Following the Person
Tracking the human's walking path via 2D LiDAR and navigating behind them while maintaining a comfortable social distance:

![Person Following Demo](carry_my_luggage_2.gif)

---

## How It Works

### Part 1: Pointing Gesture Detection (`pointing_ros`)

Instead of just checking where a hand is located, this node traces the actual vector from the **elbow to the wrist**:

* **Vision Models**:
  * **YOLOv12m** running under `/yolo_detection` for general object recognition.
  * **YOLOv11m-pose** running under `/yolo_pose` for human keypoint extraction (COCO format).
  * Runs on CUDA with a confidence threshold of 0.5 and *Best Effort* QoS for fast, low-latency frames.
* **Synchronization**: Uses `ApproximateTimeSynchronizer` (slop = 0.1s, queue = 2) to match pose keypoints and object bounding boxes with the latest camera frame without latency buildup.
* **Vector Raycasting**:
  * Extracts keypoints for the elbow and wrist (supporting both left and right arms).
  * Constructs a 2D line equation: $P(t) = P_{elbow} + t \cdot (P_{wrist} - P_{elbow})$.
  * Evaluates intersection with detected object bounding boxes.
  * Enforces $t > 1.0$ so the ray only matches objects located **in front of the wrist** (no accidental self-detections or targeting objects behind the person).
* **Outputs**:
  * `/pointing/detections`: Publishes a JSON payload containing `person_id`, `target_object`, and `target_id`.
  * `/dbg_pointing`: Live OpenCV visualization drawing arm rays, bounding boxes, and the matched target.

---

### Part 2: Person Following & Social Navigation (`follow_ros`)

Once on the move, vision alone can easily lose track of people due to occlusions or camera field-of-view limits. This package relies on laser range finding for robust tracking:

* **Leg Detection with 2D LiDAR**:
  * Integrates `upo_laser_people_detector` with an ONNX-based deep learning model (`LFE-PPN.onnx`).
  * Scans `/scan` to detect human leg patterns and publishes 3D marker coordinates on `/detected_people_markers`.
* **Coordinate Transformations (TF2)**:
  * Looks up the robot's current pose (`map` -> `base_link`).
  * Transforms the detected person marker pose from the sensor frame into the global `map` frame using `tf2_geometry_msgs.do_transform_pose`.
* **Tracking & Navigation (Nav2)**:
  * Finds the closest person within range.
  * Computes a navigation waypoint along the line between the robot and the person, set back by a configurable `safety_distance` (default: `1.0 m`).
  * Automatically sets the goal orientation so the robot faces the user.
  * Dispatches `NavigateToPose` action goals to Nav2. If the user stops and is closer than `safety_distance`, the robot halts gracefully.

---

## Repository Structure

```text
carry_my_luggage/
├── pointing_ros/               # Vision & pointing gesture package
│   ├── pointing_ros/
│   │   └── pointing_node.py   # Main gesture raycasting node
│   └── launch/
│       └── main.launch.py     # Gazebo + YOLOv11 + YOLOv12 + pointing node
├── follow_ros/                 # Person following & navigation package
│   ├── follow_ros/
│   │   └── follow_person_node.py # Nav2 person tracking node
│   └── launch/
│       └── follow.launch.py   # Gazebo + LiDAR leg detector + follow node
├── carry_my_luggage_1.gif     # Demo gif: pointing recognition
├── carry_my_luggage_2.gif     # Demo gif: following
└── README.md                  # Project documentation
```

---

## Setup & Installation

### Requirements
* **Ubuntu 22.04** with **ROS 2 Humble**
* **Nav2** (`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`)
* **Gazebo** & RB-1 Robot simulator:
  ```bash
  cd ~/ros2_ws/src
  git clone -b actor_walking https://github.com/igonzf/ros2_rb1.git
  ```
* **YOLO ROS & Ultralytics**:
  ```bash
  pip install ultralytics
  git clone https://github.com/mgonzs13/yolo_ros.git ~/ros2_ws/src/yolo_ros
  ```
* **LiDAR People Detector**:
  ```bash
  git clone https://github.com/robotics-upo/upo_laser_people_detector.git ~/ros2_ws/src/upo_laser_people_detector
  # Install ONNX Runtime GPU dependency
  wget https://robotics.upo.es/~famozur/onnx/onnxruntime-gpu_1.16.3_amd64.deb
  sudo apt install ./onnxruntime-gpu_1.16.3_amd64.deb
  ```

### Build
From your ROS 2 workspace:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## How to Run

### 1. Test Pointing Gesture Recognition
Launch the complete pointing stack (Gazebo simulation, YOLO detection & pose nodes, and the gesture analyzer):
```bash
ros2 launch pointing_ros main.launch.py
```
To view the debug camera output with projected rays and bounding boxes in real-time:
```bash
ros2 run rqt_image_view rqt_image_view /dbg_pointing
```

### 2. Test Person Following
Launch the following pipeline (Gazebo environment, LiDAR leg detector, and the Nav2 tracker):
```bash
ros2 launch follow_ros follow.launch.py
```
*(Optional) You can customize the safety distance (in meters):*
```bash
ros2 launch follow_ros follow.launch.py safety_distance:=1.5
```

### 3. Move the Actor in Simulation
To test the robot's tracking behavior, move the walking person around in Gazebo via the command line:

* **Move actor**:
  ```bash
  ros2 topic pub /actor/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
* **Stop actor**:
  ```bash
  ros2 topic pub /actor/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

---

## Key Topics & Configuration

| Topic / Action | Type | Description |
| :--- | :--- | :--- |
| `/camera/rgb/image_raw` | `sensor_msgs/msg/Image` | Camera feed from the RB-1 robot |
| `/yolo_pose/detections` | `yolo_msgs/msg/DetectionArray` | 2D body keypoints (YOLOv11-pose) |
| `/yolo_detection/detections` | `yolo_msgs/msg/DetectionArray` | Object bounding boxes (YOLOv12) |
| `/dbg_pointing` | `sensor_msgs/msg/Image` | Visual feedback showing arm ray & targeted object |
| `/pointing/detections` | `std_msgs/msg/String` | JSON results of recognized user pointing gestures |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR raw scan data |
| `/detected_people_markers` | `visualization_msgs/msg/MarkerArray` | Leg detection markers from LiDAR |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 action server for robot navigation |