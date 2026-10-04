# 🚁 Autonomous AI Rescue Drone

An autonomous rescue-drone simulation built with **ROS 2, PX4, Gazebo, YOLO11, and depth perception**.

The drone uses computer vision to detect a person, estimates the person's distance using depth data, autonomously approaches the target, simulates emergency assistance, returns to its home position, and lands.

---

## 🎯 Project Goal

The goal is to simulate an autonomous drone capable of assisting a person during an emergency without manual piloting.

### Autonomous Mission

```text
SEARCH
   ↓
PERSON DETECTED
   ↓
DEPTH ESTIMATION
   ↓
APPROACH PERSON
   ↓
DELIVERY RANGE
   ↓
EMERGENCY PACKAGE DELIVERED
   ↓
RETURN HOME
   ↓
LAND
```

### 🧠 System Architecture

```              Gazebo
                │
          RGB + Depth Camera
                │
                ▼
             ROS 2
                │
        ┌───────┴────────┐
        ▼                ▼
      YOLO11        Depth Camera
        │                │
        ▼                ▼
 Person Detection   Distance Estimation
        │                │
        └───────┬────────┘
                ▼
       Rescue Mission Controller
                │
                ▼
               PX4
                │
                ▼
          Gazebo X500
```

## ✨ Features

### 👁️ Computer Vision

- YOLO11 person detection
- Real-time person localization
- Bounding-box visualization
- Person center estimation
- Detection confidence

### 📏 Depth Perception
- Depth camera integration
- Person-to-drone distance estimation
- RGB/depth coordinate mapping
- Real-time target distance

### 🚁 Autonomous Flight
- PX4 Offboard control
- Autonomous takeoff
- Person approach
- Target centering
- Delivery-range detection
- Simulated emergency package delivery
- Return-to-home
- Autonomous landing

## 🛠️ Technologies

| Technology | Purpose |
| :--- | :--- |
| **ROS 2 Jazzy** | Robotics middleware |
| **PX4** | Flight controller |
| **Gazebo Sim 8** | Drone simulation |
| **YOLO11** | Person detection |
| **Python** | AI and control |
| **OpenCV** | Computer vision |
| **cv_bridge** | ROS image processing |
| **ROS-Gazebo Bridge** | Sensor communication |
| **Micro XRCE-DDS** | PX4 ↔ ROS 2 communication |
| **QGroundControl** | Flight monitoring |


### 📂 Project Structure

```
autonomous-ai-rescue-drone/
│
├── ros2/
│   └── rescue_drone/
│       ├── rescue_drone/
│       │   ├── person_detector.py
│       │   ├── person_tracker.py
│       │   ├── person_distance.py
│       │   ├── depth_reader.py
│       │   ├── rescue_controller.py
│       │   ├── rescue_mission.py
│       │   └── ...
│       │
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
├── models/
│
├── simulation/
│
├── rescue_world.sdf
├── README.md
└── .gitignore
```

## ⚙️ Environment

Tested on:

Ubuntu 24.04
ROS 2 Jazzy
Gazebo Sim 8
Python 3.12
PX4 SITL
CPU-based YOLO inference
🚀 Version 1

The current version demonstrates a complete autonomous rescue mission:

YOLO Detection
      ↓
Person Position
      ↓
Depth Distance
      ↓
Autonomous Approach
      ↓
Delivery Range
      ↓
Simulated Assistance
      ↓
Return Home
      ↓
Autonomous Landing

The system was tested successfully in Gazebo with a simulated rescue environment.

🎥 Demonstration

A demonstration video and screenshots will be added here.

Future versions will include a more advanced demonstration showing dynamic target tracking and improved autonomous behavior.

🔮 Version 2 — Planned Improvements

The next version will focus on making the drone better at following a moving person.

Planned improvements:

Dynamic person tracking
Moving-target prediction
Target velocity estimation
Improved RGB/depth alignment
Better camera-to-drone coordinate transformation
Adaptive approach speed
More robust target-loss handling
Improved delivery behavior
Better tracking of moving rescue targets
🧩 Why This Project?

This project combines several areas of robotics and AI:

Computer Vision + Depth Perception + ROS 2 + Autonomous Flight

Instead of manually controlling the drone, the system allows the drone to make decisions based on what its sensors see.

👨‍💻 Author
Hayat Ahmedjara

AI & Machine Learning Engineer

Addis Ababa, Ethiopia

Focus areas:

Machine Learning
Computer Vision
Robotics
LLM Engineering
📌 Project Status

Version 1 — Completed ✅

Autonomous simulated rescue mission successfully demonstrated.

Version 2 — In Development 🚧

Dynamic person tracking and improved target following.
