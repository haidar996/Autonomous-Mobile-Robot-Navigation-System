# 🤖 Autonomous Mobile Robot Navigation System

This repository contains the complete implementation of an autonomous mobile robot navigation system that integrates **visual localization** (ArUco markers), **inertial sensing** (IMU), and **obstacle avoidance** (LIDAR) for indoor environments.

---

## 📋 Project Overview

The system features a hybrid computational architecture where:

- **Raspberry Pi 4** handles high-level perception tasks:  
  - ArUco marker detection  
  - Pose estimation

- **Arduino Uno R3** manages low-level sensor fusion and motor control

The robot utilizes a differential-drive platform equipped with:

- **Arducam** for visual input  
- **MPU6050** IMU for orientation sensing  
- **RPLIDAR A1M8** for obstacle detection  
- **L298** motor driver for motion control

---

## 🚀 Key Features

- Real-time visual localization using ArUco markers with **sub-degree angular accuracy**
- Sensor fusion combining visual and inertial measurements via **Kalman filter**
- Obstacle avoidance using LIDAR with **Dynamic Window Approach-inspired** algorithm
- Goal-based navigation with **proportional controller** for waypoint following
- Modular architecture enabling future extensions and improvements

---

## 📁 Repository Structure

├── SLAM.py # Main Raspberry Pi SLAM implementation
├── SLAM.ino # Main Arduino navigation and control code
├── aruco_with_homo.py # ArUco marker detection and pose estimation
├── aruco_with_homo.ino # Arduino code for marker-based localization
├── calibrate.py # Camera calibration script
├── take_photo.py # Utility for capturing calibration images
├── animate.py # LIDAR visualization tool
└── Report.docx # Comprehensive project documentation

---

## 🛠️ Hardware Requirements

- Raspberry Pi 4 (2GB+ RAM)  
- Arduino Uno R3  
- Arducam (compatible with Raspberry Pi)  
- MPU6050 IMU sensor  
- RPLIDAR A1M8  
- L298 Motor Driver  
- DC Motors (differential drive configuration)  
- Power supply (6–12V)

---

## 🔧 Software Dependencies

### Raspberry Pi (Python)

```bash
pip install opencv-python picamera2 numpy pyserial
## 🧰 Arduino Libraries

- [`BasicLinearAlgebra`](https://github.com/tomstewart89/BasicLinearAlgebra) library  
- [`MPU6050_light`](https://github.com/rfetick/MPU6050_light) library  
- Standard Arduino libraries: `Wire`, `Serial`

---

## 🎯 Getting Started

### 🛠️ Hardware Assembly

- Connect all sensors to appropriate interfaces  
- Ensure proper power supply for all components  
- Mount camera with known extrinsic parameters

### 🎥 Camera Calibration

- Use `take_photo.py` to capture calibration images  
- Run `calibrate.py` to compute camera matrix and distortion coefficients  
- Update the calibration values in the main scripts

### 🧩 ArUco Marker Placement

- Place markers in the environment with known positions  
- Update marker positions in the Arduino code

### 🚦 Deployment

- Upload Arduino code to the microcontroller  
- Run the main SLAM script on Raspberry Pi  
- Monitor serial communication for debugging

---

## 📊 System Architecture

The system follows a **distributed processing** approach:

### 🔍 Perception Layer (Raspberry Pi)

- Camera image capture and processing  
- ArUco marker detection and pose estimation  
- LIDAR data acquisition

### 🧠 Control Layer (Arduino)

- Sensor fusion using **Kalman filter**  
- Motion control and motor driving  
- Serial communication with Raspberry Pi

---

## 🔄 Data Flow

1. Camera captures frames and detects ArUco markers  
2. Pose estimation calculates robot position relative to markers  
3. Position data is sent to Arduino via serial communication  
4. Arduino fuses visual data with IMU readings using Kalman filter  
5. Navigation algorithm computes optimal path considering obstacles  
6. Motor commands are generated and executed

---

## ⚙️ Configuration

Key configuration parameters (adjust in code as needed):

- Camera calibration matrix and distortion coefficients  
- ArUco marker sizes and dictionary type  
- Robot physical parameters (wheelbase, wheel radius)  
- Control gains (proportional constants for navigation)  
- Kalman filter noise parameters  
- Obstacle avoidance thresholds

---

## 🧪 Testing and Validation

- Use `animate.py` to visualize LIDAR data  
- Monitor serial output for pose estimates and navigation status  
- Test in controlled environments with known marker placements  
- Validate obstacle avoidance with various obstacle configurations

---

## 📈 Performance

The system demonstrates:

- Real-time operation at manageable frame rates  
- Accurate localization within **centimeter-level precision**  
- Effective obstacle avoidance in **dynamic environments**  
- Smooth navigation between waypoints

---

## 🔮 Future Enhancements

Potential improvements and extensions:

- Full SLAM implementation  
- Multi-marker simultaneous observation  
- Advanced path planning algorithms (e.g., A\*, RRT)  
- Machine learning for improved perception  
- Multi-robot coordination  
- Cloud integration for remote monitoring

---

## 📚 Documentation

For detailed theoretical background and implementation details, refer to `Report.docx`, which includes:

- System architecture overview  
- Mathematical formulations  
- Experimental results  
- Comprehensive references

---

## 👥 Contributors

- **Alaa Hussein**  
- **Baraa Lazkani**  
- **Haidar Saad**  
- **Humam Yehia**

**Supervised by:**

- Dr. **Fadi Muttawag**  
- Eng. **Baher Kher-Bek**

---

## 📄 License

This project is available for **academic and research purposes**.  
Please cite appropriately if used in research work.

---

## 🤝 Contributing

We welcome contributions to enhance this project.  
Please feel free to submit **issues** and **pull requests** for improvements and bug fixes.
