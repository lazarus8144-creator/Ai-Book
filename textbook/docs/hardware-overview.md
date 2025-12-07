---
sidebar_position: 2
title: Hardware Overview
description: Complete hardware requirements for all course modules
keywords: [hardware, requirements, equipment, robotics]
---

# Hardware Requirements Overview

This page provides a comprehensive overview of all hardware needed across the four modules. Requirements are organized by category to help you plan your setup.

## Quick Summary

| Module | Primary Hardware | Estimated Cost | Required/Optional |
|--------|-----------------|----------------|-------------------|
| Module 1: ROS 2 | Ubuntu workstation | $0-$1000 | Required |
| Module 2: Digital Twin | Gaming GPU (GTX 1660+) | $200-$600 | Recommended |
| Module 3: NVIDIA Isaac | NVIDIA Jetson Orin | $500-$2000 | Required |
| Module 4: VLA | USB Camera + Jetson | $30-$400 | Required |

## Computing Hardware

### Workstation Requirements

**Minimum Specifications:**
- CPU: Intel Core i5 or AMD Ryzen 5 (4 cores)
- RAM: 16 GB
- Storage: 256 GB SSD
- OS: Ubuntu 22.04 LTS

**Recommended Specifications:**
- CPU: Intel Core i7/i9 or AMD Ryzen 7/9 (8+ cores)
- RAM: 32 GB or more
- Storage: 512 GB NVMe SSD
- OS: Ubuntu 22.04 LTS

### GPU Requirements (Module 2 & 3)

**For Digital Twin Simulation:**
- Minimum: NVIDIA GTX 1660 (6GB VRAM)
- Recommended: NVIDIA RTX 3060 or better (12GB+ VRAM)
- Required for: Gazebo simulation, Unity integration

**For NVIDIA Isaac:**
- Required: NVIDIA Jetson Orin Nano (8GB) or better
- Recommended: NVIDIA Jetson AGX Orin (32GB or 64GB)
- Alternative: Desktop GPU with CUDA support (RTX 3090, A6000)

## Sensors

### Camera Equipment

**Module 1 & 4 Requirements:**
- Minimum: USB Webcam (720p)
- Recommended: Intel RealSense D435 Depth Camera
- Features needed: RGB, optional depth sensing
- Estimated cost: $30-$400

### Optional Sensors

**For Advanced Labs:**
- LiDAR: RPLIDAR A1 or equivalent (~$100)
- IMU: MPU-6050 or similar (~$5-20)
- Servo Motors: For robotic arm integration (~$20-200)

## Networking

**Required:**
- Router with 5GHz Wi-Fi (for ROS 2 distributed systems)
- Ethernet cables for stable connections
- Estimated cost: $30-100 (if not already available)

## Module-Specific Breakdown

### Module 1: ROS 2

**Required:**
- Ubuntu 22.04 workstation (see specifications above)
- Internet connection for package downloads

**Optional:**
- Second computer or Raspberry Pi for distributed ROS 2 testing
- USB Camera for vision examples

### Module 2: Digital Twin

**Required:**
- GPU-enabled workstation (GTX 1660+)
- 20 GB free disk space for Gazebo and Unity

**Recommended:**
- Dedicated GPU with 12GB+ VRAM for complex simulations
- Second monitor for simultaneous code and simulation viewing

### Module 3: NVIDIA Isaac

**Required:**
- NVIDIA Jetson Orin Nano (8GB minimum)
- USB-C power supply (appropriate for your Jetson model)
- MicroSD card (64GB minimum, UHS-I U3 rated)

**Recommended:**
- NVIDIA Jetson AGX Orin for better performance
- Active cooling solution
- External SSD for additional storage

### Module 4: Vision-Language-Action

**Required:**
- NVIDIA Jetson (from Module 3)
- USB Camera with 1080p support
- Robotic arm or manipulator (can be simulated)

**Recommended:**
- Intel RealSense depth camera
- Physical robotic arm for real-world testing

## Cost Estimate Summary

### Budget Setup (~$1,000-$1,500)
- Used/refurbished workstation with GTX 1660
- NVIDIA Jetson Orin Nano (8GB)
- Basic USB webcam
- Standard networking equipment

### Recommended Setup (~$2,500-$3,500)
- New workstation with RTX 3060
- NVIDIA Jetson AGX Orin (32GB)
- Intel RealSense D435
- Quality networking equipment

### Professional Setup (~$5,000+)
- High-end workstation with RTX 4090
- NVIDIA Jetson AGX Orin (64GB)
- Multiple sensors and cameras
- Physical robotic platform

## Purchasing Recommendations

### Where to Buy

**Computing Hardware:**
- NVIDIA Jetson: [NVIDIA Store](https://www.nvidia.com/), authorized distributors
- GPUs: Newegg, Amazon, B&H Photo
- Workstations: System76, Dell, or custom build

**Sensors & Robotics:**
- Intel RealSense: Intel store, robotics suppliers
- LiDAR/IMU: RobotShop, SparkFun, Adafruit
- Robotic Arms: Universal Robots, various suppliers

### Cost-Saving Tips

1. **Start with simulation**: Use your existing computer for Module 1-2 before investing in Jetson
2. **Buy refurbished**: Jetson and GPUs available refurbished at significant savings
3. **University access**: Check if your institution has hardware labs
4. **Community sharing**: Join robotics meetups for equipment access

## Software Requirements

All software used in this course is **free and open-source**:

- ROS 2 Humble (free)
- Gazebo (free)
- Unity (free for educational use)
- NVIDIA Isaac Sim (free with NVIDIA account)
- Python and related libraries (free)

## Setup Validation

Before starting each module, verify you have:

- [ ] Required hardware for that module
- [ ] Ubuntu 22.04 LTS installed
- [ ] Stable internet connection
- [ ] Sufficient disk space (minimum 50GB free)
- [ ] All necessary cables and adapters

## Alternative Approaches

If you cannot access required hardware:

- **Cloud Computing**: Use cloud instances with GPU support (AWS, Google Cloud, Azure)
- **Simulation Only**: Many concepts can be learned entirely in simulation
- **Hybrid Approach**: Use simulation for learning, hardware for final projects

## Questions?

If you're unsure about hardware compatibility:
1. Check the detailed requirements in each module's introduction
2. Search for specific compatibility questions
3. Community forums and Discord channels (links in course resources)

---

**Next Steps:** Review [Module 1: ROS 2](/module-1-ros2) to begin your learning journey.
