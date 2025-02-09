# Autonomous Vehicle System

This project implements an **Autonomous Vehicle System** using a **Raspberry Pi 4B**, **Arduino Mega**, **ultrasonic sensors**, and a **camera**. The system integrates several key functionalities to enable autonomous navigation, including **speed control**, **object tracking**, **obstacle avoidance**, **distance measurement**, and **lane detection**. 

### Project Components:
- **Raspberry Pi 4B**: Controls the overall system and performs object detection, tracking, and decision-making logic.
- **Arduino Mega**: Manages the motor control and reads data from the ultrasonic sensors for obstacle detection.
- **Ultrasonic Sensors**: Two ultrasonic Sensors to detect obstacles and assist in collision avoidance.
- **Camera**: Provides real-time video feed for object detection and lane tracking using computer vision.
  
### Key Features:
1. **Speed Control**: The system adjusts the vehicle's speed based on detected obstacles or lane boundaries.
2. **Object Tracking**: Using **YOLOv5** and the camera, the system detects and tracks objects in the environment.
3. **Obstacle Avoidance**: The ultrasonic Sensors detect obstacles, and the system adjusts the vehicle's movement accordingly.
4. **Distance Measurement**: Ultrasonic sensors measure the distance to obstacles, providing data for decision-making.
5. **Lane Detection**: The system detects lane markings on the road and adjusts the vehicle’s position within the lane to stay on course.
6. **Autonomous Navigation**: The vehicle can move autonomously by processing real-time data from sensors and camera to navigate the environment.# Autonomous Vehicle System
 
