# AI-UAVs
The Repo contains python scripts of projects related to autonomous drone.

# Overview
Project Overview: Human Following Drone

This project focuses on developing an autonomous drone capable of tracking and following a specific individual in real time. The primary goal is to maintain continuous, stable, and safe tracking of the target using a combination of advanced computer vision techniques, control algorithms, and sensor integrations.

## Detection and Recognition:

**Face Detection and Recognition:**
        The system employs YOLO v8 to detect faces in the drone's camera feed. When a face is detected, facial recognition is performed by comparing the captured face with pre-stored embeddings of the target individual. This ensures that the drone only initiates tracking when the intended person is identified.
        
**Person Detection:**
        Once the target's identity is confirmed, the drone activates a dedicated person detection model. This model continuously tracks the target, ensuring the drone remains focused on the correct individual despite potential distractions in the environment.

## Drone Control and Stabilization:

**DroneKit Integration:**
        The DroneKit library is used to interface with and control the drone. It enables the execution of high-level flight commands and real-time adjustments based on input from the detection modules.

**PID Controllers for Smooth Navigation:**
        To ensure smooth and responsive movements, PID (Proportional-Integral-Derivative) controllers are employed. These controllers process the positional data derived from the dimensions of the bounding boxes (obtained in pixel coordinates) and convert them into precise movement commands. This approach helps minimize jerky movements, keeping the target person centered in the frame by adjusting the drone’s left/right and up/down positions.

## Sensor Integration and Obstacle Avoidance:

  **LIDAR for Depth and Distance Measurement:**

A LIDAR sensor provides accurate measurements of the drone’s distance from obstacles and the target. This sensor plays a crucial role in controlling the forward and backward movements, ensuring that the drone maintains an optimal and safe distance from the person it is following.

**Multiple Sonar Sensors for Comprehensive Obstacle Avoidance:**
       
In addition to LIDAR, the system is equipped with multiple sonar sensors positioned in all four directions around the drone. These sensors continuously monitor the surrounding environment for nearby objects. The sonar sensors provide detailed distance readings that enable the drone to detect obstacles in close proximity, even when they are not in the direct line of sight of the camera or LIDAR. The data from these sonar sensors is integrated into the flight control system to generate real-time obstacle avoidance maneuvers. This ensures that the drone can safely navigate around objects and maintain a clear path while following the target, particularly in cluttered or dynamic environments.

  **Additional Sensors and Feedback Loops:**
  
Depending on the flight conditions, additional sensors such as inertial measurement units (IMUs) or GPS can be incorporated to further enhance stability and navigation. These sensors help the drone adapt to environmental changes, such as wind or rapid movements, ensuring reliable tracking.

## Algorithmic Flow and System Architecture:

  **Real-Time Processing:**
  
The entire system operates in real time, where the face detection, recognition, and person tracking algorithms work in tandem with the flight control system. This ensures that any detection of the target triggers an immediate response in the drone’s flight path adjustments.

  **Data Conversion and Command Generation:**
  
The bounding box dimensions (representing the target’s location in pixel coordinates) are continuously converted into movement commands via the PID controllers. These commands are then sent to the drone through the DroneKit interface, allowing for immediate adjustments in the X and Y axes. Simultaneously, LIDAR and sonar data inform the forward/backward adjustments, creating a comprehensive control mechanism that balances speed, distance, and positioning.


# Hardware
1 - Pixhawk 2.4.8

2 - Raspberry pi 5 8gb

3 - F450 Quadcopter frame

4 - DJI 935kva BLDC motors x4

5 - FlySky FS-i6x Transmitter and Reciver 

6 - Benewake Tf Luna LIDAR 8m

7 - Raspberry Pi Camera 2

8 - XY-3606 5V Buck Converter

9 - 5200 mah Lipo 3S 12V battery 

10 - HC-SR04 Ultrasonic Sensors x4

11 - GPS and Telemetry


NOTE: A test video is attached. The flight was unstable due to high wind speeds. However, the drone's PID can be tuned to perform well in various environments.
