Omnidirectional Robot Control with ROS2 and Python

This a control system for a three-wheel omnidirectional robot, developed using Python, ROS2, and a Raspberry Pi. This project implements the inverse kinematics needed to transform high-level motion commands (via geometry_msgs/Twist) into angular velocities for each wheel, enabling precise omnidirectional movement.

This work was created for Activity 1.5: Motion Test with an Omnidirectional Robot in the Mobile Ground Robots course at ITESM (June 2025).

📽️ Demo

https://github.com/user-attachments/assets/f89d2aac-42d5-40e9-87ea-0ad41d847922



A demonstration video showing the robot executing live Twist commands can be found in the demo/ folder or click here if browsing locally.

⚙️ System Overview

🛠 Hardware Used

    Raspberry Pi (with GPIO support)
    
    3 omnidirectional wheels placed at 120° angles
    
    DC motors with encoders
    
    Motor drivers
    
    LEDs and servos (optional features)

🧠 Software Stack

    Python 3
    ROS2 (Humble)
    
    gRPC for Raspberry Pi motor control
    
    teleop_twist_keyboard for velocity command input

📐 Inverse Kinematics

The robot receives desired linear and angular velocities (Vx, Vy, ωz) and converts them into angular velocities (ωi) for each of the three wheels using the following equations:

    ωᵢ = ( -sin(θᵢ)·Vₓ + cos(θᵢ)·Vᵧ + R·ω_z ) / r

Where:

    r: radius of the wheel (0.024 m)
    
    R: distance from center to wheel (0.1041 m)
    
    θᵢ: orientation of each wheel (0, 120°, 240°)

These equations were implemented inside the SetState method in the RPIMotorService.py file, as follows:

<img width="665" alt="image" src="https://github.com/user-attachments/assets/883eb30a-2eec-4273-8ff4-590b65a4841b" />


This was the main change made to adapt the base motor control code to work with omnidirectional motion.

🚀 How to Run

Clone the repository:

Download the .zip with the name omni.zip

Launch the motor control service on the Raspberry Pi:

python3 src/RPIMotorService.py

In your ROS2 workspace, launch the teleoperation node:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

If using twist_listener.py, make sure ROS2 and gRPC stubs are set up, then run:

python3 scripts/twist_listener.py



👨‍💻 Contributors

    Hugo Daniel Castillo Ovando (A00836025)
    Rosendo De Los R´ıos (A01198515)
    Jes´us Garc´ıa (A01369587)
    V´ıctor Meneses (A01384002)
    Rub´en Hilario (A00835860)
    Juan Jos´e J´auregui (A00836722)
    Jordan Palafox (A00835705)

🏁 Final Notes

This project showcases the integration of theory, control systems, and practical testing to achieve synchronized omnidirectional movement using ROS2 and embedded systems. It also serves as a base for future swarm robotics or path planning extensions.

