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

\omega_i = \frac{1}{r}(-\sin(\theta_i) V_x + \cos(\theta_i) V_y + R \cdot \omega_z)

Where:

r: radius of the wheel (0.024 m)

R: distance from center to wheel (0.1041 m)

\theta_i: orientation of each wheel (0, 120°, 240°)

These equations were implemented inside the SetState method in the RPIMotorService.py file, as follows:

self.w = np.array([
    -math.sin(0) * self.v_x + math.cos(0) * self.v_y + self.R * self.v_yaw,
    -math.sin(2*math.pi/3) * self.v_x + math.cos(2*math.pi/3) * self.v_y + self.R * self.v_yaw,
    -math.sin(4*math.pi/3) * self.v_x + math.cos(4*math.pi/3) * self.v_y + self.R * self.v_yaw
], dtype=np.float64) / self.r

This was the main change made to adapt the base motor control code to work with omnidirectional motion.

🚀 How to Run

Clone the repository:

git clone https://github.com/yourusername/TrinityDrive.git
cd TrinityDrive

Launch the motor control service on the Raspberry Pi:

python3 src/RPIMotorService.py

In your ROS2 workspace, launch the teleoperation node:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

If using twist_listener.py, make sure ROS2 and gRPC stubs are set up, then run:

python3 scripts/twist_listener.py

📄 Report

A concise technical report detailing the derivation and implementation is available at:

docs/reporte.pdf

👨‍💻 Contributors

Hugo Daniel Castillo Ovando (A00836025)

[Other team members if applicable]

🏁 Final Notes

This project showcases the integration of theory, control systems, and practical testing to achieve synchronized omnidirectional movement using ROS2 and embedded systems. It also serves as a base for future swarm robotics or path planning extensions.

Made with ❤️ for the Mobile Robots class @ ITESM, 2025.

