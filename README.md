#hra_miku_robot

ROBOT_OS v5.0 — Human Robot Artem (HRA)

Human Robot Artem (HRA) is an advanced operating system for a humanoid robot powered by NVIDIA Jetson Nano.
It unifies real-time balance control, computer vision, local AI cognition, and multilingual voice interaction into a single robotic framework.

🌟 Key Features
🧠 Neural Brain

Local LLM powered by Ollama (Phi-3)

Context-aware intelligent dialogue

Fully offline AI reasoning

⚖️ Active Stability

Real-time PID control loop

IMU-based balancing using MPU6050

Control of 4 servos (hips & knees)

👁️ RealSense Vision

Intel RealSense D435

High-speed RGB + depth processing

Vision pipeline optimized for Jetson

🗣️ Polyglot Miku Voice

Automatic language detection (RU / EN / JA)

Voice synthesis via Hatsune Miku TTS API

Dynamic language switching

👂 Coqui STT Hearing

Local speech-to-text processing

Low-latency voice command recognition

🔒 Secure Control

Encrypted TCP control server

Fernet (AES-128) encryption

Secure remote operation

🛡️ Reflex System

GPIO-based touch sensors

Immediate “pain” / impact reflex

Highest execution priority

🏗 System Architecture

ROBOT_OS uses a multi-threaded architecture to ensure critical processes (like balance) are never blocked by heavy AI workloads.

🔄 Threads Overview

Physics Thread (50 Hz)

IMU polling

PID balance correction

Servo control

TCP Thread

Encrypted command listener

Remote control interface

Main Loop

Vision pipeline

Speech recognition

Ollama reasoning

Voice synthesis

🔌 Hardware Map
Component	Interface	Description
MPU6050	I2C (SDA1 / SCL1)	Gyro + accelerometer (balance)
Left Hip Servo	Pin 24	Hip joint
Right Hip Servo	Pin 32	Hip joint
Left Knee Servo	Pin 26	Knee joint
Right Knee Servo	Pin 33	Knee joint
Touch Sensor	Pin 7	Pain / impact reflex
Camera	USB 3.0	Intel RealSense D435
🚀 Quick Start
1️⃣ Install Dependencies
pip install mpu6050-raspberrypi pyrealsense2 sounddevice soundfile cryptography ollama gradio_client opencv-python

2️⃣ Setup Ollama
curl -fsSL https://ollama.com/install.sh | sh
ollama pull phi3

3️⃣ Run the OS
git clone https://github.com/yourname/hra_miku_robot.git
cd hra_miku_robot
python3 main.py

📝 Voice Commands (Examples)
Command	Action
"Go forward"	Initiates walking gait
"Stand straight"	Resets servos to 90° and engages PID balance
"How are you?"	Switches to English TTS and replies with AI status
"何時ですか？"	Switches to Japanese TTS and reports the time
📜 License

This project is licensed under the MIT License — free to use, modify, and expand.

👨‍💻 Developer

Artem — HRA Project

🚧 Project Status

Active Development — v5.0

💡 Possible Next Upgrades

📊 Architecture diagrams (Mermaid)

🎥 Balance & walking demo GIFs

🧩 Modular plugin system

🧠 Long-term memory for LLM

🤖 ROS2 integration bridge
