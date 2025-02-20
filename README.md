# 🚀 Differential Drive PID Controller

A ROS 2 package implementing a PID controller for a differential drive robot. It regulates wheel RPM by minimizing errors and publishing corrected velocity commands. Includes real-time RPM visualization using Matplotlib for monitoring performance. Ideal for precise speed control and stability in robotics applications.

---

## ✨ Features
- 🏎️ **PID Control**: Ensures smooth and precise motor speed regulation.
- 📡 **ROS 2 Integration**: Fully compatible with ROS 2 framework.
- 📊 **Real-time RPM Visualization**: Graphical representation using Matplotlib.
- 🛠️ **Customizable Parameters**: Tune PID gains for optimal performance.
- 🤖 **Works with Differential Drive Robots**: Suitable for mobile robots and AGVs.

---

## 📁 Project Structure
```
differential_drive_controller/
│── CMakeLists.txt
│── package.xml
│── launch/
│   ├── gazebo.launch.py
│   ├── state_publisher.launch.py
│── models/
│   ├── meshes
│   ├── urdf
│── src/
│   ├── script_a.cpp                   # RPM calculation from /cmd_vel and publish /left_wheel_rpm , /right_wheel_rpm.
│── scripts/
│   ├── rpm_visualizer.py              # Visualization through Matplotlib.
│   ├── waypoint_navigation.py         # Navigate the robot to two waypoints sequentially.
│── include/
│   └── differential_drive_controller
│── README.md
```

---

## ⚙️ Installation & Setup

### 🔹 Prerequisites
Make sure you have the following installed:
- **ROS 2 (Humble)** 🤖
- **C++17 & Python3** 🛠️
- **Colcon Build System** ⚙️
- **Matplotlib, Seaborn (for visualization)** 📊
- **Gazebo multi-robot simulator, version 11.10.2** 🔄
- **Requests, Plotly, Numpy (Python Libraries)** 📊
  
### 🔹 Setup Instructions
## ✅ Install ROS Dependencies
Install ROS 2 (if not already installed)
```bash
sudo apt update && sudo apt install -y ros-humble-desktop
```
Source ROS 2 setup file
```bash
source /opt/ros/humble/setup.bash
```
Create a ROS 2 workspace
```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
```
Clone the repository
```bash
git clone https://github.com/yashbhaskar/differential_drive_pid_controller.git
```
Navigate to the workspace
```bash
cd ~/ros_ws/
```
Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```
Build the package
```bash
colcon build --packages-select rse_assignment
```
Source the workspace
```bash
source install/setup.bash
```
## ✅ Install C++ Dependencies
Install C++ Compiler and Build Tools:
```bash
sudo apt update && sudo apt install -y build-essential cmake g++ gcc
```
Run the following command to install necessary C++ libraries:
```bash
sudo apt update && sudo apt install -y libhttplib-dev libjsoncpp-dev libboost-all-dev cmake
```
## ✅ Install Python Dependencies
Install Python and Pip:
```bash
sudo apt install -y python3 python3-pip python3-venv
```
Use pip to install the required Python packages:
```bash
pip install requests dash plotly numpy matplotlib
```

---

## 🚀 Usage

### Launch the System
```bash
ros2 launch differential_drive_pid pid_controller.launch.py
```
- **launch gazebo**
  ```bash
  ros2 launch differential_drive_controller gazebo.launch.py 
  ```
- **launch state-pubisher**
  ```bash
  ros2 launch differential_drive_controller state_publisher.launch.py 
  ```
- **Run script_a.cpp for rpm calculation from /cmd_vel and publish /left_wheel_rpm , /right_wheel_rpm.**
  ```bash
  ros2 run differential_drive_controller script_a 
  ```
- **Run script waypoint_navigation.py for navigate the robot to two waypoints sequentially**
  ```bash
  ros2 run differential_drive_controller waypoint_navigation.py
  ```
- **Run Visualization on GUI through Matplotlib**
  ```bash
  ros2 run differential_drive_controller rpm_visualizer.py
  ```


---

## 📊 Visualization
Run the following command to see the real-time RPM plot:
```bash
ros2 run differential_drive_controller rpm_visualizer.py
```
You should see a Matplotlib window displaying the RPM data of both wheels dynamically.

---

## 🤝 Contributing
Feel free to submit pull requests, open issues, and suggest improvements!

---

## 📜 License
This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## 📧 Contact
For questions, reach out via GitHub Issues or email me at `your.email@example.com`. 🚀

