# 🛩️ Skywalker X8 Gazebo + ArduPilot + ROS 2 Setup

This repository provides the required files to enable the **SKYWALKER_X8** model with a front camera and the necessary simulation parameters for use with **Gazebo**, **ArduPilot SITL**, **MAVROS**, and **ROS 2**.

---

## 📦 Prerequisites

Clone the official **ArduPilot SITL Models** repository:

```bash
git clone https://github.com/ArduPilot/SITL_Models.git
```

Copy the files from this repository into the corresponding directories of the cloned `SITL_Models` repository.

This adds:

* 📷 Front camera
* ✈️ Skywalker X8 model modifications
* ⚙️ Required parameter files
* 🌍 Gazebo world configuration

---

# 🚀 Running the Simulation

## 1. Export Gazebo Resource Path

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds
```

---

## 2. Launch Gazebo

```bash
gz sim -v4 -r skywalker_x8_runway.sdf
```

---

## 3. Launch ArduPilot SITL

From your **ArduPilot** directory:

```bash
sim_vehicle.py \
-v ArduPlane \
--model JSON \
--add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param \
--console \
--map
```

---

## 4. Launch MAVROS

```bash
ros2 launch mavros apm.launch \
fcu_url:=udp://:14550@
```

---

# 📷 Camera Verification

Verify that Gazebo is publishing the camera topic:

```bash
gz topic -l | grep camera
```

---

# 🔄 Bridge Gazebo Camera to ROS 2

Run the parameter bridge using the exact Gazebo camera topic:

```bash
ros2 run ros_gz_bridge parameter_bridge \
"/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
--ros-args \
-r /world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/image:=/camera/image_raw
```

The ROS 2 image topic will be available at:

```
/camera/image_raw
```

---

# 📌 Optional: Bridge Camera Info

Recommended for image-processing pipelines that require camera calibration information.

```bash
ros2 run ros_gz_bridge parameter_bridge \
/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
--ros-args \
-r /camera/camera_info:=/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/camera_info
```

---

# 🖼️ View the Camera Feed

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```
/camera/image_raw
```

---

# 🛩️ Mini Talon V-Tail Configuration

Launch the Mini Talon:

```bash
sim_vehicle.py \
-v ArduPlane \
-f gazebo-zephyr \
--model JSON \
--console \
--map \
--out=udp:127.0.0.1:14550
```

Configure the V-Tail servo functions:

```bash
param set SERVO2_FUNCTION 79
param set SERVO4_FUNCTION 80
```

Where:

| Parameter         | Function          |
| ----------------- | ----------------- |
| `SERVO2_FUNCTION` | `79` → VTailLeft  |
| `SERVO4_FUNCTION` | `80` → VTailRight |

Save the parameters:

```bash
param save
```

---

# 📡 ROS Topics

| Topic                 | Description                    |
| --------------------- | ------------------------------ |
| `/camera/image_raw`   | Front camera image             |
| `/camera/camera_info` | Camera calibration information |

---

# ✅ Simulation Workflow

```text
Gazebo
    │
    ▼
Skywalker X8 + Camera
    │
    ▼
ros_gz_bridge
    │
    ▼
ROS 2 (/camera/image_raw)
    │
    ▼
YOLO Detection Node
```

---

# 📁 Tested With

* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo Harmonic
* ArduPilot SITL
* MAVROS, should also work with pymavlink
* Ultralytics YOLOv8
