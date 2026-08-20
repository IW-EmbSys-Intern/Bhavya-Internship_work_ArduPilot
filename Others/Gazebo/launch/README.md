# 🛩️ Plane & Drone Simulation Bridges

ROS 2 launch files that bring up **MAVROS** and **Gazebo → ROS 2 topic bridges** for an ArduPilot SITL simulation. Two variants are provided depending on whether you're flying just the fixed-wing plane, or the plane and a multirotor together.

| File | Vehicles | Use case |
|---|---|---|
| [`plane_bridges.launch.py`](./plane_bridges.launch.py) | Plane only | Single-vehicle plane missions |
| [`plane_and_drone_bridges.launch.py`](./plane_and_drone_bridges.launch.py) | Plane + Copter | Dual-vehicle plane/drone missions |

---

## 📦 Prerequisites

Before launching, make sure the following are installed and working:

- **ROS 2** (Humble or newer recommended)
- [`mavros`](https://github.com/mavlink/mavros) — with the `apm.launch` file available in its share directory
- [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz) — for the Gazebo ↔ ROS 2 topic bridges
- **ArduPilot SITL**, running the `runway` world with:
  - a `skywalker_x8` model (plane, front-facing camera) — required by **both** launch files
  - an `iris_I1` model with a gimbal-mounted camera (copter) — required **only** by the dual-vehicle launch file
- GeographicLib datasets installed for MAVROS (`install_geographiclib_datasets.sh`), otherwise MAVROS will fail to start cleanly

> ⚠️ **World/model names are hardcoded.** The Gazebo topic paths (`/world/runway/model/skywalker_x8/...` and `/world/runway/model/iris_I1/...`) assume your simulation uses these exact world and model names. If your world or model names differ, edit the `arguments` list in the corresponding `Node` before launching.

---

## 1️⃣ `plane_bridges.launch.py` — Plane Only

Brings up a single MAVROS instance connected to the plane's SITL, plus the plane's camera bridges.

### What it starts
| Component | Details |
|---|---|
| MAVROS | Includes `mavros`'s own `apm.launch`, connected via `fcu_url` |
| `camera_info_bridge` | Bridges Gazebo `CameraInfo` → ROS 2 `sensor_msgs/msg/CameraInfo` |
| `image_bridge` | Bridges Gazebo `Image` → ROS 2 `sensor_msgs/msg/Image`, remapped to **`/camera/image_raw`** |

### Launch arguments
| Argument | Default | Description |
|---|---|---|
| `fcu_url` | `udp://:14550@` | FCU connection URL for MAVROS |

### Usage
```bash
ros2 launch <your_package> plane_bridges.launch.py
```

Override the FCU URL if needed:
```bash
ros2 launch <your_package> plane_bridges.launch.py fcu_url:=udp://:14550@127.0.0.1:14550
```

---

## 2️⃣ `plane_and_drone_bridges.launch.py` — Plane + Copter

Brings up **two** MAVROS instances (one per vehicle, each on its own SITL UDP port) plus camera bridges for both vehicles.

### What it starts
| Component | Namespace | Details |
|---|---|---|
| Plane MAVROS | `plane/mavros` | SITL instance 0, connects via `plane_fcu_url` |
| Copter MAVROS | `copter/mavros` | SITL instance 1, connects via `copter_fcu_url` |
| `camera_info_bridge` / `image_bridge` | — | Plane's front camera → **`/camera/image_raw`** |
| `copter_camera_info_bridge` / `copter_image_bridge` | — | Copter's gimbal camera → **`/copter/camera/image_raw`** |

### Launch arguments
| Argument | Default | Description |
|---|---|---|
| `plane_fcu_url` | `udp://:14550@` | FCU connection URL for the plane's MAVROS instance |
| `copter_fcu_url` | `udp://:14560@` | FCU connection URL for the copter's MAVROS instance |

### Usage
```bash
ros2 launch <your_package> plane_and_drone_bridges.launch.py
```

Override either FCU URL if your ports differ:
```bash
ros2 launch <your_package> plane_and_drone_bridges.launch.py \
    plane_fcu_url:=udp://:14550@ \
    copter_fcu_url:=udp://:14560@
```

### Talking to a specific vehicle
Each downstream mission node needs to be pointed at the correct MAVROS namespace:

```bash
# Copter mission node
ros2 run <pkg> mavros_mission_node.py --ros-args -r __ns:=/copter/mavros -p target_lat:=... 

# Plane mission node
ros2 run <pkg> plane_mission_node.py --ros-args -r __ns:=/plane/mavros
```

---

## 🔍 Topic Reference

| Topic | Type | Vehicle |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Plane |
| `.../front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Plane |
| `/copter/camera/image_raw` | `sensor_msgs/msg/Image` | Copter (dual launch only) |
| `.../gimbal/link/pitch_link/sensor/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Copter (dual launch only) |
| `/plane/mavros/...` | MAVROS topics/services | Plane (dual launch only) |
| `/copter/mavros/...` | MAVROS topics/services | Copter (dual launch only) |

> Note: in the single-vehicle launch file, MAVROS is **not namespaced**, so its topics appear directly under `/mavros/...` rather than `/plane/mavros/...`.

---

## 🛠️ Troubleshooting

- **MAVROS never connects / no heartbeat** — double-check your SITL instance is actually publishing on the UDP port referenced by `fcu_url` / `plane_fcu_url` / `copter_fcu_url`, and that nothing else (e.g. another MAVROS instance or QGroundControl) is already bound to that port.
- **No image on `/camera/image_raw` or `/copter/camera/image_raw`** — confirm Gazebo is actually publishing the source topic (`gz topic -l | grep camera`) and that the model/world names match what's hardcoded in the bridge `arguments`.
- **`apm.launch` not found** — verify `mavros` is installed and sourced (`ros2 pkg prefix mavros`); the launch files locate it via `get_package_share_directory("mavros")`.
- **Two SITL instances stepping on each other** — the dual-vehicle launch expects SITL **instance 0** for the plane and **instance 1** for the copter, matching the default `14550` / `14560` ports.

---

## 📁 Suggested Package Layout
```
your_package/
├── launch/
│   ├── plane_bridges.launch.py
│   └── plane_and_drone_bridges.launch.py
├── package.xml
└── setup.py
```
