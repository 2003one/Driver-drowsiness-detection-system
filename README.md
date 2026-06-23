# Driver Monitoring System (DMS) — Edge AI Drowsiness Detection

Real-time, edge-deployed driver drowsiness detection. A hybrid **EAR + CNN** pipeline
(via MediaPipe) plus an **unsupervised anomaly ensemble** runs on a **Raspberry Pi 4**,
serves driver states over a **Flask API**, and drives a **TurtleBot3** in **Gazebo**
through a **ROS2** bridge node.

The project has **two parts that run on two machines**:

| Folder    | Runs on            | Role                                                        |
|-----------|--------------------|------------------------------------------------------------|
| `car/`    | **Raspberry Pi 4** | Camera capture + AI inference (MediaPipe, CNNs, ensemble) + Flask API |
| `dms_ws/` | **Linux host**     | ROS2 workspace — bridge node + TurtleBot3 / Gazebo control |

The Pi runs the perception; the Linux host runs the robot. They talk over the network
(Flask API → ROS2 bridge).

States map to vehicle behaviour:

| State  | Trigger (PERCLOS-based) | Action                |
|--------|-------------------------|-----------------------|
| ACTIVE | low fatigue             | full teleop control   |
| DROWSY | rising fatigue          | speed reduced to ~50% |
| DANGER | sustained closure       | emergency stop        |

---

## Architecture (overview)

```
[ Raspberry Pi 4 ]                         [ Linux host ]
Camera
  └─ MediaPipe FaceLandmarker → eye landmarks
       ├─ EAR (geometry)  ┐
       │                  ├─ hybrid open/closed decision
       ├─ CNN (V1 / V2)   ┘
       ├─ PERCLOS (rolling 900-frame window) → fatigue score
       └─ Anomaly ensemble (KMeans · GMM · IsolationForest · OneClassSVM)
             │
        Flask API  ───────(network)───────►  ROS2 bridge node
                                                   │
                                                   ▼
                                          TurtleBot3 (Gazebo)
```

---

## Repository structure

```
.
├── car/                        # ── Raspberry Pi ── inference + Flask API
│   ├── realtime.py             # main real-time loop
│   └── requirements.txt        # Pi Python dependencies
├── dms_ws/                     # ── Linux host ── ROS2 workspace
│   └── src/
│       └── dms_controller/     # ROS2 bridge package (dms_bridge_node)
├── training/                   # model training scripts (V1 CNN, V2 MobileNetV2)
├── models/                     # trained weights (via Release / Git LFS — not committed)
└── README.md
```

> Build artifacts (`dms_ws/build/`, `dms_ws/install/`, `dms_ws/log/`) and the Pi
> virtualenv are **not committed** — they are regenerated locally (see setup below).

---

## Prerequisites

**Raspberry Pi 4** (perception)
- 5V/3A supply, camera, Python **3.11**

**Linux host** (robot)
- Ubuntu 24.04, ROS2 **Jazzy**, Gazebo, TurtleBot3 packages:

```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-turtlebot3-gazebo
```

> Package names can vary by ROS2 distro — confirm against your `apt` if a launch file is missing.

---

## Setup — Raspberry Pi (`car/`)

```bash
cd ~/car
python3.11 -m venv ~/dms_env311
source ~/dms_env311/bin/activate
pip install -r requirements.txt
```

---

## Setup — Linux host (`dms_ws/`)

If you are building the workspace from scratch:

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/dms_ws/src
cd ~/dms_ws/src
ros2 pkg create --build-type ament_python dms_controller \
    --dependencies rclpy geometry_msgs std_msgs
```

Place the bridge node at
`~/dms_ws/src/dms_controller/dms_controller/dms_bridge_node.py` (must expose `main()`),
and register it in `~/dms_ws/src/dms_controller/setup.py`:

```python
entry_points={
    'console_scripts': [
        'dms_bridge_node = dms_controller.dms_bridge_node:main',
    ],
},
```

Build and source (also do this after cloning the repo):

```bash
cd ~/dms_ws
colcon build --packages-select dms_controller
source install/setup.bash
```

> Tip: add `source ~/dms_ws/install/setup.bash` to your `~/.bashrc`.

---

## Running the system

**Terminal 1 — Gazebo (host)**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 — Teleop, remapped (host)**
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args --remap /cmd_vel:=/cmd_vel_teleop
```

**Terminal 3 — DMS bridge node (host)**
```bash
cd ~/dms_ws
source install/setup.bash
ros2 run dms_controller dms_bridge_node
```

**Raspberry Pi — real-time inference + Flask API**
```bash
cd ~/car
source ~/dms_env311/bin/activate
LIBGL_ALWAYS_SOFTWARE=1 python3 realtime.py
```

> **Why teleop is remapped:** keyboard teleop publishes to `/cmd_vel_teleop` instead of
> `/cmd_vel`. The bridge node subscribes to it, applies the current driver state
> (full / 50% / stop), and republishes to `/cmd_vel` — so the DMS gates manual control
> instead of fighting it.
>
> `LIBGL_ALWAYS_SOFTWARE=1` forces software GL rendering on the Pi (no GPU path needed).

---

## Training (optional)

Model training lives in `training/`:

- `train_v1_cnn.py` — custom CNN (grayscale 32×32×1, from scratch)
- `train_v2_mobilenet.py` — MobileNetV2 transfer learning (RGB)

Trained weights are distributed via **GitHub Releases / Git LFS** (not committed).
The dataset is linked in `training/README.md` rather than stored in the repo.

```bash
cd training
python train_v2_mobilenet.py
```

---

## Limitations

- Single-developer project; thresholds tuned on limited data.

## Roadmap

- Port the inference runtime to a C++ ROS2 node for production perception.
- Real-hardware deployment.
# Driver-drowsiness-detection-system
