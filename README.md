# as2_platform_crazyswarm

ROS 2 platform plugin that bridges [Aerostack2](https://github.com/aerostack2/aerostack2) with [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2), enabling single-drone and swarm control of Crazyflie UAVs through the AS2 framework.

## Architecture

```
Aerostack2 (missions, planners, controllers)
        │
        ▼
as2_platform_crazyswarm   ←─── this package
        │
        ▼
Crazyswarm2 server  (crazyflie_server node)
        │
        ▼
Crazyflie hardware  (via USB Crazyradio PA)
```

The platform node subscribes to AS2 control commands and translates them to Crazyswarm2 topics/services. Sensor data flows in the opposite direction: Crazyswarm2 publishes odometry, status, and IMU; this node reformats and re-publishes them as AS2 sensor streams.

For motion-capture setups an external **mocap bridge** node is also required — it translates rigid-body poses from your mocap system into the `/poses` topic that Crazyswarm2 consumes to feed the drone's Kalman filter.

---

## Requirements

| Dependency | Notes |
|---|---|
| ROS 2 Humble or later | Tested on Humble |
| [Aerostack2](https://github.com/aerostack2/aerostack2) | `as2_core`, `as2_msgs` |
| [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) | `crazyflie_interfaces`, `crazyflie_server_py` |
| [motion\_capture\_tracking](https://github.com/IMRCLab/motion_capture_tracking) | `motion_capture_tracking_interfaces` |
| cflib (`pip install cflib`) | Crazyflie Python driver, required by `crazyflie_server_py` |
| transforms3d (`apt install python3-transforms3d` or `pip install transforms3d`) | Rotation math used by `crazyflie_server_py` |
| ros-humble-tf-transformations (`apt install ros-humble-tf-transformations`) | ROS 2 TF transformation utilities |
| Eigen3 | Standard system package (`apt install libeigen3-dev`) |
| Crazyflie firmware **2026.04 or later** | Earlier firmware causes EKF instability |

---

## Installation

### 1. Install ROS 2

Follow the official instructions for [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) (or later). Make sure `ros-humble-desktop` and `python3-colcon-common-extensions` are installed.

### 2. Install Aerostack2

```bash
sudo apt install ros-humble-as2-core ros-humble-as2-msgs
```

Or build from source:

```bash
mkdir -p ~/crazyflie_ws/src && cd ~/crazyflie_ws/src
git clone https://github.com/aerostack2/aerostack2.git
```

### 3. Install Crazyswarm2 and motion_capture_tracking

Crazyswarm2 and its `motion_capture_tracking` dependency are not on apt — clone both into your workspace:

```bash
cd ~/crazyflie_ws/src
git clone https://github.com/IMRCLab/crazyswarm2.git
git clone https://github.com/IMRCLab/motion_capture_tracking.git
```

Install the Crazyflie Python driver (`cflib`) and `transforms3d`, which are required by the Crazyswarm2 Python server:

```bash
pip install cflib transforms3d
```

### 4. Set up Crazyradio USB permissions

The Crazyradio PA needs a udev rule so it is accessible without root:

```bash
# Download and install the udev rule from the cflib repo
sudo curl -fsSL https://raw.githubusercontent.com/bitcraze/crazyflie-lib-python/master/udev/99-bitcraze.rules \
    -o /etc/udev/rules.d/99-bitcraze.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Log out and back in (or run `sudo usermod -aG plugdev $USER`) for the rule to take effect.

### 5. Clone this package

```bash
cd ~/crazyflie_ws/src
git clone https://github.com/aerostack2/as2_platform_crazyswarm.git
```

### 6. Install remaining system dependencies

```bash
cd ~/crazyflie_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 7. Build

```bash
colcon build --symlink-install --packages-up-to as2_platform_crazyswarm
source install/setup.bash
```

---

## Project config files

In a real project you maintain two config files. This package only provides default templates — in practice you always pass your own files via launch arguments.

### `config/config.yaml` — master AS2 config

This is the single file that every AS2 node in your project reads. The platform node reads only the `cf_name` and `mocap_id` fields from each drone's section; other fields are consumed by the state estimator, motion controller, and behavior nodes.

```yaml
# Global defaults for all nodes
/**:
  platform:
    ros__parameters:
      cmd_freq: 100.0
      info_freq: 10.0
      connection_timeout: 1.0
      multi_ranger_deck: false

# One block per drone — key is the AS2 namespace
drone0:
  platform:
    ros__parameters:
      cf_name: "cf1"    # must match a key under robots: in crazyflies.yaml
      mocap_id: "34"    # rigid-body streaming ID in Motive (only needed with mocap)

drone1:
  platform:
    ros__parameters:
      cf_name: "cf2"
      mocap_id: "35"
```

The swarm launch discovers drone namespaces by reading every top-level key in this file that is not `/**`.

### `config/crazyflies.yaml` — Crazyswarm2 drone registry

This file is read by the **Crazyswarm2 server**, not by this platform node. It maps each `cf_name` to its radio URI and configures firmware logging.

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701   # Crazyradio address printed on the drone
    initial_position: [0.0, 0.0, 0.0]
    type: cf21
    firmware_logging:
      enabled: true
      default_topics:
        odom:
          frequency: 50   # Hz — pose + velocity consumed by this platform
        status:
          frequency: 10   # Hz — battery voltage consumed by this platform
      custom_topics:
        imu:
          frequency: 100  # Hz — acc (G) and gyro (deg/s) consumed by this platform
          vars: [acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z]
```

---

## Per-drone identity chain

Each drone has three identifiers that must be kept consistent across the two config files above:

| Identifier | What it is | Where it is set | Consumed by |
|---|---|---|---|
| **`mocap_id`** | Streaming ID of the rigid body in Motive (e.g. `"34"`) | `config.yaml` per-drone block | `mocap_bridge` node |
| **`cf_name`** | Logical name in Crazyswarm2 (e.g. `"cf1"`) | `config.yaml` per-drone block **and** `crazyflies.yaml` robot key | This platform node + Crazyswarm2 server |
| **Radio URI** | Hardware address of the Crazyradio link | `crazyflies.yaml` under `robots.<cf_name>.uri` | Crazyswarm2 server |

The full chain for one drone:

```
OptiTrack Motive         config.yaml                      crazyflies.yaml
────────────────         ───────────                      ───────────────
rigid body "34" ─────▶  drone0:                          robots:
                           platform:                        cf1:
                             cf_name: "cf1"  ────────────▶    uri: radio://0/80/2M/E7E7E7E701 ──▶ hardware
                             mocap_id: "34"
```

When you add or swap a drone, update both files in sync:
1. Assign a rigid-body streaming ID in Motive and note it as `mocap_id`.
2. Add the drone to `crazyflies.yaml` with its `cf_name` and radio URI.
3. Add the drone namespace to `config.yaml` with matching `cf_name` and `mocap_id`.

---

## Usage — single drone

### Step 1 — Configure `crazyflies.yaml`

Add your drone with its radio URI and the required firmware logging (see template above).

### Step 2 — Start the Crazyswarm2 server

```bash
ros2 launch crazyflie launch.py
```

### Step 3 — Launch the platform

```bash
ros2 launch as2_platform_crazyswarm crazyswarm_launch.py \
    namespace:=drone0 \
    cf_name:=cf1
```

`cf_name` must match the key in `crazyflies.yaml`. If omitted it defaults to `namespace`.

---

## Usage — swarm

### Step 1 — Configure both files

Add all drones to `crazyflies.yaml` and to your project `config.yaml`.

### Step 2 — Start the Crazyswarm2 server

Without mocap:
```bash
ros2 launch crazyflie launch.py
```

With mocap (also starts the `mocap_bridge` node):
```bash
ros2 launch <your_project>/launch/crazyswarm_server.launch.py mocap:=true
```

### Step 3 — Launch the swarm platform

```bash
ros2 launch as2_platform_crazyswarm crazyswarm_swarm_launch.py \
    swarm_config_file:=config/config.yaml
```

One `CrazyswarmPlatform` node is spawned per drone namespace found in `config.yaml`. The `/**` block provides shared defaults; per-drone blocks provide `cf_name` (and optionally `mocap_id`).

---

## Motion capture setup

The `mocap_bridge` node translates rigid-body poses from your mocap system into the `/poses` topic that Crazyswarm2 uses to feed each drone's Kalman filter.

It subscribes to `/mocap/rigid_bodies` (`mocap4r2_msgs/RigidBodies`) and republishes on `/poses` (`motion_capture_tracking_interfaces/NamedPoseArray`). At startup it reads `config.yaml`, building a `mocap_id → cf_name` mapping from each drone's `platform.ros__parameters` block. Rigid bodies whose streaming ID is not in the map are forwarded with their original name unchanged.

The `mocap_bridge` must receive poses at 100 Hz or faster — the Crazyswarm2 server subscribes to `/poses` with a 100 Hz QoS deadline.

---

## Platform parameters reference

These parameters live under `<namespace>.platform.ros__parameters` in your `config.yaml` (or in the package's `config/platform_config_file.yaml` for the single-drone launch).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cf_name` | `string` | namespace (no `/`) | Crazyswarm2 robot name. Must match a key under `robots:` in `crazyflies.yaml`. |
| `mocap_id` | `string` | — | Rigid-body streaming ID in Motive. Only read by the `mocap_bridge` node. |
| `cmd_freq` | `double` | `100.0` | Rate (Hz) at which command setpoints are sent to Crazyswarm2. |
| `info_freq` | `double` | `10.0` | Rate (Hz) at which platform info (state, control mode) is published. |
| `connection_timeout` | `double` | `1.0` | Seconds without an odometry message before the drone is declared disconnected. |
| `multi_ranger_deck` | `bool` | `false` | Set to `true` if a Multi-ranger deck is mounted. Enables the LaserScan sensor stream. |

---

## Launch arguments

### `crazyswarm_launch.py` — single drone

| Argument | Default | Description |
|---|---|---|
| `namespace` | `drone0` | AS2 namespace for this drone |
| `cf_name` | `""` (→ namespace) | Crazyswarm2 robot name |
| `control_modes_file` | package default | Path to `control_modes.yaml` |
| `platform_config_file` | package default | Path to `platform_config_file.yaml` |

### `crazyswarm_swarm_launch.py` — swarm

| Argument | Default | Description |
|---|---|---|
| `swarm_config_file` | package default | Path to the project `config.yaml` (or any YAML with the same structure) |
| `control_modes_file` | package default | Path to `control_modes.yaml` |
| `platform_config_file` | package default | Path to `platform_config_file.yaml` (provides shared defaults) |

---

## ROS 2 interface

### Topics published by this node (per drone, under `/<namespace>/`)

| Topic | Type | Description |
|---|---|---|
| `sensor_measurements/imu` | `sensor_msgs/Imu` | IMU (accelerometer + gyroscope) |
| `sensor_measurements/odom` | `nav_msgs/Odometry` | Odometry from Crazyswarm2 |
| `sensor_measurements/battery` | `sensor_msgs/BatteryState` | Battery voltage |
| `sensor_measurements/lidar/scan` | `sensor_msgs/LaserScan` | Multi-ranger scan (only if `multi_ranger_deck: true`) |
| `platform/info` | `as2_msgs/PlatformInfo` | Platform state (armed, offboard, control mode) |

### Topics subscribed from Crazyswarm2 (under `/<cf_name>/`)

| Topic | Type | Description |
|---|---|---|
| `/<cf_name>/odom` | `nav_msgs/Odometry` | Odometry from Crazyswarm2 server |
| `/<cf_name>/status` | `crazyflie_interfaces/Status` | Battery and status from firmware |
| `/<cf_name>/imu` | `crazyflie_interfaces/LogDataGeneric` | IMU log block from firmware |
| `/<cf_name>/scan` | `sensor_msgs/LaserScan` | Multi-ranger scan (optional) |

### Topics published to Crazyswarm2 (under `/<cf_name>/`)

| Topic | Type | Description |
|---|---|---|
| `/<cf_name>/cmd_position` | `crazyflie_interfaces/Position` | Position + yaw setpoint |
| `/<cf_name>/cmd_velocity_world` | `crazyflie_interfaces/VelocityWorld` | World-frame velocity setpoint |
| `/<cf_name>/cmd_hover` | `crazyflie_interfaces/Hover` | Horizontal velocity + altitude hold |

### Services called (under `/<cf_name>/`)

| Service | Type | Description |
|---|---|---|
| `/<cf_name>/arm` | `crazyflie_interfaces/Arm` | Arm/disarm the drone |
| `/<cf_name>/emergency` | `std_srvs/Empty` | Kill switch — cuts motors immediately |
| `/<cf_name>/notify_setpoints_stop` | `crazyflie_interfaces/NotifySetpointsStop` | Signal end of offboard setpoints |

---

## License

BSD-3-Clause. See [LICENSE](LICENSE) for details.

Copyright 2024 Universidad Politécnica de Madrid.
