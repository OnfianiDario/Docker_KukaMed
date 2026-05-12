# KUKA Med LBR 7 — ROS1 Node

ROS1 interface for the KUKA Med LBR 7 robot, built on top of the [kuka_med_bridge](https://github.com/ARSControl/kuka_med_bridge) C++ library.

## Requirements

- ROS1 (tested on Noetic)
- `roscpp`, `sensor_msgs`, `std_msgs`, `geometry_msgs`
- The bridge C++ library (included in this repo: `src/` and `include/`)

## Package structure

```
ros_node/
├── CMakeLists.txt
├── package.xml
├── README.md
└── src/
    ├── med_driver.cpp   ← main driver node
    └── demo_node.cpp    ← interactive demo
```

## Build

Place the package inside your catkin workspace and build:

```bash
cd ~/your_ws
catkin_make
source devel/setup.bash
```

## Usage

### 1. Start the driver

```bash
rosrun kuka_ros_node med_driver _port:=5005
```

The default port is `5005`. Change it to match the port configured on the robot side.

### 2. Run the demo (optional)

```bash
rosrun kuka_ros_node demo_node
```

An interactive menu will let you test the available motion modes.

---

## Topics

### Published by `med_driver`

| Topic | Type | Description |
|---|---|---|
| `/robot/state/joint_states` | `sensor_msgs/JointState` | Joint positions and measured torques |
| `/robot/state/cart_pose` | `std_msgs/Float64MultiArray` | Cartesian pose `[x, y, z, A, B, C]` (mm / deg) |
| `/robot/state/ext_wrench` | `geometry_msgs/WrenchStamped` | External wrench at end-effector |
| `/robot/state/ext_torques` | `std_msgs/Float64MultiArray` | External joint torques |
| `/robot/state/meas_torques` | `std_msgs/Float64MultiArray` | Measured joint torques |

### Subscribed by `med_driver`

| Topic | Type | Description |
|---|---|---|
| `/robot/control_mode` | `std_msgs/Int32` | Set active control mode (see table below) |
| `/robot/cmd/joint_ptp` | `std_msgs/Float64MultiArray` | Joint PTP: `[q0..q6, velocity]` (8 values) |
| `/robot/cmd/cart_ptp` | `std_msgs/Float64MultiArray` | Cartesian PTP: `[x, y, z, A, B, C, velocity]` (7 values) |
| `/robot/cmd/joint_stiffness` | `std_msgs/Float64MultiArray` | Joint stiffness: `[k0..k6]` (7 values) |
| `/robot/cmd/joint_damping` | `std_msgs/Float64MultiArray` | Joint damping: `[d0..d6]` (7 values) |
| `/robot/cmd/cart_stiffness` | `std_msgs/Float64MultiArray` | Cartesian stiffness: `[kx, ky, kz, krx, kry, krz]` (6 values) |
| `/robot/cmd/cart_damping` | `std_msgs/Float64MultiArray` | Cartesian damping: `[dx, dy, dz, drx, dry, drz]` (6 values, range [0,1]) |
| `/robot/cmd/ptp_with_impedance` | `std_msgs/Float64MultiArray` | PTP motion with impedance (see layout below) |

---

## Control modes

| Value | Mode |
|---|---|
| `0` | Joint PTP |
| `1` | Cartesian PTP |
| `2` | Joint Impedance |
| `3` | Cartesian Impedance |

---

## `/robot/cmd/ptp_with_impedance` layout

The message combines a PTP target with impedance parameters in a single array:

```
[motionType, impedanceType, target..., velocity, stiffness..., damping...]
```

| motionType | impedanceType | target size | stiffness/damping size | total values |
|---|---|---|---|---|
| 0 (Joint PTP) | 2 (Joint Imp.) | 7 | 7 | **17** |
| 0 (Joint PTP) | 3 (Cart. Imp.) | 7 | 6 | **16** |
| 1 (Cart. PTP) | 2 (Joint Imp.) | 6 | 7 | **16** |
| 1 (Cart. PTP) | 3 (Cart. Imp.) | 6 | 6 | **15** |
