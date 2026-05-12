# Bridge for KUKA Med LBR 7

## Branches

| Branch | Description |
|---|---|
| `main` | C++ standalone library (TCP bridge) |
| `ros1` | ROS1 node built on top of the bridge |

---

## `ros1` branch — ROS1 Node

This branch adds a ROS1 interface for the KUKA Med LBR 7.
For full documentation (topics, control modes, usage) see [ros_node/README.md](ros_node/README.md).

---

## C++ Bridge

This repo contains the C++ library for communicating with the KUKA Med through a TCP socket.

### Structure

```
kuka_med_bridge/
├── CMakeLists.txt
├── include/
│   ├── RobotData.hpp       ← Structs definition
│   ├── TcpServer.hpp       ← Communication interface header
│   └── Controller.hpp      ← Controller header
├── src/
│   ├── TcpServer.cpp       ← Socket code
│   ├── Controller.cpp      ← Main controller
│   └── main.cpp
└── ros_node/               ← ROS1 package (this branch)
    ├── CMakeLists.txt
    ├── package.xml
    └── src/
        ├── med_driver.cpp
        └── demo_node.cpp
```
