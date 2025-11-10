# Adaptive User-Responsive Automaton (AURA) ROS2 Humble Hawksbill

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?logo=ubuntu)
![VMware](https://img.shields.io/badge/VMware-Workstation-607078?logo=vmware)


> Adaptive, user-driven robotic automation for small-scale industrial workflows — a ROS 2 (Humble) + MoveIt 2 project implementing custom services, actions, and interactive control for pick/place and motion tasks.

---

## Table of Contents

- [Project Overview](#project-overview)  
- [Motivation & Use Case](#motivation--use-case)  
- [Key Features](#key-features)  
- [Architecture & Components](#architecture--components)  
- [Repository Layout](#repository-layout)  
- [Prerequisites](#prerequisites)  
- [Install & Build (End-to-End)](#install--build-end-to-end)  
- [Run / Demo Flow (Phases 1–4)](#run--demo-flow-phases-1-4)  
- [Service & Action API](#service--action-api)  
- [How to Use — Examples](#how-to-use---examples)  
- [Video Demo & Screenshots](#-🎥-project-demo)  
- [Troubleshooting](#troubleshooting)  
- [Contributing](#contributing)  
- [License](#license)  
- [Acknowledgements](#acknowledgements)

---

## Project Overview

**Adaptive User-Responsive Automaton (AURA)** is an end-to-end robotic control framework built with ROS 2 (Humble) and MoveIt 2 that enables user-driven, adaptive automation for repeated industrial tasks such as trimming, capping, and packaging. The project is designed to be modular so small and medium-scale industries (MSMEs) can adopt robotic arms to automate post-processing tasks.

This repository contains:
- Custom ROS 2 interfaces (services & actions)
- A control node that translates user requests into motion plans
- A real-time keyboard control node for manual intervention
- Integration with MoveIt 2 for motion planning & visualization (RViz)

---

## Motivation & Use Case

In a plastic-bottle manufacturing factory in Ulhasnagar, molding is automated but subsequent processes (edge trimming, capping, packaging) were manual and repetitive. AURA aims to:
- Reduce manual labor
- Improve throughput & consistency
- Offer an affordable, reprogrammable automation solution for MSMEs

---

## Key Features

- Modular service & action interfaces for simple commands and long-running goals  
- Central controller (`moveit_service_action`) that links user intent to motion planning  
- Real-time keyboard control for manual operator interaction  
- RViz integration for motion visualization and debugging  
- Easy-to-adapt architecture for new tasks and different robot models

---

## Architecture & Components

- `panda_interfaces` — custom ROS 2 interface package (srv / action definitions)  
- `panda_keyboard_control` — control node(s): service/action server, keyboard control, command publisher  
- MoveIt 2 demos (used as workspace foundation for motion planning) — launched during Phase 1  
- RViz2 — visualization & debugging

**High-level flow**:
1. Start MoveIt / planning scene  
2. Start `moveit_service_action` server — accepts service/action requests  
3. Send service call (`MoveCommand`) for quick state transitions  
4. Send action goal (`PickPlace`) for prolonged tasks with feedback  
5. Optional: use `keyboard_control` for manual inputs and monitor `/panda_command` topic

---

## Repository Layout

```
adaptive-user-responsive-automaton/
├─ README.md
├─ LICENSE
├─ src/
│  ├─ panda_interfaces/
│  │  ├─ package.xml
│  │  ├─ srv/
│  │  │  └─ MoveCommand.srv
│  │  └─ action/
│  │     └─ PickPlace.action
│  ├─ panda_keyboard_control/
│  │  ├─ package.xml
│  │  ├─ src/
│  │  │  ├─ moveit_service_action.py
│  │  │  └─ keyboard_control.py
│  │  └─ launch/
│  │     └─ demo_launch.py
│  └─ (optional) other helper packages
└─ scripts/
   └─ run_demo_tmux.sh
```

---

## Prerequisites

Tested on:
- Ubuntu 22.04 (VMware/physical)
- ROS 2 Humble
- MoveIt 2 for Humble
- Python 3.10 (ROS 2 Humble default)
- `colcon` build tools

Install ROS 2 Humble / MoveIt 2 following official guides. Make sure your environment has:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep
sudo rosdep init
rosdep update
```

---

## Install & Build (End-to-End)

1. Create workspace:
```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
git clone https://github.com/abizer007/adaptive-user-responsive-automaton_ROS2-Humble.git
cd ~/ws_moveit2
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ws_moveit2/install/setup.bash
```

---

## Run / Demo Flow (Phases 1–4)

### Phase 1 — Launch MoveIt demos
**Terminal 1**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
ros2 launch moveit2_tutorials mtc_demo.launch.py
```

**Terminal 2**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
ros2 launch moveit2_tutorials pick_place_demo.launch.py
```

### Phase 2 — Start server & invoke a service
**Terminal 3**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
ros2 run panda_keyboard_control moveit_service_action
```

**Terminal 4**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
ros2 service list | grep -E "moveit_command|move_command" || true
ros2 service call /moveit_command panda_interfaces/srv/MoveCommand "{command: 'home'}"
```

### Phase 3 — Send an action goal
**Terminal 5**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
ros2 action list | grep pick || true
ros2 action send_goal /pick_place panda_interfaces/action/PickPlace "{object_name: 'cube'}" --feedback
```

### Phase 4 — Keyboard control & command echo
**Terminal 6**
```bash
source ~/ws_moveit2/install/setup.bash
ros2 run panda_keyboard_control keyboard_control
```

**Terminal 7**
```bash
source ~/ws_moveit2/install/setup.bash
ros2 topic echo /panda_command
```

---

## Service & Action API

### MoveCommand.srv
```
string command
---
bool success
string message
```

### PickPlace.action
```
string object_name
---
bool success
string message
---
float32 progress_percentage
string state
```

---

## How to Use — Examples

**Send a move command:**
```bash
ros2 service call /moveit_command panda_interfaces/srv/MoveCommand "{command: 'home'}"
```

**Start a pick-and-place action:**
```bash
ros2 action send_goal /pick_place panda_interfaces/action/PickPlace "{object_name: 'cube'}" --feedback
```

**Manual keyboard control:**
```bash
ros2 run panda_keyboard_control keyboard_control
```

Monitor commands:
```bash
ros2 topic echo /panda_command
```

---

## 🎥 Project Demo
Click to view video

[![AURA Demo Preview](Screenshot%202025-11-09%20174402.png)](E034_MDRIA%20Project%20%281%29.mp4)


---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Service or action not found | Ensure `moveit_service_action` is running & sourced workspace |
| RViz not showing | Enable 3D acceleration in VMware / use host GPU |
| `colcon build` errors | Run `rosdep install --from-paths src --ignore-src -r -y` then rebuild |
| Python permission error | `chmod +x` Python scripts & verify shebang `#!/usr/bin/env python3` |

---

## Contributing

1. Fork the repo  
2. Create a feature branch: `git checkout -b feature/my-improvement`  
3. Make changes & test: `colcon build`  
4. Open a Pull Request with description

---

## License

This project is licensed under the **Apache 2.0**.

---

## Acknowledgements

- ROS 2 and MoveIt 2 open-source projects  
- Real-world MSME use-case inspiration (Ulhasnagar factory)  
- Mentors, peers, and open-source community support

---
