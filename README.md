# 🤖 Ball Chasing Robot – ROS 2 Harmonic Project

Welcome! This repository contains my submission for the CSE412 Robotics (Fall 2025) Assignment&nbsp;2. The goal is to design, build, and operate a differential-drive robot in Gazebo Harmonic that can chase a white ball using ROS&nbsp;2 Humble. Along the way we publish the full TF tree, stream camera data, and visualise everything in RViz.  

<p align="center">
  <a href="https://www.youtube.com/watch?v=SzWEBW0BYnY">
    🎥 <strong>Watch the 5‑minute demo video on YouTube</strong>
  </a>
</p>

---

## 🌟 Highlights
- Custom URDF robot with chassis, driven wheels, caster wheel, camera, and counterweight.
- Gazebo Harmonic world containing the robot and a white ball target.
- ROS 2 bridge pipeline for `/cmd_vel`, `/odom`, `/camera/image_raw`, `/camera/camera_info`, `/tf`, and `/tf_static`.
- Python ball-chasing node that analyses the camera feed (OpenCV) and publishes velocity commands.
- RViz configuration to inspect TF frames, robot model, and live camera overlay.
- Convenience scripts for building, launching, and running the full stack.

---

## 🧱 System Architecture

```
Gazebo Harmonic ── ros_gz_bridge ── ROS 2 Humble
        │                               │
        │                             robot_state_publisher
        │                             joint_state_publisher
        │                               │
      Physics & Sensors            TF tree (world → odom → base_link → … → camera_link)
        │                               │
        └─ Camera image / Odom ──▶ BallChaserNode (OpenCV) ──▶ /cmd_vel ──▶ DiffDrive plugin
```

All ROS processes are orchestrated through `ros2 launch my_robot_bringup bringup.launch.py`, which starts:
- Gazebo with the `ball_world.sdf` environment,
- ros_gz_bridge parameter bridge instances,
- `robot_state_publisher` & `joint_state_publisher`,
- RViz with the preconfigured view,
- `tf2_tools view_frames` (optional PDF output),
- `ball_chaser_node` (Python implementation).

---

## 🗂️ Workspace Layout

| Path | Description |
|------|-------------|
| `src/my_robot_description` | URDF, meshes, and Gazebo world files. |
| `src/my_robot_bringup` | Launch files, RViz configuration, package metadata. |
| `src/ball_chaser` | Python node, package configuration, and resources. |
| `run_fixed.sh` | Helper script that applies NVIDIA/Qt env vars and launches the staggered bringup. |
| `build.sh` | Wrapper around `colcon build` with workspace setup. |

---

## ⚙️ Prerequisites

- Ubuntu 22.04 (Jammy) with ROS 2 Humble.
- Gazebo Harmonic (via `ros-gz` integration).
- NVIDIA GPU drivers (optional but recommended; handled by `run_fixed.sh`).
- `colcon` and typical ROS 2 development tools.

---

## 🚀 Setup & Build

```bash
git clone <this-repo>
cd robot
./build.sh            # runs 'colcon build' with the appropriate overlays
source install/setup.bash
```

Make sure Gazebo Harmonic resources are available (e.g., `ros-gz-sim`).  
If using NVIDIA PRIME offload, confirm that `run_fixed.sh` aligns with your GPU setup.

---

## ▶️ Launch Instructions

### One-shot launch (assignment requirement)

```bash
source /opt/ros/humble/setup.bash
cd ~/robot
source install/setup.bash
ros2 launch my_robot_bringup bringup.launch.py
```

This command starts Gazebo, RViz, bridges, TF tooling, and the ball chaser node simultaneously.

### Staggered launch with GPU-friendly settings

```bash
./run_fixed.sh
```

The script:
- Kills any lingering Gazebo/RViz processes,
- Sets up NVIDIA PRIME / Qt environment variables,
- Sources the workspace,
- Launches `bringup_staggered.launch.py`, which sequences Gazebo → bridges → nodes → RViz.

---

## 🧠 Ball Chaser Node (Python)

File: `src/ball_chaser/ball_chaser/ball_chaser_node.py`

Core pipeline:
1. Subscribe to `/camera/image_raw`.
2. Convert to OpenCV format via `CvBridge`.
3. Apply HSV masking to emphasise bright, low-saturation pixels (white ball).
4. Run a hybrid detection strategy:
   - Hough Circle Transform to capture round, centered targets.
   - Contour fallback with strict coverage and position filters to avoid floor glare.
5. Decide motion:
   - Ball left → rotate left (`angular.z > 0`).
   - Ball right → rotate right (`angular.z < 0`).
   - Ball centered → move forward (`linear.x > 0`).
   - Ball missing → pause, then rotate to re-acquire after a short timeout.
6. Publish commands on `/cmd_vel` (diff drive plugin consumes these).

Logs in the console describe each decision, making debugging easier (e.g., “Ball detected on LEFT via hough …” or “No ball detected – Searching”).

---

## 🛰️ RViz & TF Visualisation

The RViz configuration (`src/my_robot_bringup/rviz/robot_view.rviz`) shows:
- TF frames (`world`, `odom`, `base_link`, wheel links, `camera_link`).
- Robot model with visual meshes.
- Camera display overlaying the simulated image.
- Grid and TF markers for spatial awareness.

`tf2_tools view_frames` periodically exports a PDF/Graphviz snapshot to verify the tree integrity.

---

## 📹 Demonstration Video

- **YouTube (Unlisted)**: [https://www.youtube.com/watch?v=SzWEBW0BYnY](https://www.youtube.com/watch?v=SzWEBW0BYnY)
- Includes required SSF selfie intro, launch procedure, TF inspection, and live ball chasing scenario.

---

## 🛡️ Troubleshooting Tips

- **Ball not detected**: Check lighting in Gazebo. Adjust HSV bounds in `ball_chaser_node.py` if necessary.
- **Robot tips forward**: The URDF includes a counterweight link; ensure the latest URDF is built (`colcon build`).
- **RViz warnings about camera**: Relaunch after ensuring the TF bridge is running; the launch file already starts static transforms.
- **Performance issues**: Use `bringup_staggered.launch.py` (via `run_fixed.sh`) to give Gazebo a head start before RViz and the ball chaser begin.

---
