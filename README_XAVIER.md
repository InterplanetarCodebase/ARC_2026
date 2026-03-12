## Running ROS2 on Jetson Xavier via Docker

The Jetson Xavier used on the rover does not natively support the required **ROS2 Humble environment**.
Instead, ROS2 runs inside a **Docker container** that provides a full Ubuntu 22.04 + ROS2 runtime.

The container exposes:

* USB devices for Arduino and ESP32
* host networking for DDS discovery
* the local ROS workspace as a mounted volume
* optional GUI access for debugging tools

This allows the rover software to run on Jetson while maintaining compatibility with the ROS2 ecosystem.

---

## Docker Setup (Jetson Xavier Only)

Install Docker:

```bash
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER
```

Log out and back in after adding the user to the docker group.

Verify installation:

```bash
docker run hello-world
```

---

## Workspace Layout on Jetson

The ROS2 workspace lives on the Jetson host and is mounted into the container.

```
~/ros2_ws
├── Dockerfile
├── build/
├── install/
├── log/
└── src/
    └── ARC_2026
```

Clone the repository:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone -b ros2 https://github.com/InterplanetarCodebase/ARC_2026.git
```

---

## Building the Docker Image

From the workspace root:

```bash
cd ~/ros2_ws
docker build -t interplanetar_ros2 .
```

This builds a ROS2 environment containing:

* ROS2 Humble base
* CycloneDDS middleware
* Python dependencies (`pyserial`, `pygame`)
* development tools

---

## Running the Rover Container

Start the container with full hardware and networking access:

```bash
docker run -it \
--network host \
--ipc host \
--privileged \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/ros2_ws:/workspace \
-v /dev:/dev \
--name rover_ros2 \
interplanetar_ros2
```

### Explanation of important flags

| Option                         | Purpose                              |
| ------------------------------ | ------------------------------------ |
| `--network host`               | Required for DDS discovery           |
| `--ipc host`                   | Enables ROS2 shared memory transport |
| `--privileged`                 | Allows hardware access               |
| `-v /dev:/dev`                 | Exposes USB devices (Arduino/ESP32)  |
| `-v ~/ros2_ws:/workspace`      | Mounts ROS2 workspace                |
| `DISPLAY` and `/tmp/.X11-unix` | Enables GUI applications             |

---

## Allowing GUI Applications from Docker

If running graphical applications inside the container for debugging:

Run this on the **Jetson host before starting the container**:

```bash
xhost +local:docker
```

This allows Docker containers to access the X display server.

---

## Building the ROS2 Package Inside the Container

Inside the running container:

```bash
cd /workspace

source /opt/ros/humble/setup.bash

colcon build --packages-select interplanetar_rover --symlink-install
```

Then load the workspace:

```bash
source install/setup.bash
```

---

## Launching the Rover Nodes

Inside the container:

```bash
ros2 launch interplanetar_rover rover.launch.py \
  wheel_port:=/dev/ttyUSB0 \
  arm_port:=/dev/ttyUSB1
```

These devices correspond to:

* `/dev/ttyUSB0` → Arduino Nano wheel controller
* `/dev/ttyUSB1` → ESP32 arm controller

---

## Updating Code on the Rover

Because the workspace is mounted from the host, updates are simple.

Pull the latest code on the Jetson host:

```bash
cd ~/ros2_ws/src/ARC_2026
git pull origin ros2
```

Restart the ROS launch process inside the container.

Python changes take effect immediately because the workspace is built with:

```
--symlink-install
```

No rebuild is required for normal code edits.

---

## Serial Device Detection

Check available serial devices:

```bash
ls /dev/ttyUSB*
```

You can also inspect kernel logs:

```bash
dmesg | tail
```

The container automatically sees devices because `/dev` is mounted.

---

## Debugging ROS2 Inside Docker

Useful commands:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

Verify DDS configuration:

```bash
ros2 doctor --report | grep cyclone
echo $CYCLONEDDS_URI
```

---

## Recommended Terminal Workflow

Use multiple host terminals to interact with the container.

### Terminal 1 — Start container

```bash
docker start -ai rover_ros2
```

### Terminal 2 — Shell inside container

```bash
docker exec -it rover_ros2 bash
```

### Terminal 3 — ROS debugging

```bash
docker exec -it rover_ros2 bash
```

This allows simultaneous monitoring of logs, shell commands, and ROS topics.

---

## Development Cycle

Typical workflow during development:

On development machine:

```bash
git add -A
git commit -m "fix: rover control bug"
git push origin ros2
```

On Jetson rover:

```bash
cd ~/ros2_ws/src/ARC_2026
git pull
```

Restart the ROS launch.

---

## Resulting System Architecture

```
Base Station Laptop
  ROS2 Humble
  gui_node.py (pygame teleop)

           ↓ CycloneDDS over WiFi

Jetson Xavier (Rover)
  Docker container
      ROS2 Humble
      wheel_bridge_node.py
      arm_bridge_node.py
           │
           ├── Arduino Nano
           └── ESP32
```

