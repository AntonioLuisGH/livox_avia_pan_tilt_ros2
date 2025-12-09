# Livox Avia ROS2 Workspace (Simple Pan/Tilt)

This workspace contains the drivers and tools required to operate the **Livox Avia** LiDAR in **ROS2 Humble**, featuring a custom TF broadcaster that stabilizes the PointCloud using the internal IMU data (Pan/Tilt correction).

## Packages Included

| Package | Description |
| :--- | :--- |
| **`livox_ros2_avia`** | The ROS2 driver for Livox Avia (ASIG-X fork). |
| **`imu_tools`** | Contains the **Madgwick Filter** to fuse raw IMU data into a stable orientation. |
| **`simple_pan_tilt`** | Custom node that broadcasts a dynamic TF (`world` -> `livox_frame`) based on the filtered IMU orientation. |

---

## Prerequisites

* **OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **ROS2:** Humble Hawksbill
* **Hardware:** Livox Avia LiDAR + Livox Converter 2.0

### 1. Install Livox SDK v1 (Required)
The Avia driver requires the original Livox SDK (v1), **not** SDK2.

```bash
sudo apt install cmake build-essential
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
mkdir build && cd build
cmake ..
make
sudo make install
```

### 2. Critical Fix: Resolve Library Conflict
The Livox SDK installs a static library that conflicts with the ROS2 driver build. You must rename or remove it:

```bash
sudo mv /usr/local/lib/liblivox_sdk_static.a /usr/local/lib/liblivox_sdk_static.a.bak
```

---

## Installation

Clone this repository (or your workspace):

```bash
git clone <YOUR_REPO_URL> ~/livox_avia_ws
cd ~/livox_avia_ws
```

Install Dependencies:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

Build the Workspace:
> **Note:** We avoid `--symlink-install` to prevent specific Python environment issues with this setup.

```bash
colcon build
```

Source the setup file:

```bash
source install/setup.bash
```

---

## Usage

We have created a single launch file that starts the entire pipeline:

1.  **Driver:** Connects to the Avia and publishes raw points & IMU.
2.  **Filter:** Fuses IMU data to calculate orientation (Quaternion).
3.  **Broadcaster:** Publishes the TF so the LiDAR moves in 3D space.
4.  **RViz:** Visualizes the result.

### Run the System

```bash
ros2 launch simple_pan_tilt pan_tilt.launch.py 
```

---

## Configuration

*   **LiDAR Config:** `src/livox_ros2_avia/config/livox_lidar_config.json`
    *   Ensure your **Broadcast Code** (Serial Number + "1") is set here.

*   **IMU Gain:** Found in `src/simple_pan_tilt/launch/pan_tilt.launch.py`
    *   **Default:** `0.1`
    *   Lower this value if the horizon drifts too much; raise it if the response is too slow.

---

## Troubleshooting

### "The build fails with error: option --editable not recognized"
This happens if the `setup.py` in `simple_pan_tilt` is malformed. Ensure you are not using `--symlink-install` if you recently edited the python files, or clean the build artifacts:

```bash
rm -rf build install log
colcon build
```

### "I don't see any points in RViz"
1.  Check your network settings (IP `192.168.1.5`, Subnet `255.255.255.0`).
2.  Verify the **Broadcast Code** in `livox_lidar_config.json` matches your device.
3.  Ensure the firewall is disabled or allowing traffic on ports `55000` & `65000`.