# BEVFusion with ROS2 - Docker Setup
This repository provides a comprehensive step-by-step setup for running BEVFusion-ROS-TensorRT inside a Docker container configured with CUDA 11.8, cuDNN 8.6.0, and TensorRT 8.5.

## Prerequisites

Download the following into your working folder (`~/docker`):

* **CUDA 11.8** [Runfile](https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=runfile_local)
* **cuDNN 8.6.0** from [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)
* **TensorRT 8.5.3.1** from [NVIDIA TensorRT](https://developer.nvidia.com/nvidia-tensorrt-8x-download)
* **protobuf v3.6.1** [protobuf archive](https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.6.1.tar.gz)

Ensure your folder structure is as follows:

```
docker/
├── cuda_11.8.0.run
├── cudnn-linux-x86_64-8.6.0.tgz
├── TensorRT-8.5.3.1.tar.gz
├── Dockerfile
```

## Setup ROS Workspace (on Host Machine)

```bash
mkdir -p ~/bevfusion_ws/src
cd ~/bevfusion_ws/src

# Clone repository
git clone https://github.com/linClubs/BEVFusion-ROS-TensorRT.git
cd BEVFusion-ROS-TensorRT
git checkout humble-devel
```
Alternatively, you can download the full folder and place it inside ```bevfusion_ws/src/```, then rename it to ```BEVFusion-ROS-TensorRT ```. This saves you from manually cloning and checking out the branch.

## Docker Build and Run

### Build Docker Image

```bash
docker build -t bevfusion:cu118-trt85 .
```

### Run Docker Container

```bash
docker run --gpus all -it \
  --name bevfusion_ros2 \
  -v ~/bevfusion_ws:/workspace/bevfusion_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all,graphics \
  --network host \
  bevfusion:cu118-trt85
```

### Manage Container

* **Start again**:

```bash
docker start -ai bevfusion_ros2
```

* **Exit Container**:

```bash
exit
```

* **Stop Container**:

```bash
docker stop bevfusion_ros2
```

## Models and Example Data

* **Download from**: [BEVFusion-ROS2 data](https://universityoflincoln-my.sharepoint.com/:f:/r/personal/26619055_students_lincoln_ac_uk/Documents/PhD%20-%20Prabuddhi%20-%202025/Reports/5%20-%20May/BEVFusion%20-%20ROS2?csf=1&web=1&e=6Mny1I)

Place downloaded data as follows:

```
bevfusion_ws/
├── src/
│   └── BEVFusion-ROS-TensorRT/
│       ├── models/
│       └── example-data/
```

## Inside Docker: Setup Workspace

```bash
# Source ROS
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ROS dependencies
cd /workspace/bevfusion_ws
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## Changes in Files

Use the provided modified versions of key files to ensure correct toolkit paths and runtime configurations:

- **`CMakeLists.txt`**  
  Updated to include paths for:
  - **CUDA 11.8**
  - **cuDNN 8.6.0**
  - **TensorRT 8.5**
  - **Protobuf 3.6.1**

  These changes ensure the build system correctly locates required headers and libraries.

- **`environment.sh`**  
  This is a comprehensive and idempotent environment setup script designed for BEVFusion with ROS 2 and TensorRT. It includes:

  1. **Path Configuration**
     - Sets environment variables for CUDA, cuDNN, and TensorRT.
     - Sets `DEBUG_MODEL` (e.g., `resnet50`) and `DEBUG_PRECISION` (e.g., `fp16`) to match `bevfusion.launch.py`.

  2. **Validation**
     - Verifies that required executables like `trtexec` and `nvcc` exist.

  3. **Python Support (optional)**
     - Dynamically sets `Python_Inc`, `Python_Lib`, and `Python_Soname` if Python support is enabled.

  4. **Path Deduplication**
     - Appends all paths (`PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`) only if not already included.

  5. **GPU Architecture Detection**
     - Automatically sources `tool/cudasm.sh` to detect and export the current GPU’s compute capability (`CUDASM`).

  6. **Idempotency**
     - Prevents re-sourcing the script if it has already been sourced during the session.

> 🆚 **Difference from minimal setup**  
> Unlike a basic configuration like:
> ```bash
> export DEBUG_MODEL=resnet50
> export DEBUG_PRECISION=fp16
> ```
> which only sets two variables, this full `environment.sh` script ensures the entire runtime and build environment is fully configured and ready for BEVFusion execution.



## Changes in Files

Use provided modified versions:

* **CMakeLists.txt**: Updated to include the paths for CUDA, TensorRT and Protobuf
* **tool/environment.sh**: Updated to include the environment variables for CUDA, cuDNN, and TensorRT and sets `DEBUG_MODEL` (e.g., `resnet50`) and `DEBUG_PRECISION` (e.g., `fp16`) to match `bevfusion.launch.py`.

Ensure consistency between `environment.sh` and your launch file.

## Build BEVFusion

```bash
source src/BEVFusion-ROS-TensorRT/tool/environment.sh
colcon build --symlink-install
```

## Create TensorRT Engines

```bash
cd src/BEVFusion-ROS-TensorRT
./tool/build_trt_engine.sh resnet50 fp16
```

## Launch BEVFusion

```bash
cd ~/bevfusion_ws
source install/setup.bash
ros2 launch bevfusion bevfusion.launch.py
```

### Enable RViz 

```bash
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb
```

### Launch RViz

```bash
rviz2 -d src/BEVFusion-ROS-TensorRT/launch/view.rviz
```

## Testing with ROS2 Bag

Test the setup with a ROS2 bag generated using:

* **[nuscenes2rosbag repo](https://github.com/Prabuddhi-05/nuscenes2rosbag)**

## Maintenance

* **Clean previous builds**:

```bash
rm -rf build/ install/ log/
```
