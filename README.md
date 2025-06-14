# BEVFusion with ROS2 - Docker Setup
This repository provides a comprehensive, Docker-based setup of [BEVFusion-ROS-TensorRT](https://github.com/linClubs/BEVFusion-ROS-TensorRT/tree/humble_devel), to run inside a container environment with **CUDA 11.8**, **cuDNN 8.6.0**, and **TensorRT 8.5**.


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

# Allow Docker container to access host X server
xhost +local:root

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

Use provided modified versions:

* **CMakeLists.txt**: Updated to include the paths for CUDA, TensorRT and Protobuf
* **tool/environment.sh**: Updated to include the environment variables for CUDA, cuDNN, and TensorRT and sets `DEBUG_MODEL` (e.g., `resnet50`/`resnet50int8`/`swint`) and `DEBUG_PRECISION` (e.g., `fp16`/`int8`) to match `bevfusion.launch.py`. Do not use `int8` with `swint`.  

Ensure consistency between `environment.sh` and your launch file.

## Build BEVFusion

```bash
source src/BEVFusion-ROS-TensorRT/tool/environment.sh
colcon build --symlink-install
```

## Create TensorRT Engines

```bash
cd src/BEVFusion-ROS-TensorRT
./tool/build_trt_engine.sh swint fp16
```

## Launch BEVFusion

```bash
cd ~/bevfusion_ws
source install/setup.bash
ros2 launch bevfusion bevfusion.launch.py
```

### Enable RViz 

```bash
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR
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
