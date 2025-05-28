###############################################################################
#  Dockerfile – Ubuntu 22.04  +  CUDA 11.3  +  cuDNN 8.6  +  TensorRT 8.5  +
#                GCC-9  +  ROS 2 Humble  +  Protobuf 3.6.1
###############################################################################
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
# Use bash for the following RUN instructions
SHELL ["/bin/bash", "-c"]


###############################################################################
# 1. Core build tools + GCC-9
###############################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git wget curl unzip pkg-config software-properties-common \
    gcc-9 g++-9 cmake ninja-build \
    python3.10 python3.10-venv python3-pip \
 && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 \
 && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100 \
 && rm -rf /var/lib/apt/lists/*

ENV CC=/usr/bin/gcc-9
ENV CXX=/usr/bin/g++-9

###############################################################################
# 2. Install CUDA 11.3 toolkit from .run file
###############################################################################
COPY cuda_11.8.0_520.61.05_linux.run /tmp/
RUN chmod +x /tmp/cuda_11.8.0_520.61.05_linux.run && \
    /tmp/cuda_11.8.0_520.61.05_linux.run --silent --toolkit --override && \
    rm /tmp/cuda_11.8.0_520.61.05_linux.run
ENV CUDA_HOME=/usr/local/cuda-11.8
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:/opt/TensorRT/lib:/opt/protobuf-3.6.1/lib

###############################################################################
# 3. cuDNN 8.6.0 for CUDA 11.x
###############################################################################
COPY cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.xz /tmp/
RUN tar -xf /tmp/cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.xz -C /tmp && \
    cp /tmp/cudnn-linux-x86_64-8.6.0.163_cuda11-archive/include/* ${CUDA_HOME}/include/ && \
    cp /tmp/cudnn-linux-x86_64-8.6.0.163_cuda11-archive/lib/* ${CUDA_HOME}/lib64/ && \
    chmod a+r ${CUDA_HOME}/include/* ${CUDA_HOME}/lib64/* && \
    rm -rf /tmp/cudnn-linux-x86_64-8.6.0.163_cuda11-archive*

###############################################################################
# 4. TensorRT 8.5.x  (your tarball name on host)
###############################################################################
COPY TensorRT-8.5.3.1.Linux.x86_64-gnu.cuda-11.8.cudnn8.6.tar.gz /opt/
RUN cd /opt && \
    tar -xzf TensorRT-8.5.3.1.Linux.x86_64-gnu.cuda-11.8.cudnn8.6.tar.gz && \
    rm TensorRT-8.5.3.1.Linux.x86_64-gnu.cuda-11.8.cudnn8.6.tar.gz && \
    mkdir -p /opt/TensorRT && \
    mv TensorRT-8.5.*/* /opt/TensorRT
ENV TENSORRT_ROOT=/opt/TensorRT
ENV LD_LIBRARY_PATH=${TENSORRT_ROOT}/lib:${LD_LIBRARY_PATH}
RUN pip install ${TENSORRT_ROOT}/python/tensorrt-*-cp310-*.whl


###############################################################################
# 5. ROS 2 Humble
###############################################################################
# Add the correct keyring first
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor -o /etc/apt/keyrings/ubuntu.gpg && \
    echo "deb [trusted=yes] http://archive.ubuntu.com/ubuntu jammy main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb [trusted=yes] http://archive.ubuntu.com/ubuntu jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb [trusted=yes] http://archive.ubuntu.com/ubuntu jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb [trusted=yes] http://security.ubuntu.com/ubuntu jammy-security main restricted universe multiverse" >> /etc/apt/sources.list && \
    apt-get update && apt-get install -y \
    software-properties-common \
    gnupg lsb-release curl



# Add ROS 2 key (dearmor to keyring file, preferred over apt-key)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
 > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble desktop
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rosdep init && rosdep update


###############################################################################
# 6. Extra dev libraries (OpenCV, PCL, etc.)
###############################################################################
RUN apt-get update && apt-get install -y libopencv-dev libpcl-dev && \
    rm -rf /var/lib/apt/lists/*

###############################################################################
# 7. Protobuf 3.6.1  (build from source)
###############################################################################
RUN wget -qO- https://github.com/protocolbuffers/protobuf/archive/v3.6.1.tar.gz | \
    tar -xz -C /tmp && \
    cd /tmp/protobuf-3.6.1 && \
    ./autogen.sh && ./configure --prefix=/opt/protobuf-3.6.1 && \
    make -j$(nproc) && make install && ldconfig && \
    cd / && rm -rf /tmp/protobuf-3.6.1
ENV PROTOBUF_ROOT=/opt/protobuf-3.6.1
ENV LD_LIBRARY_PATH=${PROTOBUF_ROOT}/lib:${LD_LIBRARY_PATH}

###############################################################################
# 8. Python ML stack (+ cu113 wheels)
###############################################################################
# remove Ubuntu’s blinker pkg so pip can own it
#RUN apt-get purge -y python3-blinker

#RUN pip install --upgrade pip && \
    #pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 \
        #-f https://download.pytorch.org/whl/torch_stable.html && \
    #pip install onnx onnxsim mmengine "mmcv==2.0.0rc4" \
    #    mmdet==2.25.0 \
        #git+https://github.com/open-mmlab/mmdetection3d.git@v1.1.0 && \
    #pip install --extra-index-url https://pypi.nvidia.com \
        #"pytorch-quantization==2.1.2+cu116"


###############################################################################
# 9. Final settings
###############################################################################
WORKDIR /workspace
CMD ["/bin/bash"]

