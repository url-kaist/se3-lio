# Python-only image — build & run the se3_lio Python package + CLI, no ROS.
# Compiles the ROS-agnostic C++ core into the pybind11 wheel and installs the
# optional Polyscope viewer ([viz]). To show the viewer, run with X11 forwarding
# and --gpus all (see run_docker.sh).
#
# Build from the REPO ROOT (context needs cpp/ and python/):
#   docker build -f docker/python/Dockerfile -t se3_lio:py .
#
# For a CPU-only / headless image, swap the base for ubuntu:22.04 (software GL).
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Toolchain to compile the core (OpenMP via libgomp) + GL/X11 runtime for Polyscope.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3 python3-pip python3-dev \
      build-essential cmake git ninja-build libgomp1 \
      libgl1 libglu1-mesa libxext6 libxrender1 \
      libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir scikit-build-core pybind11 ninja

# python/CMakeLists.txt compiles ../cpp/se3_lio into the wheel.
WORKDIR /ws/src/SE3-LIO
COPY cpp/    ./cpp/
COPY python/ ./python/
RUN rm -rf python/build \
    && pip install --no-cache-dir --no-build-isolation "./python/[viz]"

WORKDIR /ws
CMD ["bash"]
