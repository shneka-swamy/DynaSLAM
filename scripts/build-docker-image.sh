#!/bin/bash
# This script builds the docker image for the application.

if ! command -V docker > /dev/null 2>&1; then
    echo "Please install docker first!"
    echo "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-docker"
    exit 1
fi

# Check if the current user can run the hello-world image
if docker run hello-world >/dev/null 2>&1; then
    echo "Docker is installed and the user can run the hello-world image."
else
    echo "Docker is installed, but the user cannot run the hello-world image. Please check your Docker permissions."
    echo "https://phoenixnap.com/kb/docker-permission-denied#ftoc-heading-4"
fi

if docker run --rm --runtime=nvidia --gpus all nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04 nvidia-smi >/dev/null 2>&1; then
  echo "Nvidia docker is installed and the user can run the nvidia-smi command."
else
  echo "nvidia docker might need to be installed. Please check your nvidia docker permissions."
  echo "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit"
fi

sudo docker build -t dynaslam:latest ./dockerfiles/dynaslam/

