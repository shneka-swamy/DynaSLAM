#!/bin/bash
#

# Usage: ./scripts/run-docker.sh <command>

# print usage on -h or --help

if [[ $1 == "-h" || $1 == "--help" ]]; then
  echo "Usage: ./scripts/run-docker.sh <command>"
  exit 0
fi

# if hostname is tesla then datasetFolder is /media/extra/slam_net/datasets/TUM/
# else datasetFolder is /media/scratch/
if [[ $(hostname) == "tesla" ]]; then
  datasetFolder=/media/extra/slam_net/datasets/TUM/
else
  datasetFolder=/media/scratch/
fi

DOCKER_VOLUME="
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${PWD}":"/dynaslam":rw \
  --volume="${datasetFolder}":"/media/scratch":rw \
"

DOCKER_ENV_VARS="
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
"

DOCKER_ARGS=${DOCKER_VOLUME}" "${DOCKER_ENV_VARS}

image=dynaslam:latest

if [ $# -eq 0 ]; then
  echo "No command provided. Running bash."
  sudo docker run -it --net=host --ipc=host ${DOCKER_ARGS} ${image} bash
  exit 0
else
  echo "Running command: $1"
  sudo docker run -it --net=host --ipc=host ${DOCKER_ARGS} ${image} bash -c "$1"
  exit 0
fi

