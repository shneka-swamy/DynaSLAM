#!/bin/bash

cmake --build cmake-docker-release
./cmake-docker-release/bin/mono_tum_dup Vocabulary/ORBvoc.txt Examples/Monocular/TUM3.yaml /media/scratch/rgbd_dataset_freiburg3_walking_xyz/ ./DynaSLAM/src/