#!/bin/bash
# 

set -e # Exit on error
set -x # Print commands

cmake -S . -B cmake-docker-debug -DCMAKE_BUILD_TYPE=Debug -DDOCKER_ENVIRONMENT=ON -G Ninja
cmake -S . -B cmake-docker-release -DCMAKE_BUILD_TYPE=Release -DDOCKER_ENVIRONMENT=ON -G Ninja
