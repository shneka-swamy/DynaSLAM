FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

RUN apt-get update \
  && apt-get install -y locales lsb-release gpg-agent curl vim tmux wget git build-essential cmake g++ ninja-build make

ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LANG=en_US.UTF-8

RUN apt-get install -y libssl-dev libeigen3-dev libboost-all-dev

RUN mkdir /cmake && cd /cmake \
    && wget https://github.com/Kitware/CMake/releases/download/v3.26.4/cmake-3.26.4.tar.gz \
    && tar zxvf cmake-3.26.4.tar.gz --no-same-owner \
    && cd cmake-3.26.4 \
    && ./bootstrap --parallel=10 \
    && make -j 10 \
    && make install

RUN wget -O opencv-3.4.12.tar.gz https://github.com/opencv/opencv/archive/refs/tags/3.4.12.tar.gz \
 && wget -O opencv_contrib-3.4.12.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/3.4.12.tar.gz \
 && tar zxvf opencv-3.4.12.tar.gz --no-same-owner \
 && tar zxvf opencv_contrib-3.4.12.tar.gz --no-same-owner \
 && mkdir opencv-build && cd opencv-build \
 && cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_EXTRA_MODULES_PATH=/opencv_contrib-3.4.12/modules -DOPENCV_GENERATE_PKGCONFIG=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DWITH_CUDA=ON -DWITH_CUBLAS=1 -DENABLE_FAST_MATH=1 -DCUDA_FAST_MATH=1 ../opencv-3.4.12 \
  && cmake --build . --config Release --target install && cd ..

RUN apt-get install -y libgl1-mesa-dev libglew-dev

RUN mkdir /pangolin && cd /pangolin \
    && wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.tar.gz \
    && tar zxvf v0.6.tar.gz --no-same-owner \
    && cd Pangolin-0.6 \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja .. \
    && cmake --build . --config Release --target install && cd ..

RUN apt-get install -y qt5-default python-pip python-virtualenv htop python-tk \
    && pip2 install numpy==1.16.6 scikit-image==0.14.5 scipy==1.2.3 matplotlib==2.2.5 \
         cython==0.29.35 tensorflow==1.12.3 protobuf==3.13.0 keras==2.0.8 

SHELL ["/bin/bash", "-l", "-c"]
RUN pip2 install pycocotools==2.0.0

RUN mkdir /dynaslam && mkdir -p /media/scratch/

WORKDIR /dynaslam

