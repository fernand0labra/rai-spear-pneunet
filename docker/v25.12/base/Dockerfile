FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Set environment variable
ENV QT_QPA_PLATFORM=xcb

# Install dependencies
RUN apt-get update && \
    apt-get -y install git curl wget software-properties-common build-essential && \
    apt-get -y install libxcb-* libxkbcommon-x11-0 libx11-xcb1 libfreetype6 libfontconfig1 libdbus-1-3 && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get -y install gcc-11 g++-11 cmake cmake-qt-gui ninja-build ccache libtinyxml2-dev libboost-all-dev && \
    apt-get -y install python3.10-dev pybind11-dev python3.10-distutils && \
    curl -L https://bootstrap.pypa.io/pip/get-pip.py --output /tmp/get-pip3.py && \
    python3.10 /tmp/get-pip3.py && \
    python3.10 -m pip install --upgrade pip && \
    python3.10 -m pip install numpy==1.23.5 scipy pybind11==2.9.1 && \
    apt-get -y install libpng-dev libjpeg-dev libtiff-dev libglew-dev zlib1g-dev && \
    apt-get -y install libeigen3-dev xorg-dev libgtk-3-dev && \
    apt-get -y install qt5-qmake qtbase5-dev qtchooser qt5-qmake-bin qtbase5-dev-tools qtwayland5 && \
    apt-get -y install libcgal-dev libcgal-qt5-dev && \
    apt-get -y install liboce-ocaf-dev && \
    apt-get -y install libassimp-dev && \
    apt-get -y install nvidia-cuda-toolkit && \
    apt-get -y install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev && \
    apt-get -y install libblas-dev liblapack-dev && \
    wget https://github.com/VirtualGL/virtualgl/releases/download/3.1.2/virtualgl_3.1.2_amd64.deb && \
    apt-get -y install libxtst6 libxv1 libglu1-mesa libegl1 && \
    dpkg -i virtualgl_3.1.2_amd64.deb && \
    rm virtualgl_3.1.2_amd64.deb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create working directories
RUN mkdir -p /sofa/src && \
    mkdir -p /sofa/build/master && \
    mkdir -p /sofa/build/v25.12

# Clone Sofa Framework
RUN git clone -b v25.12 https://github.com/sofa-framework/sofa.git /sofa/src

# Copy plugin sources into container
COPY plugins /sofa/plugins

WORKDIR /sofa
