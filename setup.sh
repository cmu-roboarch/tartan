#!/bin/bash

# MIT License
#
# Copyright (c) 2024 Carnegie Mellon University
#
# This file is part of Tartan.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Set this if you have storage somewhere else. Otherwise, leave it commented
# out and the current location will be used for storing large files
# DISK_PATH="/n/PrescriptiveMemBenchmarks/tartan-large-files"

## Flag files
LINUX_PROGRAMS_FLAG_FILE="./.linux_programs.flag"
PYTHON_MODULES_FLAG_FILE="./.python_modules.flag"
OPENCV_FLAG_FILE="./.opencv.flag"
APP_INPUS_FLAG_FILE="apps/.apps_set_up.flag"

assert_file_exists() {
    local file_path="$1"
    if [ ! -f "$file_path" ]; then
        echo "File ${file_path} does not exist. file=${BASH_SOURCE[0]}, line=$LINENO, function=${FUNCNAME[1]}, caller=${FUNCNAME[0]}"
        exit 1
    fi
}

assert_dir_exists() {
    local dir_path="$1"
    if [ ! -d "$dir_path" ]; then
        echo "Directory ${dir_path} does not exist. file=${BASH_SOURCE[0]}, line=$LINENO, function=${FUNCNAME[1]}, caller=${FUNCNAME[0]}"
        exit 1
    fi
}

original_dir=$(pwd)

echo "Installing Linux programs..."
if [[ ! -f "$LINUX_PROGRAMS_FLAG_FILE" ]]; then
    sudo apt-get -y update
    sudo apt-get -y upgrade

    packages=(
        build-essential autoconf automake libbz2-dev
        liblzma-dev libcurl4-gnutls-dev libssl-dev cmake git
        libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev
        libswscale-dev libopencv-dev libtbb-dev hyperfine jq
        parallel cppcheck coinor-libipopt-dev libgtk-3-dev
        libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev
        libpng-dev libtiff-dev gfortran openexr
        libatlas-base-dev python3-dev libtbb2 libdc1394-22-dev
        git-lfs clang-format scons libconfig-dev libconfig++-dev
        libhdf5-dev libelf-dev
    )

    sudo apt-get install -y "${packages[@]}"
    touch "$LINUX_PROGRAMS_FLAG_FILE"
else
    echo "Linux programs are already installed"
fi

echo '--------------------------------------------------------------------------------'

echo "Installing Python modules..."
if [[ ! -f "$PYTHON_MODULES_FLAG_FILE" ]]; then
    python_modules=(numpy pandas pybind11 columnar)

    if command -v pip3 &>/dev/null; then
        echo "pip is already installed"
    else
        echo "pip is not installed on this system. Installing..."
        curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
        python3 get-pip.py
    fi

    pip3 install "${python_modules[@]}"
    touch "$PYTHON_MODULES_FLAG_FILE"
else
    echo "Python modules are already installed"
fi

echo '--------------------------------------------------------------------------------'

echo "Installing OpenCV..."

if [[ ! -f "$OPENCV_FLAG_FILE" ]]; then
    mkdir -p opencv_build

    if [[ -n "$DISK_PATH" ]]; then
        mv opencv_build "${DISK_PATH}/"
        ln -sfn "${DISK_PATH}/opencv_build" .
    fi

    cd "opencv_build"
    git clone https://github.com/opencv/opencv.git
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv && mkdir -p build && cd build
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_C_EXAMPLES=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D BUILD_EXAMPLES=ON ..
    make -j"$(nproc)"
    sudo chown -R ${USER}:${USER} "${DISK_PATH}/opencv_build"
    sudo make install

    cd "$original_dir"

    touch "$OPENCV_FLAG_FILE"
else
    echo "OpenCV is already installed"
fi

echo '--------------------------------------------------------------------------------'

echo "Installing Intel Pin..."
if [[ -z "$PINPATH" ]]; then
    pin_url="https://software.intel.com/sites/landingpage/pintool/downloads/pin-2.14-71313-gcc.4.4.7-linux.tar.gz"
    pin_dir="${pin_url##*/}"
    pin_dir="${pin_dir%%.tar.gz}"

    curl -L "$pin_url" -o pin.tar.gz
    tar -xzf pin.tar.gz
    rm pin.tar.gz
    if [[ ! -d "$pin_dir" ]]; then
        echo "$pin_dir is not created!"
        exit 1
    fi

    echo "export PINPATH=$(readlink -f $pin_dir)" >>$HOME/.bashrc
    echo "Run the following command to apply changes across all sessions."
    echo "source ~/.bashrc"
else
    echo "Pin is already installed. PINPATH=$PINPATH"
fi

echo '--------------------------------------------------------------------------------'

echo "Setting up applications' inputsets..."
if [[ ! -f "$APP_INPUS_FLAG_FILE" ]]; then
    echo "Setting up apps/"
    cd "$original_dir"
    cd "apps/"
    ./prep_inputs.sh
    cd "$original_dir"
    touch "$APP_INPUS_FLAG_FILE"
else
    echo "Applications are already set up"
fi

echo '--------------------------------------------------------------------------------'

echo "Setting up ZSim configuration files"
cd apps/
apps_dir=$(pwd)

./clean.sh
./build.sh || (echo "Building applications failed. Exitting" && exit 1)

for robot in Carri Deli Fly Home Move Patrol; do
    robot_dir="${robot}Bot"
    assert_dir_exists "$robot_dir"

    cd "$robot_dir"
    assert_file_exists "echo_run_baseline.sh"
    assert_file_exists "echo_run_tartan.sh"

    baseline_cmd=$(./echo_run_baseline.sh)
    tartan_cmd=$(./echo_run_tartan.sh)

    cd "${original_dir}/zsim/tests"
    assert_file_exists "baseline.cfg"
    assert_file_exists "tartan.cfg"

    robot_baseline_cfg="${robot}Bot_baseline.cfg"
    cp "baseline.cfg" "$robot_baseline_cfg"
    baseline_cmd_escaped=$(echo "$baseline_cmd" | sed 's_/_\\/_g')
    sed -i "/process0 = {/,/};/ s/command = \".*\";/command = \"$baseline_cmd_escaped\";/" "$robot_baseline_cfg"

    robot_tartan_cfg="${robot}Bot_tartan.cfg"
    cp "tartan.cfg" "$robot_tartan_cfg"
    tartan_cmd_escaped=$(echo "$tartan_cmd" | sed 's_/_\\/_g')
    sed -i "/process0 = {/,/};/ s/command = \".*\";/command = \"$tartan_cmd_escaped\";/" "$robot_tartan_cfg"

    cd "$apps_dir"
done

cd "$original_dir"
