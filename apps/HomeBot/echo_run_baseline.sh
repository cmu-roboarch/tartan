#! /usr/bin/env bash

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

BINARY="homebot_baseline.out"
INPUT_SCENE="./living-room/"
INPUT_CAMERA_FILE="./camera1.json"

if [[ ! -f $BINARY || ! -d $INPUT_SCENE || ! -f $INPUT_CAMERA_FILE ]]; then
    echo "Missing required files!"
    echo "Binary: $BINARY ?"
    echo "Input scene: $INPUT_SCENE ?"
    echo "Input camera file: $INPUT_CAMERA_FILE ?"
    exit 1
fi

echo "$(readlink -f $BINARY) --path=$(readlink -f $INPUT_SCENE) --camera=$(readlink -f $INPUT_CAMERA_FILE) --downsample=8"
