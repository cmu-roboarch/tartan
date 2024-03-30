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

BINARY="patrolbot_tartan.out"
LABELS_FILE="./mobilenet-data/object_detection_classes_coco.txt"
MODEL_FILE="./mobilenet-data/frozen_inference_graph.pb"
CFG_FILE="./mobilenet-data/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt"
PATH_FILE="./path-logs/path1.txt"
IMAGES_DIR="./cmu-tour"
SENSOR_LOG_FILE="./sensor-logs/data.txt"


if [[ ! -f $BINARY || ! -d $IMAGES_DIR || ! -f $LABELS_FILE || ! -f $MODEL_FILE || ! -f $CFG_FILE ]]; then
    echo "Missing required files!"
    echo -n "Binary: $BINARY ? " && ( [ -f $BINARY ] || echo -n "Doesn't exist" ) && echo ""
    echo -n "Input dir data/images: $IMAGES_DIR ?" && ( [ -d $IMAGES_DIR ] || echo -n "Doesn't exist" ) && echo ""
    echo -n "Input file data/labels: $LABELS_FILE ?" && ( [ -f $LABELS_FILE ] || echo -n "Doesn't exist" ) && echo ""
    echo -n "Input file data/model: $MODEL_FILE ?" && ( [ -f $MODEL_FILE ] || echo -n "Doesn't exist" ) && echo ""
    echo -n "Input file data/cfg: $CFG_FILE ?" && ( [ -f $CFG_FILE ] || echo -n "Doesn't exist" ) && echo ""
    exit 1
fi


echo "$(readlink -f $BINARY) --cfg=$(readlink -f $CFG_FILE) --path=$(readlink -f $PATH_FILE) --imgdir=$(readlink -f $IMAGES_DIR) --labels=$(readlink -f $LABELS_FILE) --model=$(readlink -f $MODEL_FILE) --scale=0.001 --confidence=0.4 --log=$(readlink -f $SENSOR_LOG_FILE)"
