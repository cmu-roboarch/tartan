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

INPUTSET_FILE="tartan-apps-input.tar.gz"

if [[ ! -f "$INPUTSET_FILE" ]]; then
    echo "Inputset file doesn't exist: $INPUTSET_FILE"
    exit 1
fi

INPUTSET_DIR=${INPUTSET_FILE%%.*}
mkdir "$INPUTSET_DIR"
tar xf "$INPUTSET_FILE" -C "$INPUTSET_DIR"

for robot in Carri Deli Fly Home Move Patrol
do
    mv "${INPUTSET_DIR}/${robot}Bot/"* "${robot}Bot/"
done

rm -rf "$INPUTSET_DIR"
