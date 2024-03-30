#! /usr/bin/env python3

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

'''
This script outputs the total number of cycles spent in NPU execution. We make
several assumptions:

1. NPU execution is not pipelined. I.e., two consecutive invocations of NPU are
executed completely serially. This is a pessimistic assumption.

2. There's no NPU flush. This is somewhat optimistic but NPU flushes (happen on
branch mispredictions) are extremely rare in our workloads. The branch miss
ratio in all of the workloads is < 1%.

3. Every individual MAC operation consumes 8 cycles and every CPU-NPU
communication consumes 4 cycles. The scheduling latency is 5 cycles, and the
LUT lookup consumes 1 cycle. These numbers are reasonably close to those in
<https://www.cs.utexas.edu/~cart/publications/dissertations/hadi.pdf>.
'''

def getNpuCycles(robotName):
    if robotName == "PatrolBot": return 145167506
    elif robotName == "HomeBot": return 41387820
    elif robotName == "FlyBot": return 242968478
    else: return 0

