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

import os
import sys
import subprocess
import shutil
from datetime import datetime
import h5py
import numpy as np
import tkinter as tk
from scipy.stats import gmean

sys.path.append('apps/')
from npu import *

# quickRun = True
# numQuickInstructions = 15 * 1000
quickRun = False
NUM_RUNS = 10

# Set this if you have storage somewhere else. Otherwise, leave it commented
# out and the current location will be used for storing large files
# DISK_PATH = '/n/PrescriptiveMemBenchmarks/tartan-runs'

RES_DIR = "results"

def assertFileExists(filePath):
    if not os.path.isfile(filePath):
        raise FileNotFoundError(f"File {filePath} does not exist. file=__file__, line={sys._getframe().f_lineno}, function={sys._getframe().f_code.co_name}, caller={sys._getframe().f_back.f_code.co_name}")

def assertDirExists(dirPath):
    if not os.path.isdir(dirPath):
        raise NotADirectoryError(f"Directory {dirPath} does not exist. file=__file__, line={sys._getframe().f_lineno}, function={sys._getframe().f_code.co_name}, caller={sys._getframe().f_back.f_code.co_name}")

def isRunningInDocker():
    if os.path.exists('/.dockerenv'):
        return True

    try:
        with open('/proc/self/cgroup', 'rt') as f:
            if 'docker' in f.read():
                return True

    except Exception:
        pass

    return False


originalDir = os.getcwd()

# Build applications
assertDirExists("apps")
os.chdir("apps/")
process = subprocess.run(['./build.sh', '-j16'], check=True)
assert process.returncode == 0
os.chdir(originalDir)

# Build ZSim
assertDirExists("zsim")
os.chdir("zsim/")
subprocess.run(['scons', '-j16'], check=True)
zsimBinary = "build/opt/zsim"
assertFileExists(zsimBinary)
zsimBinary = os.path.realpath(zsimBinary)

# Run experiments
cfgsDir = os.path.realpath("tests/")
date = datetime.now().strftime('%Y%m%d_%H%M%S')
runDir = f"runs.{date}"
os.mkdir(runDir)
runDir = os.path.realpath(runDir)
os.chdir(runDir)

try:
    if DISK_PATH:
        os.chdir('..')
        newRunDirPath = os.path.join(DISK_PATH, os.path.basename(runDir))
        shutil.move(runDir, newRunDirPath)
        os.symlink(newRunDirPath, runDir)
        os.chdir(runDir)
except:
    pass

allRobots = ["DeliBot", "PatrolBot", "MoveBot", "HomeBot", "FlyBot", "CarriBot"]
allResults = {}  # robot --> [baseline cycles, tartan cycles]
for robot in allRobots:
    allResults[robot] = [0, 0]

for runIdx in range(NUM_RUNS):
    os.chdir(runDir)
    processes = []
    for robot in allRobots:
        for variant in ["baseline", "tartan"]:
            runName = f"{robot}_{variant}"
            os.makedirs(runName, exist_ok=True)
            os.chdir(runName)
            cfgFile = f"{cfgsDir}/{runName}.cfg"
            assertFileExists(cfgFile)
            with open(cfgFile, "r") as f: config = f.read()
            if quickRun: config = config.replace('100000000000L', f'{int(numQuickInstructions)}L')
            with open("run.cfg", "w") as f: f.write(config)
            process = subprocess.Popen([zsimBinary, "run.cfg"], stdout=open("log.txt", "w"), stderr=subprocess.STDOUT)
            processes.append(process)
            os.chdir("..")

    print("Submitted all runs. Waiting for completion...")

    for process in processes:
        process.wait()
        assert process.returncode == 0

    print(f"All runs completed successfully! runIdx={runIdx}")

    # Parsing stats
    getStatValue = lambda statPointer, parameter, beginIndex=0: np.array(statPointer[parameter][-1]) - np.array(statPointer[parameter][beginIndex])

    os.chdir(runDir)
    for robot in allRobots:
        for vIdx, variant in enumerate(["baseline", "tartan"]):
            runName = f"{robot}_{variant}"
            assertDirExists(runName)
            os.chdir(runName)
            statFile = 'zsim.h5'
            assertFileExists(statFile)
            stats = h5py.File(statFile, 'r')
            stats = stats['stats']['root']['c']
            totalCycles = max(getStatValue(stats, 'cycles'))
            if variant == "tartan": totalCycles += getNpuCycles(robot)
            allResults[robot][vIdx] += totalCycles
            os.chdir(runDir)

print(allResults)

os.makedirs(RES_DIR, exist_ok=True)

resFile = f"res_{date}.csv"
resFile = os.path.join(RES_DIR, resFile)

plotFile = f"res_{date}.png"
plotFile = os.path.join(RES_DIR, plotFile)

try:
    import matplotlib.pyplot as plt
    import pandas as pd

    # Plotting the results
    plt.style.use('seaborn-darkgrid')

    labels = list(allResults.keys())
    baselineValues = [values[0] for values in allResults.values()]
    tartanValues = [values[1] for values in allResults.values()]

    normalizedTartanValues = [baseline / tartan for baseline, tartan in zip(baselineValues, tartanValues)]
    geoMeanBaseline = 1.0
    geoMeanTartan = gmean(normalizedTartanValues)

    extendedLabels = labels + ['GMean']
    extendedBaselineValues = [1.0] * (len(labels) + 1)
    extendedTartanValues = normalizedTartanValues + [geoMeanTartan]

    xExtended = np.arange(len(extendedLabels))

    fig, ax = plt.subplots(figsize=(20, 12))
    width = 0.35
    ax.bar(xExtended - width/2, extendedBaselineValues, width, label='Baseline', color='#1f77b4')
    ax.bar(xExtended + width/2, extendedTartanValues, width, label='Tartan', color='#2ca02c')

    ax.set_xlabel('Applications', fontsize=14)
    ax.set_ylabel('Normalized Performance', fontsize=14)
    ax.set_title('End-to-End Performance of Tartan', fontsize=16, fontweight='bold')
    ax.set_xticks(xExtended)
    ax.set_xticklabels(extendedLabels, rotation=45, ha='right', fontsize=12)

    legend = ax.legend(fontsize=12, frameon=True, loc='upper left', bbox_to_anchor=(1, 1))
    legend.get_frame().set_color('white')
    legend.get_frame().set_edgecolor('black')

    for i in range(len(extendedTartanValues)):
        height = extendedTartanValues[i]
        ax.text(xExtended[i] + width/2, height + 0.02, f'{height:.2f}', ha='center', va='bottom', rotation=90, color='#2ca02c', fontsize=12)

    ax.yaxis.grid(True)

    plt.tight_layout()
    os.chdir(originalDir)

    print(f"Saving performance results in {resFile}")
    max_length = max(len(extendedBaselineValues), len(extendedTartanValues), len(allRobots) + 1)

    df = pd.DataFrame({
        "Robot": allRobots + ["GMean"] + [""] * (max_length - len(allRobots) - 1),
        "Baseline": extendedBaselineValues + ["" for _ in range(max_length - len(extendedBaselineValues))],
        "Tartan": extendedTartanValues + ["" for _ in range(max_length - len(extendedTartanValues))]
        })
    df.to_csv(resFile, index=False)

    print(f"Saving plot {plotFile}")
    plt.savefig(plotFile, dpi=300)

    if not isRunningInDocker: plt.show()

except tk.TclError:
    print("No display available. Plot has been saved in {plotFile}")
