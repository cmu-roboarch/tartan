# Tartan: Microarchitecting a Robotic Processor

This repository hosts the source code linked to the research detailed in the following paper presented at the International Symposium on Computer Architecture (ISCA), 2024:

```
@inproceedings{tartan,
  title={Tartan: Microarchitecting a Robotic Processor},
  author={Mohammad Bakhshalipour and Phillip B. Gibbons},
  booktitle={International Symposium on Computer Architecture (ISCA)},
  year={2024}
}
```

## Introduction

The repository encompasses the implementation of Tartan's software and hardware components and includes scripts for result replication. The software components, leveraging x86 assembly, are based on the [RoWild](https://cmu-roboarch.github.io/rowild/) benchmark suite's six end-to-end robotic applications. Hardware components are simulated using the [ZSim](https://github.com/s5z/zsim) architectural simulator.

## Replication Instructions

### Native Execution

To install the software infrastructure on the host machine:

```
git clone https://github.com/cmu-roboarch/tartan.git
./setup.sh
source ${HOME}/.bashrc
```

After installation, execute:

```
./replicate.py
```

This generates a `results/` directory with a `.csv` and a `.png` showing the results.

### Docker Execution

For those preferring containerization, a Docker image is available. This method may be slower than native execution. You may need to run the following commands with `sudo`.

**Docker Installation:**

```
apt-get install docker.io
systemctl start docker
service docker status
```

**Image Pull:**

```
docker pull kasraa/tartan:latest
```

**Container Execution:**

```
mkdir -p results
docker run --net=host -it --privileged --name my_interactive_tartan -v "$(pwd)/results:/tartan/results" kasraa/tartan:latest
```

Modify commands as necessary. Post-execution, the `results/` directory will contain the outcome files.

## Repository Structure

### `/apps`

Hosts RoWild's application source codes, input sets, and Tartan's software modifications.

- **Includes**:
  - `/XBot`: Source code for the `X` robot.
  - `/include`: Shared functions across applications.
  - Scripts for building (`build.sh`) and cleaning (`clean.sh`) applications, and `makefile.rules`.

### `/zsim`

Contains ZSim simulator source code and Tartan's hardware component implementations.

### `/misc`

Scripts for code convention enforcement and other utilities.

### Root Directory (`/`)

- **Contents**:
  - `setup.sh`: Bash script for software dependency installations.
  - `replicate.py`: Python script for simulation execution.
  - `./docker_entrypoint.sh`: Entry point for Docker container.

## Support

For inquiries, clarifications, or support, contact [m.bakhshalipour@gmail.com](mailto:m.bakhshalipour@gmail.com). Responses will be timely.

## License

Tartan is released under the MIT License. See the full license in the repository for details.

```
MIT License

Copyright (c) 2024 Carnegie Mellon University

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```
