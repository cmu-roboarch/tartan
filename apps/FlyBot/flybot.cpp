/*
 * MIT License
 *
 * Copyright (c) 2024 Carnegie Mellon University
 *
 * This file is part of Tartan.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits.h>
#include <queue>
#include <tuple>
#include <vector>

#include "args.h"
#include "log.h"
#include "utils.h"
#include "zsim_hooks.h"

#define MAX_X (128)
#define MAX_Y (128)
#define MAX_Z (128)

using namespace std;
using namespace args;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

typedef std::vector<int> PATH;

bool ***occGrid;
int minX, maxX, minY, maxY, minZ, maxZ;
int dX[6] = {-1, 1, 0, 0, 0, 0};
int dY[6] = {0, 0, -1, 1, 0, 0};
int dZ[6] = {0, 0, 0, 0, -1, 1};

double totalExecTime = 0, hCalcTime = 0;

std::vector<std::vector<int>> readDestinations(std::string fileName);
void readMap(std::string fileName);
bool isFree(int x, int y, int z);
PATH plan(int startX, int startY, int startZ, int goalX, int goalY, int goalZ,
          float hWeight);
float getEuclideanDist(int sx, int sy, int sz, int gx, int gy, int gz);
float getHeuristic(int sx, int sy, int sz, int gx, int gy, int gz);
void droneDynamics(float (&state)[6], float ctrl0, float ctrl1, float ctrl2,
                   const float dt);
std::vector<float> genControl(const PATH &path, float cost, int goalX,
                              int goalY, int goalZ);

int INITIAL_EPS = 4;

struct Point3D {
    float x, y, z;

    bool operator==(const Point3D &other) const {
        const float epsilon = 1e-3;
        return std::fabs(x - other.x) < epsilon &&
               std::fabs(y - other.y) < epsilon &&
               std::fabs(z - other.z) < epsilon;
    }
};

struct Point3DHash {
    size_t operator()(const Point3D &point) const {
        size_t hx = std::hash<float>()(point.x);
        size_t hy = std::hash<float>()(point.y);
        size_t hz = std::hash<float>()(point.z);
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};

struct Wind {
    float speed;
    float direction;
};

const float airDensity = 1.225;
const float dragCoefficient = 0.47;
const float droneCrossSectionalArea = 0.1;
const float gravitationalConstant = 9.81;
const float droneMass = 1.0;
const int integrationSteps = 16;

Point3D pathFunction(float x, float y, float z, float t) {
    return Point3D{x + 5 * cos(t), y + 5 * sin(t), z};
}

Wind windFunction(Point3D position) { return Wind{0.1f * position.z, 45}; }

float magnitude(const Point3D &p) {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float calculateDragEnergy(
    float x, float y, float z,
    const std::function<Point3D(float, float, float, float)> &pathFunction) {
    float energy = 0.0;
    for (int i = 0; i < integrationSteps; ++i) {
        float t1 = static_cast<float>(i) / integrationSteps;
        float t2 = static_cast<float>(i + 1) / integrationSteps;

        Point3D point1 = pathFunction(x, y, z, t1);
        Point3D point2 = pathFunction(x, y, z, t2);

        Point3D velocity{(point2.x - point1.x), (point2.y - point1.y),
                         (point2.z - point1.z)};
        float speed = magnitude(velocity);

        energy += 0.5 * airDensity * dragCoefficient * droneCrossSectionalArea *
                  speed * speed;
    }
    return energy;
}

float calculateAltitudeEnergy(float startAltitude, float endAltitude) {
    float energy = 0.0;
    float deltaAltitude = (endAltitude - startAltitude) / integrationSteps;

    for (int i = 0; i < integrationSteps; ++i) {
        float altitude = startAltitude + i * deltaAltitude;
        energy += droneMass * gravitationalConstant * altitude;
    }
    return energy;
}

float calculateWindEnergy(
    float x, float y, float z,
    const std::function<Point3D(float, float, float, float)> &pathFunction,
    const std::function<Wind(Point3D)> &windFunction) {
    float energy = 0.0;
    for (int i = 0; i < integrationSteps; ++i) {
        float t = static_cast<float>(i) / integrationSteps;
        Point3D position = pathFunction(x, y, z, t);
        Wind wind = windFunction(position);

        energy += wind.speed;
    }
    return energy;
}

float startAltitude = 0;
float endAltitude = 10;

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> destinationsFileArg(parser, "destinations", "",
                                           "File including the destinations");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");
    assert_msg(destinationsFileArg.found(), "Destiations file is not provided");

    std::string inputFile = inputMapArg.value();
    std::string destsFile = destinationsFileArg.value();
    std::string outputFile =
        outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    readMap(inputFile);

    auto locations = readDestinations(destsFile);
    std::vector<PATH> pathsLog;
    std::vector<std::vector<float>> ctrlLog;

    // >>> ROI
    zsim_roi_begin();
    std::vector<int> s = locations[0];
    std::vector<int> g = locations[1];
    size_t bestCost = INT_MAX;
    PATH bestPath;

    for (int i = INITIAL_EPS; i >= 1; i--) {
        PATH path =
            plan(s[0], s[1], s[2], g[0], g[1], g[2], static_cast<float>(i));

        if (path.empty()) break; // No path

        if (path.size() <= bestCost) {
            bestCost = path.size();
            bestPath = path;
        }
    }

    std::vector<float> controlSeq =
        genControl(bestPath, bestCost, g[0], g[1], g[2]);

    pathsLog.push_back(bestPath);
    ctrlLog.push_back(controlSeq);

    // <<< ROI
    zsim_roi_end();

#ifndef HIGHPERF
    std::cout << "totalExecTime: " << totalExecTime << std::endl;
    std::cout << "hCalcTime: " << hCalcTime << std::endl;
    std::cout << "hCalcTimeContribution: " << hCalcTime / totalExecTime
              << std::endl;
#endif

    // Write the output path
    std::ofstream pathFile;
    pathFile.open(outputFile);
    for (auto path : pathsLog) {
        for (auto dir : path) {
            pathFile << dir << std::endl;
        }
        pathFile << std::string(20, '-') << std::endl;
    }
    for (auto seq : ctrlLog) {
        for (auto c : seq) {
            pathFile << c << std::endl;
        }
        pathFile << std::string(20, '-') << std::endl;
    }
    pathFile.close();

    return 0;
}

int gVals[MAX_X][MAX_Y][MAX_Z];
bool visited[MAX_X][MAX_Y][MAX_Z];
PATH plan(int startX, int startY, int startZ, int goalX, int goalY, int goalZ,
          float hWeight) {

    struct Node {
        int x, y, z;
        float g, f;
        Node *parent;
        int dir;

        Node(int _x, int _y, int _z, float _g, float _f, Node *_parent,
             int _dir) {
            x = _x;
            y = _y;
            z = _z;
            g = _g;
            f = _f;
            parent = _parent;
            dir = _dir;
        }
    } __attribute__((aligned));

    struct NodeCmp {
        bool operator()(const Node *left, const Node *right) {
            return left->f > right->f;
        }
    } __attribute__((aligned));

    typedef std::priority_queue<Node *, std::vector<Node *>, NodeCmp> MIN_HEAP;

    for (int i = 0; i < MAX_X; i++) {
        for (int j = 0; j < MAX_Y; j++) {
            for (int k = 0; k < MAX_Z; k++) {
                gVals[i][j][k] = INT_MAX;
                visited[i][j][k] = false;
            }
        }
    }

    auto isGoal = [goalX, goalY, goalZ](int x, int y, int z) {
        return ((x == goalX) && (y == goalY) && (z == goalZ));
    };

    MIN_HEAP openList;
    Node *startNode = new Node(
        startX, startY, startZ, 0,
        hWeight * getHeuristic(startX, startY, startZ, goalX, goalY, goalZ),
        NULL, -1);
    openList.push(startNode);
    Node *expNode = NULL;

    auto baseTime = high_resolution_clock::now();

    while (!openList.empty()) {
        expNode = openList.top();
        openList.pop();
        int expX = expNode->x;
        int expY = expNode->y;
        int expZ = expNode->z;

        if (unlikely(visited[expX - minX][expY - minY][expZ - minZ])) continue;
        visited[expX - minX][expY - minY][expZ - minZ] = true;

        if (isGoal(expX, expY, expZ)) {

#ifndef HIGHPERF
            auto t = high_resolution_clock::now();
            totalExecTime +=
                duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;
#endif

            PATH path;
            Node *n = expNode;
            while (n) {
                path.push_back(n->dir);
                n = n->parent;
            }

            path.pop_back();
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 6; i++) {
            int xx = expX + dX[i];
            int yy = expY + dY[i];
            int zz = expZ + dZ[i];
            if (visited[xx - minX][yy - minY][zz - minZ]) continue;
            if (!isFree(xx, yy, zz)) continue;

            float g = expNode->g + 1;
            float f =
                g + hWeight * getHeuristic(xx, yy, zz, goalX, goalY, goalZ);
            if (g < gVals[xx - minX][yy - minY][zz - minZ]) {
                gVals[xx - minX][yy - minY][zz - minZ] = g;
                openList.push(new Node(xx, yy, zz, g, f, expNode, i));
            }
        }
    }

    auto t = high_resolution_clock::now();
    totalExecTime += duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;

    return {};
}

void readMap(std::string fileName) {
    std::ifstream file(fileName);
    assert(file.good());

    std::string token;

    file >> token;
    assert(token == "X");
    file >> minX >> maxX;

    file >> token;
    assert(token == "Y");
    file >> minY >> maxY;

    file >> token;
    assert(token == "Z");
    file >> minZ >> maxZ;

    int xS = maxX - minX + 1;
    int yS = maxY - minY + 1;
    int zS = maxZ - minZ + 1;

    assert(xS < MAX_X) assert(yS < MAX_Y) assert(zS < MAX_Z)

        occGrid = new bool **[xS];
    for (int i = 0; i < xS; i++) {
        occGrid[i] = new bool *[yS];
        for (int j = 0; j < yS; j++) {
            occGrid[i][j] = new bool[zS]();
        }
    }

    int x, y, z;
    while (file >> x >> y >> z) {
        occGrid[x - minX][y - minY][z - minZ] = true;
    }
    file.close();
}

bool isFree(int x, int y, int z) {
    if ((x <= minX) || (x >= maxX) || (y <= minY) || (y >= maxY) ||
        (z <= minZ) || (z >= maxZ)) {
        return false;
    }

    return !occGrid[x - minX][y - minY][z - minZ];
}

std::vector<std::vector<int>> readDestinations(std::string fileName) {
    std::ifstream file(fileName);
    assert(file.good());

    std::vector<std::vector<int>> dests;

    int x, y, z;
    while (file >> x >> y >> z) {
        dests.push_back({x, y, z});
    }

    return dests;
}

float getEuclideanDist(int sx, int sy, int sz, int gx, int gy, int gz) {
    return sqrt((sx - gx) * (sx - gx) + (sy - gy) * (sy - gy) +
                (sz - gz) * (sz - gz));
}

float getHeuristic(int sx, int sy, int sz, int gx, int gy, int gz) {
#ifndef HIGHPERF
    auto t0 = high_resolution_clock::now();
#endif

#ifdef TARTAN
    zsim_roi_end();
#endif

    float dragEnergy = calculateDragEnergy(sx, sy, sz, pathFunction);
    float altitudeEnergy = calculateAltitudeEnergy(startAltitude, endAltitude);
    float windEnergy =
        calculateWindEnergy(sx, sy, sz, pathFunction, windFunction);

#ifdef TARTAN
    zsim_roi_begin();
#endif

#ifndef HIGHPERF
    auto t1 = high_resolution_clock::now();
    hCalcTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
#endif

    return dragEnergy + altitudeEnergy + windEnergy;
}

void droneDynamics(float (&state)[6], float ctrl0, float ctrl1, float ctrl2,
                   const float dt) {
    state[0] += dt * state[3];
    state[1] += dt * state[4];
    state[2] += dt * state[5];
    state[3] += dt * (ctrl0 - 9.81);
    state[4] += dt * ctrl1;
    state[5] += dt * ctrl2;
}

std::vector<float> genControl(const PATH &path, float cost, int goalX,
                              int goalY, int goalZ) {
    std::vector<float> controlSeq;

    float alpha = 0.01;
    controlSeq.resize(3 * path.size());
    for (int iter = 0; iter < 10; iter++) {
        for (int i = 0; i < static_cast<int>(path.size()); i++) {
            for (int dim = 0; dim < 3; dim++) {
                controlSeq[i + dim] -= alpha * (cost / path.size());
            }
        }
    }

    return controlSeq;
}
