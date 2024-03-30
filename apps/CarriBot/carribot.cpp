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
#include <limits.h>
#include <queue>
#include <tuple>
#include <vector>

#include "args.h"
#include "log.h"
#include "utils.h"
#include "zsim_hooks.h"

#define MAX_X (2048)
#define MAX_Y (2048)
#define NUM_THETAS (8) // Degree change: 360°/8 = 45°

using namespace std;
using namespace args;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

typedef std::vector<int> PATH;

float **env;
int mapX, mapY;
const float probFree = 0.3, probOccupied = 0.7;
int robotLength = 4, robotWidth = 6;
double totalExecTime = 0, collCheckTime = 0;
int collThreshold = 1;
std::vector<int> sinThetas, cosThetas;

void readMap(std::string fileName, int scale);
bool isFree(int x, int y, int theta);
PATH plan(int startX, int startY, int startTheta, int goalX, int goalY,
          int goalTheta, float hWeight);

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<int> numTestsArg(parser, "num-tests", "", "Number of tests");
    KVArg<float> hWeightArg(parser, "weight", "", "Heuristic weight of A*");
    KVArg<int> scaleMapArg(parser, "scale-map", "", "Map scale factor");
    KVArg<int> scaleRobotArg(parser, "scale-robot", "", "Robot scale factor");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    std::string inputFile = inputMapArg.value();
    int numTests = numTestsArg.found() ? numTestsArg.value() : 1;
    float hWeight = hWeightArg.found() ? hWeightArg.value() : 1.0;
    int scaleMap = scaleMapArg.found() ? scaleMapArg.value() : 1;
    int scaleRobot = scaleRobotArg.found() ? scaleRobotArg.value() : 1;
    std::string outputFile =
        outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    readMap(inputFile, scaleMap);

    robotLength *= scaleRobot;
    robotWidth *= scaleRobot;
    for (int i = 0; i < NUM_THETAS; i++) {
        sinThetas.push_back(sin(i * TWO_PI / NUM_THETAS));
        cosThetas.push_back(cos(i * TWO_PI / NUM_THETAS));
    }

    auto generateRandomValidPoint = [&]() {
        while (true) {
            int x = rand() % mapX;
            int y = rand() % mapY;
            int theta = rand() % NUM_THETAS;
            if (isFree(x, y, theta)) {
                return std::make_tuple(x, y, theta);
            }
        }
    };

    auto printPath = [](int sx, int sy, int st, const PATH &path,
                        bool toFile = false,
                        const std::string &filename = "") -> void {
        std::ostream *out = &std::cout; // Default output is to console
        std::ofstream file;

        if (toFile) {
            file.open(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return;
            }
            out = &file;
        }

        for (int p : path) {
            *out << "(" << sx << ", " << sy << ", " << st << ")" << std::endl;
            switch (p) {
            case 0:
                st = (st + 1) % NUM_THETAS;
                break;
            case 1:
                st = (st - 1 + NUM_THETAS) % NUM_THETAS;
                break;
            case 2:
                sx += round(cosThetas[st]);
                sy += round(sinThetas[st]);
                break;
            case 3:
                sx -= round(cosThetas[st]);
                sy -= round(sinThetas[st]);
                break;
            }
        }
        *out << "(" << sx << ", " << sy << ", " << st << ")" << std::endl;

        if (file.is_open()) {
            file.close();
        }
    };

    for (int i = 0; i < numTests; i++) {
        auto [sx, sy, st] = generateRandomValidPoint();
        auto [gx, gy, gt] = generateRandomValidPoint();
        // std::cout << "Planning from s:(" << sx << ", " << sy << ", " << st
        // << ") to g:(" << gx << ", " << gy << ", " << gt << ")" << std::endl;

        auto path = plan(sx, sy, st, gx, gy, gt, hWeight);
        printPath(sx, sy, st, path, true, outputFile);
    }

#ifndef HIGHPERF
    std::cout << "totalExecTime: " << totalExecTime / numTests << std::endl;
    std::cout << "collCheckTime: " << collCheckTime / numTests << std::endl;
    std::cout << "collCheckTimeContribution: " << collCheckTime / totalExecTime
              << std::endl;
#endif

    return 0;
}

// Could be too large to be allocated in function stack memory
int gVals[MAX_Y][MAX_X][NUM_THETAS];
bool visited[MAX_Y][MAX_X][NUM_THETAS];
PATH plan(int startX, int startY, int startTheta, int goalX, int goalY,
          int goalTheta, float hWeight) {

    assert_msg(isFree(startX, startY, startTheta),
               "The initial state cannot be infeasible/occupied");
    assert_msg(isFree(goalX, goalY, goalTheta),
               "The goal state cannot be infeasible/occupied");

    struct Node {
        int x, y, theta;
        float g, f;
        Node *parent;
        int dir;

        Node(int _x, int _y, int _theta, float _g, float _f, Node *_parent,
             int _dir) {
            x = _x;
            y = _y;
            theta = _theta;
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

    for (int i = 0; i < MAX_Y; i++) {
        for (int j = 0; j < MAX_X; j++) {
            for (int k = 0; k < NUM_THETAS; k++) {
                gVals[i][j][k] = INT_MAX;
                visited[i][j][k] = false;
            }
        }
    }

    auto getHeuristic = [goalX, goalY](int x, int y) {
        return (x - goalX) * (x - goalX) + (y - goalY) * (y - goalY);
    };

    auto isGoal = [goalX, goalY, goalTheta](int x, int y, int theta) {
        return ((x == goalX) && (y == goalY) && (theta == goalTheta));
    };

    MIN_HEAP openList;
    Node *startNode =
        new Node(startX, startY, startTheta, 0,
                 hWeight * getHeuristic(startX, startY), NULL, -1);
    openList.push(startNode);
    Node *expNode = NULL;

    auto baseTime = high_resolution_clock::now();

    // >>> ROI
    zsim_roi_begin();
    while (!openList.empty()) {
        expNode = openList.top();
        openList.pop();
        int expX = expNode->x;
        int expY = expNode->y;
        int expTheta = expNode->theta;

        if (unlikely(visited[expY][expX][expTheta])) continue;
        visited[expY][expX][expTheta] = true;

        if (isGoal(expX, expY, expTheta)) {

            zsim_roi_end();
            // <<< ROI

            auto t = high_resolution_clock::now();
            totalExecTime +=
                duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;

            PATH path;
            Node *n = expNode;
            while (n) {
                path.push_back(n->dir /* directly store control commands */);
                n = n->parent;
            }

            path.pop_back();
            std::reverse(path.begin(), path.end());
            return path;
        }

        int dX = round(cosThetas[expTheta]);
        int dY = round(sinThetas[expTheta]);
        int neighbors[4][3] = {
            {expX, expY, (expTheta + 1) % NUM_THETAS},
            {expX, expY, (expTheta - 1 + NUM_THETAS) % NUM_THETAS},
            {expX + dX, expY + dY, expTheta},
            {expX - dX, expY - dY, expTheta}};

        for (int i = 0; i < 4; i++) {
            int xx = neighbors[i][0];
            int yy = neighbors[i][1];
            int tt = neighbors[i][2];
            if (visited[yy][xx][tt]) continue;
            if (!isFree(xx, yy, tt)) continue;

            float g = expNode->g + 1;
            float f = g + hWeight * getHeuristic(xx, yy);
            if (g < gVals[yy][xx][tt]) {
                gVals[yy][xx][tt] = g;
                openList.push(new Node(xx, yy, tt, g, f, expNode, i));
            }
        }
    }

    zsim_roi_end();
    // <<< ROI

    auto t = high_resolution_clock::now();
    totalExecTime += duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;

    return {};
}

void readMap(std::string fileName, int scale) {
    std::ifstream file(fileName);
    assert(file.good());

    std::string line;
    std::size_t pos;

    auto removeCR = [](std::string str) {
        // NOTE: The benchmarks files released by Moving AI have not been
        // produced observing Unix<->Windows file exchange norms. As such, a
        // new line in the files is represented by CR+LF instead of just LF.
        if (static_cast<int>(str.back()) == 13 /*CR ASCII*/) {
            str.pop_back();
        }
        return str;
    };

    std::getline(file, line);
    line = removeCR(line);
    assert(line == "type octile");

    std::getline(file, line);
    line = removeCR(line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "height");
    mapY = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "width");
    mapX = stoi(line.substr(pos + 1));

    std::getline(file, line);
    line = removeCR(line);
    assert(line == "map");

    /*
     * Moving AI map format:
     *  @: Obstacle
     *  .: Free
     *
     *              Width(X)
     *            ◄─────────►
     *           ▲ ....@@@@@@
     *           │ .....@@@@@
     * Height(Y) │ ......@@@@
     *           │ .......@@@
     *           ▼ ........@@
     *
     * >> Width is X and Height is Y in data structures
     * >> Top-left corner is (X=0, Y=0)
     *
     * The data structure of the environment (env) resembles its actual shape
     * to exploit the row-major memory layout locality (length of the car is
     * larger than its width). env[y][x] shows coordinate (x, y) in the map.
     *
     *
     * The modeled car:
     *
     *           Length(X)
     *          ┌─────────┐
     * Width(Y) │         │
     *          └─────────┘
     * >> Top-left corner is (X=0, Y=0, Theta=0)
     */

    env = new float *[scale * mapY];
    for (int i = 0; i < scale * mapY; i++) {
        env[i] = new float[scale * mapX];
    }

    for (int y = 0; y < mapY; y++) {
        std::getline(file, line);
        line = removeCR(line);
        assert(static_cast<int>(line.length()) == mapX);

        for (int x = 0; x < mapX; x++) {
            char point = line[x];
            assert(point == '.' /*free*/ || point == '@' /*obstacle*/);
            bool obstacle = point == '@';

            int baseY = scale * y;
            int baseX = scale * x;
            for (int sy = 0; sy < scale; sy++) {
                for (int sx = 0; sx < scale; sx++) {
                    env[baseY + sy][baseX + sx] =
                        obstacle ? probOccupied : probFree;
                }
            }
        }
    }
    file.close();

    mapX *= scale;
    mapY *= scale;
}

bool isFree(int x, int y, int theta) {
#ifndef HIGHPERF
    auto t0 = high_resolution_clock::now();
#endif

#ifdef TARTAN
    zsim_roi_end();
#endif
    int colls = 0;
    for (int i = 0; i <= robotWidth; i++) {
        for (int j = 0; j <= robotLength; j++) {
            int xx = x + round(j * cosThetas[theta]);
            int yy = y + round(i * sinThetas[theta]);

            if ((xx < 0) || (xx >= mapX) || (yy < 0) || (yy >= mapY) ||
                (env[yy][xx] > probFree)) {
                colls++;
            }
        }
    }
#ifdef TARTAN
    zsim_roi_begin();

    auto packOperands = [&env](uint32_t offset, uint32_t orient) -> uint64_t {
        return (reinterpret_cast<uint64_t>(env) +
                (static_cast<uint32_t>(offset) << 32)) |
               orient;
    };

    int vStep = 64 / sizeof(float);

    for (int i = 0; i <= robotWidth; i++) {
        for (int j = 0; j <= robotLength; j += vStep) {
            int yy = y + round(i * sinThetas[theta]);
            ovec_load(packOperands((yy * mapX + x) * sizeof(float),
                                   j * cosThetas[theta]));
        }
    }

#endif

#ifndef HIGHPERF
    auto t1 = high_resolution_clock::now();
    collCheckTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
#endif

    return colls < collThreshold;
}
