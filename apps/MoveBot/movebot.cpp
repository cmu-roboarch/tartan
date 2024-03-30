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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "args.h"
#include "env.h"
#include "log.h"
#include "lsh.h"
#include "pid.h"
#include "utils.h"
#include "vcl/vectorclass.h"
#include "zsim_hooks.h"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

typedef std::vector<float> JOINT_CFG;
Environment *env;
std::vector<JOINT_CFG> rrtVertices;
std::vector<JOINT_CFG> path;
std::vector<float> controlSignals;

double totalExecTime = 0, nnsTime = 0;

/* LoCoBot */
const int numJoints = 5;
float rDesc[6][6] = {
    {0, 0, 0, 0.08, 0, 0.159},   {0, 0, 0, 0, 0, 0.04125},
    {0, 0, 0, 0.05, 0, 0.2},     {0, 0, 0, 0.2002, 0, 0},
    {0, 0, 0, 0.063, 0.0001, 0}, {0, 0, 0, 0.106525, 0, 0.0050143}};
float axis[6][3]{{0, 0, 1}, {0, 1, 0},  {0, 1, 0},
                 {0, 1, 0}, {-1, 0, 0}, {0, 1, 0}};
float TLink[6][4][4];
float TJoint[6][4][4];
float TCurr[6][4][4];
float q[6] = {0, 0, 0, 0, 0, 0};
const float qMin = -PI / 2;
const float qMax = PI / 2;
float cIdx[4] = {1, 2, 3, 4};
float cDesc[4][6] = {
    {0, 0, 0, 0, 0, 0.09},
    {0, 0, 0, 0.075, 0, 0},
    {0, 0, 0, 0.027, -0.012, 0},
    {0, 0, 0, 0.055, 0, 0.01},
};
float cDim[4][3] = {
    {0.05, 0.05, 0.25},
    {0.25, 0.05, 0.05},
    {0.07, 0.076, 0.05},
    {0.11, 0.11, 0.07},
};
float TBlock[4][4][4];
float TColl[4][4][4];
float cPoints[4][9][3] = {};
float cAxes[4][3][3] = {};

int findNearest(const JOINT_CFG &q);
void run(const JOINT_CFG &start, const JOINT_CFG &goal, float threshold,
         int maxSamples, float goalBias);
JOINT_CFG sampleRobotCfg();
bool detectCollNode(const JOINT_CFG ang);
bool detectCollEdge(const JOINT_CFG &ang1, const JOINT_CFG &ang2,
                    int numSteps = 5);
void forwardKin(const JOINT_CFG &ang);
void compCollBlockPoints(const JOINT_CFG &ang);
float getNorm(const JOINT_CFG &q1, const JOINT_CFG &q2);

// PID controllers for each joint
std::vector<PIDController> pids = {{1.0, 0.1, 0.05},
                                   {1.0, 0.1, 0.05},
                                   {1.0, 0.1, 0.05},
                                   {1.0, 0.1, 0.05},
                                   {1.0, 0.1, 0.05}};

/* General */
float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

void outputLog(std::string fileName, const JOINT_CFG &startCfg,
               const JOINT_CFG &goalCfg, std::vector<float> const &signals);

LSHTable lsh;

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<float> thresholdArg(parser, "thresh", "",
                              "Threshold (epsilon and radius)");
    KVArg<int> samplesArg(parser, "samples", "",
                          "The maximum number of samples");
    KVArg<float> goalBiasArg(parser, "bias", "", "Bias towards goal in RRT");
    KVArg<std::string> outputLogArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    const char *inputFile = inputMapArg.value().c_str();
    float goalBias = goalBiasArg.found() ? goalBiasArg.value() : 0.05;
    float threshold = thresholdArg.found() ? thresholdArg.value() : 0.25;
    int samples = samplesArg.found() ? samplesArg.value() : 3000;
    const char *outputFile =
        outputLogArg.found() ? outputLogArg.value().c_str() : "/dev/null";

    JOINT_CFG start = {-1.3962634, 0, 0, 0, 0};
    JOINT_CFG goal = {0, 1.04719755, -1.30899694, -1.30899694, 0};

    env = new Environment(inputFile);

    for (int i = 0; i < 6; i++) {
        rpyxyzToH(rDesc[i][0], rDesc[i][1], rDesc[i][2], rDesc[i][3],
                  rDesc[i][4], rDesc[i][5], TLink[i]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &TJoint[i][0][0]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &TCurr[i][0][0]);
    }

    forwardKin({0, 0, 0, 0, 0});

    for (int i = 0; i < 4; i++) {
        rpyxyzToH(cDesc[i][0], cDesc[i][1], cDesc[i][2], cDesc[i][3],
                  cDesc[i][4], cDesc[i][5], TBlock[i]);
        std::copy(&I4[0][0], &I4[0][0] + 16, &TColl[i][0][0]);
    }

    srand(0);

    auto t0 = high_resolution_clock::now();
    // >>> ROI
    zsim_roi_begin();

    run(start, goal, threshold, samples, goalBias);

    zsim_roi_end();
    // <<< ROI
    auto t1 = high_resolution_clock::now();
    totalExecTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;

    outputLog(outputFile, start, goal, controlSignals);

#ifndef HIGHPERF
    std::cout << "totalExecTime: " << totalExecTime << std::endl;
    std::cout << "nnsTime: " << nnsTime << std::endl;
    std::cout << "nnsTimeContribution: " << nnsTime / totalExecTime
              << std::endl;
#endif

    delete env;

    return 0;
}

void outputLog(std::string fileName, const JOINT_CFG &startCfg,
               const JOINT_CFG &goalCfg, std::vector<float> const &signals) {
    auto printCfg = [](const JOINT_CFG &cfg) {
        std::string str = "";
        for (auto c : cfg)
            str += std::to_string(c) + " ";
        return str;
    };

    std::ofstream pathFile;
    pathFile.open(fileName);
    pathFile << "Start CFG: " << printCfg(startCfg) << std::endl;
    pathFile << "Goal CFG: " << printCfg(goalCfg) << std::endl;
    pathFile << "Path:" << std::endl;
    for (auto cfg : path) {
        pathFile << printCfg(cfg) << std::endl;
    }

    pathFile << "Signals: ";
    for (auto s : signals)
        pathFile << s << " ";

    pathFile << std::endl << std::string(30, '-') << std::endl;

    pathFile.close();
}

#ifndef TARTAN
int findNearest(const JOINT_CFG &q) {
    int bestIdx = 0;
    float minDist = getNorm(rrtVertices[0], q);

    for (int idx = 1; idx < static_cast<int>(rrtVertices.size()); idx++) {
        float d = getNorm(rrtVertices[idx], q);
        if (d < minDist) {
            bestIdx = idx;
            minDist = d;
        }
    }

    return bestIdx;
}
#endif

#ifdef TARTAN
int findNearest(const JOINT_CFG &q) { return lsh.findNearest(q); }
#endif

void run(const JOINT_CFG &start, const JOINT_CFG &goal, float threshold,
         int maxSamples, float goalBias) {

    std::vector<int> rrtEdges;
    rrtVertices.push_back(start);

#ifdef TARTAN
    lsh.add(start, rrtVertices.size() - 1);
#endif

    rrtEdges.push_back(0);
    bool foundSol = false;

    while (static_cast<int>(rrtVertices.size()) < maxSamples) {
        JOINT_CFG qRand = sampleRobotCfg();
        if (static_cast<float>(rand()) / RAND_MAX < 0.05) qRand = goal;

#ifndef HIGHPERF
        auto t0 = high_resolution_clock::now();
#endif

        int idxNear = findNearest(qRand);

#ifndef HIGHPERF
        auto t1 = high_resolution_clock::now();
        nnsTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
#endif

        assert(idxNear < static_cast<int>(rrtEdges.size()));
        assert(idxNear < static_cast<int>(rrtVertices.size()));
        JOINT_CFG qNear = rrtVertices[idxNear];
        assert(qNear.size() == qRand.size());

        JOINT_CFG qConnect;
        float randNearDist = getNorm(qRand, qNear);

        if (randNearDist > threshold) {
            float f = threshold / randNearDist;
            for (int i = 0; i < static_cast<int>(qRand.size()); i++) {
                qConnect.push_back(qNear[i] + f * (qRand[i] - qNear[i]));
            }
        } else {
            qConnect = qRand;
        }

        bool y = detectCollEdge(qConnect, qNear);
        if (!y) {
            rrtVertices.push_back(qConnect);

#ifdef TARTAN
            lsh.add(qConnect, rrtVertices.size() - 1);
#endif

            rrtEdges.push_back(idxNear);
        }

#ifndef HIGHPERF
        auto t2 = high_resolution_clock::now();
#endif

        idxNear = findNearest(goal);

#ifndef HIGHPERF
        auto t3 = high_resolution_clock::now();
        nnsTime += duration_cast<nanoseconds>(t3 - t2).count() * 1e-9;
#endif
        if (getNorm(goal, rrtVertices[idxNear]) < threshold) {
            rrtVertices.push_back(goal);
            rrtEdges.push_back(idxNear);
            foundSol = true;
            break;
        }
    }

    assert(rrtVertices.size() == rrtEdges.size());

    if (foundSol) {
        // Last vertex is goal
        int c = static_cast<int>(rrtVertices.size()) - 1;
        JOINT_CFG cfg1 = rrtVertices[c];
        path.push_back(cfg1);
        do {
            c = rrtEdges[c];
            JOINT_CFG cfg2 = rrtVertices[c];
            path.push_back(cfg2);

            for (size_t j = 0; j < cfg1.size(); j++) {
                controlSignals.push_back(
                    pids[j].update(cfg1[j], cfg2[j], 0.01));
            }

            cfg1 = cfg2;
        } while (c != 0);
    }
}

JOINT_CFG sampleRobotCfg() {
    JOINT_CFG cfg;
    for (int i = 0; i < numJoints; i++) {
        float r = static_cast<float>(rand()) / RAND_MAX;
        cfg.push_back(qMin + (qMax - qMin) * r);
    }

    return cfg;
}

bool detectCollNode(const JOINT_CFG ang) {
    compCollBlockPoints(ang);

    auto pointsObs = env->getPointsObs();
    auto axesObs = env->getAxesObs();
    assert(pointsObs.size() == axesObs.size());
    int numObstacles = static_cast<int>(axesObs.size());

    bool *checks = new bool[4 * numObstacles];

#ifdef HIGHPERF
#pragma omp parallel for num_threads(4)
#endif
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < numObstacles; j++) {
            checks[i * numObstacles + j] = cuboidCuboidCollision(
                cPoints[i], cAxes[i], pointsObs[j].point, axesObs[j].axes);
        }
    }

    for (int i = 0; i < 4 * numObstacles; i++) {
        if (checks[i]) {
            delete[] checks;
            return true;
        }
    }

    delete[] checks;
    return false;
}

bool detectCollEdge(const JOINT_CFG &ang1, const JOINT_CFG &ang2,
                    int numSteps) {
    assert(ang1.size() == ang2.size());

    float stride = 1.0 / (numSteps - 1);
    for (float s = 0; s <= 1; s += stride) {
        JOINT_CFG ang;
        for (size_t k = 0; k < ang1.size(); k++) {
            ang.push_back(ang1[k] + s * (ang2[k] - ang1[k]));
        }

        if (detectCollNode(ang)) return true;
    }

    return false;
}

void forwardKin(const JOINT_CFG &ang) {
    for (int i = 0; i < numJoints; i++) {
        q[i] = ang[i];
    }

    auto isAxis = [&](int idx, const float(&c)[3]) {
        return std::equal(std::begin(axis[idx]), std::end(axis[idx]),
                          std::begin(c));
    };

    // Compute current joint and end effector coordinate frames
    // Notice not all joints rotate about the z axis!
    for (int i = 0; i < 6; i++) {
        if (isAxis(i, {0, 0, 1})) {
            float t[4][4] = {{cosf(q[i]), -sinf(q[i]), 0, 0},
                             {sinf(q[i]), cosf(q[i]), 0, 0},
                             {0, 0, 1, 0},
                             {0, 0, 0, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &TJoint[i][0][0]);
        } else if (isAxis(i, {-1, 0, 0})) {
            float t[4][4] = {{1, 0, 0, 0},
                             {0, cosf(q[i]), sinf(q[i]), 0},
                             {0, -sinf(q[i]), cosf(q[i]), 0},
                             {0, 0, 0, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &TJoint[i][0][0]);
        } else if (isAxis(i, {0, 1, 0})) {
            float t[4][4] = {{cosf(q[i]), 0, sinf(q[i]), 0},
                             {0, 1, 0, 0},
                             {-sinf(q[i]), 0, cosf(q[i]), 0},
                             {0, 0, 0, 1}};
            std::copy(&t[0][0], &t[0][0] + 16, &TJoint[i][0][0]);
        } else {
            panic("Axis rotation is not defined");
        }

        if (i == 0) {
            matrixMultiplication<float, 4, 4, 4, 4>(TLink[i], TJoint[i],
                                                    TCurr[i]);
        } else {
            float temp[4][4];
            matrixMultiplication<float, 4, 4, 4, 4>(TCurr[i - 1], TLink[i],
                                                    temp);
            matrixMultiplication<float, 4, 4, 4, 4>(temp, TJoint[i], TCurr[i]);
        }
    }
}

void compCollBlockPoints(const JOINT_CFG &ang) {
    forwardKin(ang);

    // Compute current collision boxes for the arm
    for (int i = 0; i < 4; i++) {
        int idx = cIdx[i];
        matrixMultiplication<float, 4, 4, 4, 4>(
            TCurr[idx] /*Joint frame*/, TBlock[i] /*Local box transform*/,
            TColl[i]);

        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                cAxes[i][j][k] = TColl[i][k][j];
            }
        }

        blockDescToBoundingBox(TColl[i], cDim[i], cPoints[i]);
    }
}

float getNorm(const JOINT_CFG &q1, const JOINT_CFG &q2) {
    assert(q1.size() == q2.size());
    float dist = 0;
    for (int i = 0; i < static_cast<int>(q1.size()); i++) {
        dist += (q1[i] - q2[i]) * (q1[i] - q2[i]);
    }
    return sqrt(dist);
}
