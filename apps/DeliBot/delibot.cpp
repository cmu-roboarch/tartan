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

#include "args.h"
#include "log.h"
#include "utils.h"
#include "zsim_hooks.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits.h>
#include <random>
#include <string>
#include <vector>

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

/* Map */
int sizeX, sizeY;
float **map;
std::vector<int> *freeXs, *freeYs;
void readMap(std::string fileName);
bool isValid(int x, int y);
bool isFree(int x, int y);
float getProb(int x, int y);

/* Particle filter */
typedef std::vector<float> READING;
struct Particle {
    float x, y, theta, w;

    Particle(float _x, float _y, float _theta, float _w)
        : x(_x), y(_y), theta(_theta), w(_w) {}

    Particle(const Particle &_p) {
        x = _p.x;
        y = _p.y;
        theta = _p.theta;
        w = _p.w;
    }
};
int numParticles;
std::vector<Particle> particles;
float alpha1 = 0.001, alpha2 = 0.001, alpha3 = 0.1, alpha4 = 0.8;
float zHit = 10.0, zShort = 0.01, zMax = 0.1, zRand = 10.0;
float sigmaHit = 50.0, lambdaShort = 0.1;
float minProbability = 0.35, maxRange = 700.0, resolution = 25.0;
float sensorOffset = 25.0;
std::default_random_engine resamRandGen;
std::uniform_real_distribution<float> *resamDist;
void initializeParticles();
void updateMotion(READING *prevOdometry, READING *currOdometry);
void updateSensor(READING *laserReading);
float calcProbability(float zkt, float zktStar);
float rayCast(float x, float y, float theta, float degree);
void initializeResampler();
void resample();
std::vector<float> getBelief();

/* Measurement */
#define NUM_ODOMETRY_MEASUR (3) // X, Y, Theta
#define NUM_LASER_MEASUR (180)  // All angles
typedef std::vector<READING> MEASUREMENT_LOG;
void readMeasurements(std::string inputFile, MEASUREMENT_LOG *odometryLog,
                      MEASUREMENT_LOG *laserLog);

/* Planner */
float goalX, goalY;
float legSteps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
int plan(float x, float y);

double totalExecTime = 0, rayCastingTime = 0;
int main(int argc, const char **argv) {
    using args::FlagArg;
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<std::string> measurementsArg(parser, "measurements", "",
                                       "Input measurements file");
    KVArg<int> particlesArg(parser, "particles", "", "Number of particles");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");
    assert_msg(measurementsArg.found(),
               "Input measurements file is not provided");

    std::string inputMapFile = inputMapArg.value();
    std::string measurementsFile = measurementsArg.value();
    int _numParticles = particlesArg.found() ? particlesArg.value() : 500;
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";

    assert(_numParticles > 0);

    readMap(inputMapFile);

    MEASUREMENT_LOG *odometryLog = new MEASUREMENT_LOG();
    MEASUREMENT_LOG *laserLog = new MEASUREMENT_LOG();
    readMeasurements(measurementsFile, odometryLog, laserLog);
    assert(odometryLog->size() == laserLog->size());

    numParticles = _numParticles;
    initializeParticles();
    initializeResampler();

    goalX = 7000.0;
    goalY = 5000.0;

    MEASUREMENT_LOG outputLog;
    outputLog.push_back(getBelief());

    std::vector<int> planLog;

    // >>> ROI
    zsim_roi_begin();
    for (int i = 1; i < static_cast<int>(odometryLog->size()); i++) {
        READING *prevOdometry = &odometryLog->at(i - 1);
        READING *currOdometry = &odometryLog->at(i);
        assert(static_cast<int>(prevOdometry->size()) == NUM_ODOMETRY_MEASUR);
        assert(static_cast<int>(currOdometry->size()) == NUM_ODOMETRY_MEASUR);

        READING *laserReading = &laserLog->at(i);
        assert(static_cast<int>(laserReading->size()) == NUM_LASER_MEASUR);

        auto t0 = high_resolution_clock::now();

        updateMotion(prevOdometry, currOdometry);
        updateSensor(laserReading);
        resample();

        auto state = getBelief();
        auto move = plan(state[0], state[1]);

        auto t1 = high_resolution_clock::now();
        totalExecTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;

        outputLog.push_back(state);
        planLog.push_back(move);

        if (i >= 100) break; // Enough to get steady simulation results
    }
    zsim_roi_end();
    // <<< ROI

    // Write the output log
    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (auto l : outputLog) {
        for (auto e : l) {
            outLogFile << std::setprecision(4) << e << " ";
        }
        outLogFile << std::endl;
    }
    outLogFile << std::string(20, '-') << std::endl;
    for (auto p : planLog) {
        outLogFile << p << std::endl;
    }
    outLogFile.close();

#ifndef HIGHPERF
    std::cout << "totalExecTime: " << totalExecTime << std::endl;
    std::cout << "rayCastingTime: " << rayCastingTime << std::endl;
    std::cout << "rayCastingTimeContribution: "
              << rayCastingTime / totalExecTime << std::endl;
#endif

    delete resamDist;

    return 0;
}

void readMeasurements(std::string inputFile, MEASUREMENT_LOG *odometryLog,
                      MEASUREMENT_LOG *laserLog) {
    std::ifstream mFile;
    mFile.open(inputFile);
    assert(mFile.good());

    std::string line;
    while (std::getline(mFile, line)) {
        READING l;

        std::stringstream ss(line);
        float e;
        while (ss >> e) {
            l.push_back(e);
            if (static_cast<int>(l.size()) == NUM_ODOMETRY_MEASUR) {
                odometryLog->push_back(l);
                break;
            }
        }
        l.clear();

        while (ss >> e) {
            l.push_back(e);
        }
        assert(static_cast<int>(l.size()) == NUM_LASER_MEASUR);
        laserLog->push_back(l);
    }

    mFile.close();
}

void readMap(std::string fileName) {
    std::ifstream mapFile(fileName);
    assert(mapFile.good());

    std::string strToken;
    int intToken;

    mapFile >> strToken >> intToken;
    assert(strToken == "mapsize_x");
    sizeX = intToken;

    mapFile >> strToken >> intToken;
    assert(strToken == "mapsize_y");
    sizeY = intToken;

    mapFile >> strToken >> intToken;
    assert(strToken == "resolution");
    resolution = intToken;

    sizeX /= resolution;
    sizeY /= resolution;

    std::getline(mapFile, strToken);
    assert(strToken.empty());

    map = new float *[sizeX];
    for (int i = 0; i < sizeX; i++) {
        map[i] = new float[sizeY];
    }

    freeXs = new std::vector<int>();
    freeYs = new std::vector<int>();

    for (int i = 0; i < sizeX; i++) {
        std::getline(mapFile, strToken);
        assert(!strToken.empty());

        std::stringstream row(strToken);

        int j = 0;
        float e;
        while (row >> e) {
            map[i][j] = e;
            if (e == 0) {
                freeXs->push_back(i);
                freeYs->push_back(j);
            }
            j++;
        }

        assert(j == sizeY);
    }
    mapFile.close();
}

bool isValid(int x, int y) {
    return ((x >= 0) && (x < sizeX) && (y >= 0) && (y < sizeY));
}

bool isFree(int x, int y) { return map[x][y] == 0; }

float getProb(int x, int y) {
    assert(isValid(x, y));
    return map[x][y];
}

int plan(float x, float y) {
    int bestLeg = 0;
    float bestDistance = INT_MAX;
    for (int i = 0; i < 4; i++) {
        float dist =
            (x + legSteps[i][0] - goalX) * (x + legSteps[i][0] - goalX) +
            (y + legSteps[i][1] - goalY) * (y + legSteps[i][1] - goalY);

        if (dist < bestDistance) {
            bestLeg = i;
            bestDistance = dist;
        }
    }

    return bestLeg;
}

void initializeParticles() {
    assert(freeXs->size() == freeYs->size());
    int numFreeLocs = static_cast<int>(freeXs->size());

    std::default_random_engine intGen;
    std::uniform_int_distribution<int> intDist(0, numFreeLocs - 1);

    std::default_random_engine realGen;
    std::uniform_real_distribution<float> realDist(-PI, PI);

    for (int i = 0; i < numParticles; i++) {
        int idx = intDist(intGen);
        assert(idx >= 0 && idx < numFreeLocs);

        float x = freeXs->at(idx);
        float y = freeYs->at(idx);
        assert(isFree(static_cast<int>(x), static_cast<int>(y)));

        x *= resolution;
        y *= resolution;

        float theta = realDist(realGen);
        assert(theta >= -PI && theta < PI);

        float weight = 1.0 / numParticles;
        particles.push_back(Particle(x, y, theta, weight));
    }
}

void updateMotion(READING *prevOdometry, READING *currOdometry) {
    float xDiff = currOdometry->at(0) - prevOdometry->at(0);
    float yDiff = currOdometry->at(1) - prevOdometry->at(1);
    float thetaDiff = currOdometry->at(2) - prevOdometry->at(2);

    if (unlikely(std::abs(xDiff) + std::abs(yDiff) + std::abs(thetaDiff) <
                 1e-1)) {
        return;
    }

    float dRot1 = atan2(yDiff, xDiff) - prevOdometry->at(2);
    float dTrans = sqrt(xDiff * xDiff + yDiff * yDiff);
    float dRot2 = currOdometry->at(2) - prevOdometry->at(2) - dRot1;

    std::default_random_engine generator;

    float scaleH1 = alpha1 * dRot1 * dRot1 + alpha2 * dTrans * dTrans;
    float scaleTh = alpha3 * dTrans * dTrans + alpha4 * dRot1 * dRot1 +
                    alpha4 * dRot2 * dRot2;
    float scaleH2 = alpha1 * dRot2 * dRot2 + alpha2 * dTrans * dTrans;

    std::normal_distribution<float> h1Dist(0, scaleH1);
    std::normal_distribution<float> thDist(0, scaleTh);
    std::normal_distribution<float> h2Dist(0, scaleH2);

    float dRh1 = dRot1 - h1Dist(generator);
    float dTh = dTrans - thDist(generator);
    float dRh2 = dRot2 - h2Dist(generator);

#ifdef HIGHPERF
    // #pragma omp parallel for num_threads(8)
#endif
    for (int i = 0; i < numParticles; i++) {
        float thetaPrime = particles[i].theta + dRh1;

        particles[i].x += dTh * cos(thetaPrime);
        particles[i].y += dTh * sin(thetaPrime);
        particles[i].theta = particles[i].theta + dRh1 + dRh2;
    }
}

void updateSensor(READING *laserReading) {
    for (int i = 0; i < numParticles; i++) {
        float x = particles[i].x;
        float y = particles[i].y;
        float theta = particles[i].theta;

        std::vector<float> zStar;
        for (int d = 0; d < 180; d++) {
            zStar.push_back(rayCast(x, y, theta, static_cast<float>(d)));
        }

        float probability = 1.0;
        for (int eIdx = 0; eIdx < static_cast<int>(zStar.size()); eIdx++) {
            float expDist = zStar[eIdx];
            float readDist = laserReading->at(eIdx);
            probability *= calcProbability(readDist, expDist);
        }

        particles[i].w = probability;
    }
}

float calcProbability(float zkt, float zktStar) {
    float pRand = 0;
    if (zkt >= 0 && zkt < maxRange) {
        pRand = 1.0 / maxRange;
    }

    float pMax = 0;
    if (zkt >= maxRange) {
        pMax = 1.0;
    }

    float pShort = 0;
    if (likely(zkt >= 0 && zkt <= zktStar)) {
        float n = 1.0 / (1 - exp(-lambdaShort * zktStar));
        pShort = n * lambdaShort * exp(-lambdaShort * zkt);
    }

    float pHit = 0;
    if (likely(zkt >= 0 && zkt <= maxRange)) {
        pHit = exp(-0.5 * (zkt - zktStar) * (zkt - zktStar) /
                   (sigmaHit * sigmaHit));
        pHit /= sqrt(TWO_PI * sigmaHit * sigmaHit);
    }

    return zHit * pHit + zShort * pShort + zMax * pMax + zRand * pRand;
}

float rayCast(float x, float y, float theta, float degree) {
    float xRay = x + sensorOffset * cos(theta);
    float yRay = y + sensorOffset * sin(theta);

    float step = resolution;
    float xStep = step * cos(PI / 2 + theta - (degree * PI / 180));
    float yStep = step * sin(PI / 2 + theta - (degree * PI / 180));

    float dist = 0;

#ifdef TARTAN
    zsim_roi_end();
#endif

#ifndef HIGHPERF
    auto t0 = high_resolution_clock::now();
#endif

    while (true) {
        dist += step;
        xRay += xStep;
        yRay += yStep;

        int xIdx = static_cast<int>(xRay / resolution);
        int yIdx = static_cast<int>(yRay / resolution);

        if (dist >= maxRange || xIdx >= sizeX || yIdx >= sizeY || xIdx < 0 ||
            yIdx < 0) {
            break;
        }

        float occ = getProb(xIdx, yIdx);
        if (occ >= minProbability) {
            break;
        }
    }

#ifdef TARTAN
    zsim_roi_begin();

    // #pragma GCC diagnostic push
    // #pragma GCC diagnostic ignored "-Wall"
    auto packOperands = [&map](uint32_t offset, uint32_t orient) -> uint64_t {
        // Have to pack everything in one variable. Real-world implementation
        // doesn't have to be this complex!
        return (reinterpret_cast<uint64_t>(map) +
                (static_cast<uint32_t>(offset) << 32)) |
               orient;
    };
    // #pragma GCC diagnostic pop

    uint32_t mapOffset = xRay * sizeY + yRay;
    uint32_t vStep = 64 / sizeof(float);
    uint32_t numIters = static_cast<uint32_t>(dist / step) / vStep;
    for (uint32_t i = 0; i < numIters; i++) {
        ovec_load(packOperands((mapOffset + i * vStep) * sizeof(float),
                               yStep)); // It's volatile; won't be optimized out
    }

#endif

#ifndef HIGHPERF
    auto t1 = high_resolution_clock::now();
    rayCastingTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
#endif

    return dist;
}

void initializeResampler() {
    assert(numParticles > 0);
    resamDist =
        new std::uniform_real_distribution<float>(0, 1.0 / numParticles);
}

void resample() {
    float wSum = 0;
    for (int i = 0; i < numParticles; i++) {
        wSum += particles[i].w;
    }
    for (int i = 0; i < numParticles; i++) {
        particles[i].w /= wSum;
    }

    std::vector<Particle> newParticles;

    float M1 = 1.0 / numParticles;
    float r = (*resamDist)(resamRandGen);
    float c = particles[0].w;
    int i = 0;

    for (int m = 0; m < numParticles; m++) {
        float u = r + m * M1;
        while (u > c) {
            i++;
            c += particles[i].w;
        }
        newParticles.push_back(particles[i]);
    }

    particles.clear();
    std::copy(newParticles.begin(), newParticles.end(),
              std::back_inserter(particles));
}

std::vector<float> getBelief() {
    float xMean = 0, yMean = 0, thetaMean = 0;
    for (int i = 0; i < numParticles; i++) {
        Particle p = particles[i];
        xMean += p.x;
        yMean += p.y;
        thetaMean += p.theta;
    }

    xMean /= numParticles;
    yMean /= numParticles;
    thetaMean /= numParticles;

    return {xMean, yMean, thetaMean};
}
