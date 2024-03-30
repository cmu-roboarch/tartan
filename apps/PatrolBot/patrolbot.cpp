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
#include "ekf.h"
#include "log.h"
#include "pp.h"
#include "slam.h"
#include "zsim_hooks.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

std::vector<std::string> classLabels;
cv::dnn::Net model;
float confThresh;

double totalExecTime = 0, inferenceTime = 0;
void evaluate(std::string imageFile, float scale);

std::vector<Point> readPath(const char *fileName) {
    std::vector<Point> path;

    std::ifstream pathFile;
    pathFile.open(fileName);
    assert(pathFile.good());

    std::string line;
    while (std::getline(pathFile, line)) {
        double x, y;
        std::stringstream ss(line);
        ss >> x >> y;
        path.push_back(Point(x, y));
    }

    pathFile.close();

    return path;
}

std::vector<std::vector<double>> readSensorLog(std::string inputLogFile) {
    std::ifstream logFile;
    logFile.open(inputLogFile);
    assert(logFile.good());

    std::vector<std::vector<double>> log;

    std::string line;
    while (std::getline(logFile, line)) {
        std::stringstream entrySS(line);

        std::vector<double> entry;
        double e;
        while (entrySS >> e) {
            entry.push_back(e);
        }

        log.push_back(entry);
    }
    logFile.close();

    return log;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputLabelsArg(parser, "labels", "",
                                      "Input file containing class names");
    KVArg<std::string> inputModelArg(parser, "model", "",
                                     "Pre-trained model containing weights");
    KVArg<std::string> inputCfgArg(parser, "cfg", "",
                                   "Model configuration file");
    KVArg<std::string> inputImgDirArg(parser, "imgdir", "", "Input image file");
    KVArg<float> imgScaleArg(parser, "scale", "", "Image scale factor");
    KVArg<float> confThreshArg(
        parser, "confidence", "",
        "The minimum confidence threshold for drawing bounding boxes");
    KVArg<std::string> pathFileArg(parser, "path", "", "Input path file");
    KVArg<std::string> sensorLogArg(parser, "log", "", "Input sensor log file");
    KVArg<std::string> outputArg(parser, "output", "", "Output log file");

    if (!parser.parse()) assert(false);

    assert_msg(inputLabelsArg.found(), "Input class file is not provided");
    assert_msg(inputModelArg.found(), "Input model file is not provided");
    assert_msg(inputCfgArg.found(), "Input configuration file is not provided");
    assert_msg(inputImgDirArg.found(), "Input image directory is not provided");
    assert_msg(pathFileArg.found(), "Input path file is not provided");
    assert_msg(sensorLogArg.found(), "Input sensor log file is not provided");

    cv::setNumThreads(4);

    // Mobile-Net SSD
    std::string inputLabelsFile = inputLabelsArg.value();
    std::string inputModelFile = inputModelArg.value();
    std::string inputCfgFile = inputCfgArg.value();
    std::string inputImgDir = inputImgDirArg.value();
    float imgScale = imgScaleArg.found() ? imgScaleArg.value() : 1.0;
    confThresh = confThreshArg.found() ? confThreshArg.value() : 0.4;
    std::ifstream inFile(inputLabelsFile);
    std::string line;
    while (std::getline(inFile, line)) {
        classLabels.push_back(line);
    }

    model = cv::dnn::readNet(inputModelFile, inputCfgFile, "TensorFlow");

    assert(confThresh >= 0 && confThresh <= 1);
    assert(imgScale >= 0 && imgScale <= 1);

    // SLAM
    std::string inputSensorLogFile = sensorLogArg.value();
    double sigX2 = 0.25 * 0.25;
    double sigY2 = 0.1 * 0.1;
    double sigAlpha2 = 0.1 * 0.1;
    double sigBeta2 = 0.01 * 0.01;
    double sigR2 = 0.08 * 0.08;

    std::vector<std::vector<double>> inputSensorLog =
        readSensorLog(inputSensorLogFile);
    std::vector<double> initialMeasurement = inputSensorLog.front();
    int numLandmarks = static_cast<int>(initialMeasurement.size() / 2);
    SLAM *ekfslam =
        new EKFSLAM(numLandmarks, sigX2, sigY2, sigAlpha2, sigBeta2, sigR2);
    size_t sensorLogIdx = 1;

    // Pure Pursuit
    const char *pathFile = pathFileArg.value().c_str();
    auto path = readPath(pathFile);
    PurePursuit *tracker = new PurePursuit(path, 2.0);

    // Logging
    std::string outputFile =
        outputArg.found() ? outputArg.value() : "/dev/null";
    std::vector<std::vector<double>> outputLog;

    try {
        for (const auto &entry : fs::directory_iterator(inputImgDir)) {
            if (entry.is_regular_file()) {
                auto filePath = entry.path();
                if (filePath.extension() == ".jpg" ||
                    filePath.extension() == ".png") {

                    auto t0 = high_resolution_clock::now();

                    // >>> ROI
                    zsim_roi_begin();
#ifdef HIGHPERF
                    std::thread evaluationThread(evaluate, filePath.string(),
                                                 imgScale);
                    // Comment it out to dodge ZSim's wacky antics in
                    // fast-forwarding evaluate(filePath.string(), imgScale);
#else
                    evaluate(filePath.string(), imgScale);
#endif

                    if (sensorLogIdx < inputSensorLog.size()) {
                        std::vector l = inputSensorLog[sensorLogIdx];
                        if (l.size() == 2) {
                            ekfslam->motionUpdate(l[0], l[1], l[2]);
                        } else if (l.size() ==
                                   2 * static_cast<size_t>(numLandmarks)) {
                            ekfslam->measurementUpdate(l);
                        } else {
                            panic("Illegal input");
                        }

                        std::vector state = ekfslam->getStatus();
                        Point lookahead = tracker->getLookAheadPoint(
                            Point(state[0], state[1]));
                        state.push_back(lookahead.x);
                        state.push_back(lookahead.x);

                        outputLog.push_back(state);
                        sensorLogIdx++;
                    }

#ifdef HIGHPERF
                    if (evaluationThread.joinable()) {
                        evaluationThread.join();
                    }
#endif

                    zsim_roi_end();
                    // <<< ROI

                    auto t2 = high_resolution_clock::now();
                    totalExecTime +=
                        duration_cast<nanoseconds>(t2 - t0).count() * 1e-9;

                    break; // One image enough to get stable simulation results
                }
            }
        }
    } catch (const fs::filesystem_error &e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "General error: " << e.what() << std::endl;
    }

#ifndef HIGHPERF
    std::cout << "totalExecTime: " << totalExecTime << std::endl;
    std::cout << "inferenceTime: " << inferenceTime << std::endl;
    std::cout << "inferenceTimeContribution: " << inferenceTime / totalExecTime
              << std::endl;
#endif

    // Write the output log
    std::ofstream outLogFile;
    outLogFile.open(outputFile);
    for (auto l : outputLog) {
        for (auto e : l) {
            outLogFile << std::setprecision(4) << e << " ";
        }
        outLogFile << std::endl;
    }
    outLogFile.close();

    delete ekfslam;

    return 0;
}

void evaluate(std::string imageFile, float scale) {
    assert(scale > 0 && scale <= 1);

    cv::Mat image = cv::imread(imageFile);
    if (image.empty()) {
        return;
    }

    cv::Mat blob =
        cv::dnn::blobFromImage(image, scale, cv::Size(300, 300),
                               cv::Scalar(127.5, 127.5, 127.5), true, false);

#ifndef HIGHPERF
    auto t0 = high_resolution_clock::now();
#endif

#ifdef TARTAN
    zsim_roi_end();
#endif

    model.setInput(blob);
    cv::Mat output = model.forward();
    cv::Mat detectionMat(output.size[2], output.size[3], CV_32F,
                         output.ptr<float>());

    for (int i = 0; i < detectionMat.rows; i++) {
        if (detectionMat.at<float>(i, 2) >= confThresh) {
            int classId = detectionMat.at<float>(i, 1);

            if (classId == 49 /*Knife*/) {
                std::cout << "Alert! Detected: " << classLabels[classId - 1]
                          << " on campus" << std::endl;
            }
        }
    }

#ifdef TARTAN
    zsim_roi_begin();
#endif

#ifndef HIGHPERF
    auto t1 = high_resolution_clock::now();
    inferenceTime += duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
#endif
}
