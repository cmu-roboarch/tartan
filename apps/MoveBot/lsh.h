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

#include "vcl/vectorclass.h"
#include <iostream>
#include <limits>
#include <random>
#include <unordered_map>
#include <vector>

class LSHTable {
  public:
    LSHTable() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<float> dis(0, 1);

        float *pv = new float[5];
        for (int i = 0; i < 5; ++i) {
            pv[i] = dis(gen);
            projectionArray[i] = dis(gen);
        }
        this->projectionVector.store(pv);
    }

    void add(const std::vector<float> &vec, int idx) {
        float projectedValue = project(vec);
        int hashValue = static_cast<int>(projectedValue);
        table[hashValue].push_back(idx);
        vectors.push_back(vec);
    }

    int findNearest(const std::vector<float> &q) {
        float projectedValue = project(q);
        int hashValue = static_cast<int>(projectedValue);
        if (table.find(hashValue) == table.end()) return 0;

#ifdef __AVX512F__
        Vec8f v1(q[0], q[1], q[2], q[3], q[4], 0, 0, 0);
#endif

        float minDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for (int idx : table[hashValue]) {

#ifdef __AVX512F__
            Vec8f v2(vectors[idx][0], vectors[idx][1], vectors[idx][2],
                     vectors[idx][3], vectors[idx][4], 0, 0, 0);
            Vec8f d = (v1 - v2) * (v1 - v2);
            float bestDist = horizontal_min(d);

            if (bestDist >= minDist) continue; // Likely case

            minDist = bestDist;
            bestIdx = horizontal_find_first(d == bestDist);
#else

            float dist = 0.0f;
            for (size_t i = 0; i < 5; ++i) {
                float diff = vectors[idx][i] - q[i];
                dist += diff * diff;
            }

            if (dist < minDist) {
                minDist = dist;
                bestIdx = idx;
            }
        }
#endif
            return bestIdx;
        }

      private:
        std::unordered_map<int, std::vector<int>> table;
        std::vector<std::vector<float>> vectors;
        Vec8f projectionVector;
        float projectionArray[5];

        float project(const std::vector<float> &vec) const {
#ifdef __AVX512F__
            // Terrible performance if the architecture doesn't support AVX-512
            Vec8f v(vec[0], vec[1], vec[2], vec[3], vec[4], 0, 0, 0);
            v = v * projectionVector;
            return horizontal_add(v);
#else
        return vec[0] * projectionArray[0] + vec[1] * projectionArray[1] +
               vec[2] * projectionArray[2] + vec[3] * projectionArray[3] +
               vec[4] * projectionArray[4];
#endif
        }
    };
