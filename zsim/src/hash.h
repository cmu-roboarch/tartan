/** $lic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HASH_H_
#define HASH_H_

#include <stdint.h>
#include "galloc.h"
#include "log.h"
#include "zsim.h"

class HashFamily : public GlobAlloc {
    public:
        HashFamily() {}
        virtual ~HashFamily() {}

        virtual uint64_t hash(uint32_t id, uint64_t val) = 0;
};

class H3HashFamily : public HashFamily {
    private:
        const uint32_t numFuncs;
        uint32_t resShift;
        uint64_t* hMatrix;
    public:
        H3HashFamily(uint32_t numFunctions, uint32_t outputBits, uint64_t randSeed = 123132127);
        virtual ~H3HashFamily();
        uint64_t hash(uint32_t id, uint64_t val);
};

class SHA1HashFamily : public HashFamily {
    private:
        int numFuncs;
        int numPasses;

        //SHA1 is quite expensive and returns large blocks, so we use memoization and chunk the block to implement hash function families.
        uint64_t memoizedVal;
        uint32_t* memoizedHashes;
    public:
        explicit SHA1HashFamily(int numFunctions);
        uint64_t hash(uint32_t id, uint64_t val);
};

/* Used when we don't want hashing, just return the value */
class IdHashFamily : public HashFamily {
    public:
        inline uint64_t hash(uint32_t id, uint64_t val) {return val;}
};

class FcpHash : public HashFamily {
    public:
        inline uint64_t hash(uint32_t id, uint64_t val) {
            (void)id;
            // return (((val/1024) ^ (((val%1024) >> 8) & 0x3)) & 0xFFFFFFFFFFFFFFFCULL) | (((val/1024) & 0x3) ^ (((val%1024) >> 8) & 0x3));

            // return val ^ (((((val >> 11) & 1) ^ ((val >> 9) & 1)) << 11) | ((((val >> 10) & 1) ^ ((val >> 8) & 1)) << 10));

            return zinfo->lineSize == 32 ? ((((((((val >> 3) & 1) ^ ((val >> 5) & 1))? (val | (1ULL << 5)) : (val & ~(1ULL << 5))) >> 4) & 1) ^ ((((((val >> 3) & 1) ^ ((val >> 5) & 1))? (val | (1ULL << 5)) : (val & ~(1ULL << 5))) >> 6) & 1)) ? (((((val >> 3) & 1) ^ ((val >> 5) & 1))? (val | (1ULL << 5)) : (val & ~(1ULL << 5))) | (1ULL << 6)) : (((((val >> 3) & 1) ^ ((val >> 5) & 1))? (val | (1ULL << 5)) : (val & ~(1ULL << 5))) & ~(1ULL << 6))) : ((((((((val >> 2) & 1) ^ ((val >> 4) & 1))? (val | (1ULL << 4)) : (val & ~(1ULL << 4))) >> 3) & 1) ^ ((((((val >> 2) & 1) ^ ((val >> 4) & 1))? (val | (1ULL << 4)) : (val & ~(1ULL << 4))) >> 5) & 1)) ? (((((val >> 2) & 1) ^ ((val >> 4) & 1))? (val | (1ULL << 4)) : (val & ~(1ULL << 4))) | (1ULL << 5)) : (((((val >> 2) & 1) ^ ((val >> 4) & 1))? (val | (1ULL << 4)) : (val & ~(1ULL << 4))) & ~(1ULL << 5)));
        }

};

#endif  // HASH_H_
