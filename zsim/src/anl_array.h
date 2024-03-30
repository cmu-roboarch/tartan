#ifndef ANL_ARRAYS_H_
#define ANL_ARRAYS_H_

#include <stdint.h>
#include "bithacks.h"

template <uint64_t S>
class AnlArray {
    public:
        AnlArray() {}

        void terminateRegion(uint64_t region) {
            for (uint64_t w = 0; w < S; w++) {
                if (array[w].region == region) {
                    array[w].copyDegree();
                }
            }
        }

        bool isInArray(uint64_t pc, uint64_t region) {
            return findPos(pc, region) != S;
        }

        void allocate(uint64_t pc, uint64_t region) {
            assert(!isInArray(pc, region));

            uint64_t idx = findVictim();
            assert(idx < S);
            array[idx] = Entry(pc, region, 0, 0);
        }

        uint64_t generatePrefetches(uint64_t pc, uint64_t region) {
            uint64_t idx = findPos(pc, region);
            assert(idx < S);

            uint64_t lastDegree = array[idx].getLastDegree();
            array[idx].incrementCurrentDegree();
            array[idx].resetLastDegree();

            return lastDegree;
        }

    private:
        struct Entry {
            bool valid;
            uint64_t pc;
            uint64_t region;
            uint64_t cd;
            uint64_t ld;

            Entry() : valid(false), pc(0L), region(0L), cd(0L), ld(0L) {}
            Entry(uint64_t _pc, uint64_t _region, uint64_t _cd, uint64_t _ld) : valid(true), pc(_pc), region(_region), cd(_cd), ld(_ld) {}

            uint64_t getPriority() const { return MAX(cd, ld); }
            void copyDegree() { ld = cd; }
            void incrementCurrentDegree() { cd = MIN(cd + 1, (uint64_t)7L); }
            void resetLastDegree() { ld = 0; };
            uint64_t getLastDegree() const { return ld; };
        };

        Entry array[S];

        uint64_t findPos(uint64_t pc, uint64_t region) const {
            uint64_t w;
            for (w = 0; w < S; w++) {
                if (array[w].pc == pc && array[w].region == region) {
                    break;
                }
            }

            return w;
        }

        uint64_t findVictim() const {
            uint64_t bestIdx = 0;
            uint64_t bestPriority = 0;

            for (uint64_t w = 0; w < S; w++) {
                if (!array[w].valid) return w;

                uint64_t p = array[w].getPriority();
                if (p < bestPriority) {
                    bestPriority = p;
                    bestIdx = w;
                }
            }

            return bestIdx;
        }
};

#endif  // ANL_ARRAYS_H_
