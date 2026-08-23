#pragma once
#include <cstdint>
#include <vector>
#include <limits>

class PlanningResult {
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;   // min distance to any bad state along the path

    // Extra stats useful for the "Students should evaluate" section of
    // the assignment (explored states, planning time, etc). Not in the
    // spec's suggested interface but harmless additions.
    size_t statesExplored = 0;
    double planningTimeMs = 0.0;

    PlanningResult()
        : success(false), totalCost(0.0),
          safetyScore(std::numeric_limits<double>::infinity()) {}
};
