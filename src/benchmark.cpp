// Benchmark harness for the "Experimental Results" deliverable.
// Generates grid-shaped graphs of increasing size, measures the metrics
// the assignment spec asks for, and writes them to results.csv.
//
// Build/run: `make bench && ./benchmark`

#include <iostream>
#include <fstream>
#include <chrono>
#include <random>
#include <vector>
#include <string>
#include <cmath>
#include <filesystem>
#include <sys/resource.h>
#include "PlanningProblem.hpp"
#include "LPAStarPlanner.hpp"

using Clock = std::chrono::high_resolution_clock;

static double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

static long peakMemoryKb() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss; // KB on Linux
}

// Builds a gridDim x gridDim grid graph: 4-directional edges, random
// costs/reliability, ~badFraction of interior cells marked bad (start,
// goal, and their immediate neighbors are kept clear so a path always
// exists). Goal is the opposite corner from start.
static PlanningProblem buildGridProblem(int gridDim, double badFraction, unsigned seed) {
    PlanningProblem p;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> costDist(1.0, 5.0);
    std::uniform_real_distribution<double> reliabilityDist(0.7, 1.0);
    std::uniform_real_distribution<double> badDist(0.0, 1.0);

    auto idOf = [gridDim](int r, int c) -> uint64_t { return static_cast<uint64_t>(r * gridDim + c) + 1; };

    for (int r = 0; r < gridDim; ++r) {
        for (int c = 0; c < gridDim; ++c) {
            p.addState(State(idOf(r, c), {static_cast<double>(r), static_cast<double>(c)}));
        }
    }

    uint64_t start = idOf(0, 0);
    uint64_t goal = idOf(gridDim - 1, gridDim - 1);

    uint64_t tid = 1;
    for (int r = 0; r < gridDim; ++r) {
        for (int c = 0; c < gridDim; ++c) {
            uint64_t u = idOf(r, c);
            if (c + 1 < gridDim) {
                uint64_t v = idOf(r, c + 1);
                p.addTransition(Transition(tid++, u, v, costDist(rng), 1.0, reliabilityDist(rng)));
                p.addTransition(Transition(tid++, v, u, costDist(rng), 1.0, reliabilityDist(rng)));
            }
            if (r + 1 < gridDim) {
                uint64_t v = idOf(r + 1, c);
                p.addTransition(Transition(tid++, u, v, costDist(rng), 1.0, reliabilityDist(rng)));
                p.addTransition(Transition(tid++, v, u, costDist(rng), 1.0, reliabilityDist(rng)));
            }
        }
    }

    for (int r = 0; r < gridDim; ++r) {
        for (int c = 0; c < gridDim; ++c) {
            uint64_t s = idOf(r, c);
            if (s == start || s == goal) continue;
            if (badDist(rng) < badFraction) p.setBadState(s, true);
        }
    }

    p.initialState = start;
    p.goalState = goal;
    return p;
}

struct Row {
    int gridDim, n;
    std::string mode;
    double planningTimeMs;
    size_t statesExplored;
    bool success;
    double totalCost;
    double safetyScore;
    long memoryKb;
};

static void writeCsv(std::ofstream& out, const Row& r) {
    out << r.gridDim << "," << r.n << "," << r.mode << ","
        << r.planningTimeMs << "," << r.statesExplored << ","
        << (r.success ? 1 : 0) << "," << r.totalCost << ","
        << (std::isinf(r.safetyScore) ? -1.0 : r.safetyScore) << ","
        << r.memoryKb << "\n";
}

int main() {
    std::filesystem::create_directories("results"); // no-op if it already exists
    std::ofstream out("results/results.csv");
    out << "gridDim,n,mode,planningTimeMs,statesExplored,success,totalCost,safetyScore,memoryKb\n";

    std::vector<int> gridSizes = {10, 20, 30, 40, 50, 60, 70};

    for (int dim : gridSizes) {
        PlanningProblem problem = buildGridProblem(dim, 0.08, /*seed=*/dim * 7919u);
        int n = dim * dim;

        // 1. Fresh plan from scratch.
        LPAStarPlanner planner;
        auto t0 = Clock::now();
        PlanningResult r1 = planner.plan(problem);
        auto t1 = Clock::now();
        Row row1{dim, n, "initial_plan", elapsedMs(t0, t1), r1.statesExplored,
                  r1.success, r1.totalCost, r1.safetyScore, peakMemoryKb()};
        writeCsv(out, row1);

        if (!r1.success) {
            std::cerr << "grid " << dim << ": initial plan failed, skipping replan tests\n";
            continue;
        }

        // 2. Cut one transition on the found path, then INCREMENTAL replan.
        uint64_t cutId = r1.transitionPath[r1.transitionPath.size() / 2];
        problem.setTransitionAvailable(cutId, false);

        auto t2 = Clock::now();
        planner.onTransitionChanged(cutId);
        PlanningResult r2 = planner.replan();
        auto t3 = Clock::now();
        Row row2{dim, n, "incremental_replan", elapsedMs(t2, t3), r2.statesExplored,
                  r2.success, r2.totalCost, r2.safetyScore, peakMemoryKb()};
        writeCsv(out, row2);

        // 3. Same modified problem, but a completely FRESH planner/search,
        //    for a fair from-scratch comparison against the incremental one.
        LPAStarPlanner freshPlanner;
        auto t4 = Clock::now();
        PlanningResult r3 = freshPlanner.plan(problem);
        auto t5 = Clock::now();
        Row row3{dim, n, "fresh_replan", elapsedMs(t4, t5), r3.statesExplored,
                  r3.success, r3.totalCost, r3.safetyScore, peakMemoryKb()};
        writeCsv(out, row3);

        std::cout << "grid " << dim << "x" << dim << " (n=" << n << "): "
                  << "initial explored=" << r1.statesExplored
                  << ", incremental replan explored=" << r2.statesExplored
                  << ", fresh replan explored=" << r3.statesExplored << "\n";
    }

    std::cout << "\nWrote results/results.csv\n";
    return 0;
}