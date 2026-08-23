#pragma once
#include <cstdint>

// Directed edge (from -> to) with the four attributes the spec asks for.
// `available` lets us simulate transitions going up/down without deleting
// them from the problem (Test Case 4: transition becomes unavailable).
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // per-edge safety score, as required by spec
    double reliability;
    bool available;

    Transition()
        : id(0), from(0), to(0), cost(0.0), safety(0.0),
          reliability(1.0), available(true) {}

    Transition(uint64_t id_, uint64_t from_, uint64_t to_, double cost_,
               double safety_, double reliability_, bool available_ = true)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_) {}
};
