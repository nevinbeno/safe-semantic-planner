#pragma once
#include <cstdint>
#include <vector>
#include <cmath>

// A single state s_i = (x_1, ..., x_d) embedded in R^d.
// We store it as a flat vector<double> so the planner works for any
// dimensionality d without recompiling (spec just says "finite Cartesian
// state space", it doesn't fix d).
class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0) {}
    State(uint64_t id_, std::vector<double> embedding_)
        : id(id_), embedding(std::move(embedding_)) {}

    // Euclidean distance to another state. Used both for the heuristic
    // (distance to goal) and for the safety computation (distance to
    // nearest bad state).
    double distanceTo(const State& other) const {
        double sum = 0.0;
        size_t n = std::min(embedding.size(), other.embedding.size());
        for (size_t i = 0; i < n; ++i) {
            double diff = embedding[i] - other.embedding[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }
};
