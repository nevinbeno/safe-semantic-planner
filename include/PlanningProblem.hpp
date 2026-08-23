#pragma once
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include "State.hpp"
#include "Transition.hpp"

// PlanningProblem owns all states/transitions and the bad-state set, and
// exposes O(1)-ish successor/predecessor queries via adjacency maps.
// This is the piece that has to support "the environment changes
// periodically" (Dynamic Environment section of the spec): goal changes,
// bad states changing, transitions added/removed/(un)availability toggled.
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::unordered_set<uint64_t> badStates;

    std::unordered_map<uint64_t, State> states;
    std::unordered_map<uint64_t, Transition> transitions; // keyed by transition id

    // adjacency: stateId -> list of outgoing/incoming transition ids
    std::unordered_map<uint64_t, std::vector<uint64_t>> outgoing;
    std::unordered_map<uint64_t, std::vector<uint64_t>> incoming;

    void addState(const State& s) {
        states[s.id] = s;
        outgoing[s.id]; // ensure entry exists
        incoming[s.id];
    }

    void addTransition(const Transition& t) {
        transitions[t.id] = t;
        outgoing[t.from].push_back(t.id);
        incoming[t.to].push_back(t.id);
    }

    void removeTransition(uint64_t transitionId) {
        auto it = transitions.find(transitionId);
        if (it == transitions.end()) return;
        uint64_t from = it->second.from, to = it->second.to;
        auto eraseFrom = [](std::vector<uint64_t>& v, uint64_t id) {
            v.erase(std::remove(v.begin(), v.end(), id), v.end());
        };
        eraseFrom(outgoing[from], transitionId);
        eraseFrom(incoming[to], transitionId);
        transitions.erase(it);
    }

    void setTransitionAvailable(uint64_t transitionId, bool available) {
        auto it = transitions.find(transitionId);
        if (it != transitions.end()) it->second.available = available;
    }

    void setBadState(uint64_t stateId, bool isBad) {
        if (isBad) badStates.insert(stateId);
        else badStates.erase(stateId);
    }

    bool isBad(uint64_t stateId) const {
        return badStates.find(stateId) != badStates.end();
    }

    // Valid successors of u: transitions that are available and whose
    // target is not a bad state (bad states are hard-excluded, never
    // "visited" per Optimization Objective #2).
    std::vector<const Transition*> validOutgoing(uint64_t u) const {
        std::vector<const Transition*> result;
        auto it = outgoing.find(u);
        if (it == outgoing.end()) return result;
        for (uint64_t tid : it->second) {
            const Transition& t = transitions.at(tid);
            if (t.available && !isBad(t.to)) result.push_back(&t);
        }
        return result;
    }

    std::vector<const Transition*> validIncoming(uint64_t u) const {
        std::vector<const Transition*> result;
        auto it = incoming.find(u);
        if (it == incoming.end()) return result;
        for (uint64_t tid : it->second) {
            const Transition& t = transitions.at(tid);
            if (t.available && !isBad(t.from)) result.push_back(&t);
        }
        return result;
    }

    // Minimum Euclidean distance from state u to the nearest bad state.
    // Used for the "safety distance" term D in the objective function.
    // If there are no bad states, returns +infinity (maximally safe).
    double minDistanceToBadState(uint64_t u) const {
        if (badStates.empty()) return std::numeric_limits<double>::infinity();
        const State& s = states.at(u);
        double best = std::numeric_limits<double>::infinity();
        for (uint64_t b : badStates) {
            auto it = states.find(b);
            if (it == states.end()) continue;
            best = std::min(best, s.distanceTo(it->second));
        }
        return best;
    }
};
