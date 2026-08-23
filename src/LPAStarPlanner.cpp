#include "LPAStarPlanner.hpp"
#include <cmath>
#include <algorithm>

LPAStarPlanner::LPAStarPlanner(double beta, double gamma, double delta)
    : beta_(beta), gamma_(gamma), delta_(delta) {}

double LPAStarPlanner::g(uint64_t u) const {
    auto it = g_.find(u);
    return it == g_.end() ? INF : it->second;
}

double LPAStarPlanner::rhs(uint64_t u) const {
    auto it = rhs_.find(u);
    return it == rhs_.end() ? INF : it->second;
}

double LPAStarPlanner::heuristic(uint64_t u) const {
    // Euclidean distance to the current goal, scaled by beta so its units
    // roughly match the folded edge weight. This is only admissible when
    // edgeWeight is a pure (non-negative-scaled) cost, i.e. gamma_==0 and
    // delta_==0: with those terms active, edgeWeight can be pulled well
    // below beta*distance (even clamped to 0), so the raw-distance
    // heuristic would OVERESTIMATE remaining cost and break A*/LPA*'s
    // optimality guarantee. So whenever safety/reliability weighting is
    // in play we fall back to h=0, which is trivially admissible
    // (degrades gracefully to Dijkstra-style search -- still correct,
    // just explores more states). This trade-off is worth calling out
    // explicitly in the report's heuristic-function section.
    if (gamma_ != 0.0 || delta_ != 0.0) return 0.0;
    auto uit = problem_->states.find(u);
    auto git = problem_->states.find(goal_);
    if (uit == problem_->states.end() || git == problem_->states.end()) return 0.0;
    return beta_ * uit->second.distanceTo(git->second);
}

double LPAStarPlanner::edgeWeight(const Transition& t) const {
    double sd = problem_->minDistanceToBadState(t.to);
    if (std::isinf(sd)) sd = SAFETY_CAP;
    else sd = std::min(sd, SAFETY_CAP);
    double w = beta_ * t.cost - gamma_ * sd - delta_ * t.reliability;
    return std::max(w, 0.0); // keep non-negative: LPA* correctness assumes non-negative edge weights
}

LPAStarPlanner::Key LPAStarPlanner::calculateKey(uint64_t u) const {
    double m = std::min(g(u), rhs(u));
    return Key{m + heuristic(u), m};
}

void LPAStarPlanner::removeFromQueue(uint64_t u) {
    auto it = inQueueKey_.find(u);
    if (it == inQueueKey_.end()) return;
    queue_.erase({it->second, u});
    inQueueKey_.erase(it);
}

void LPAStarPlanner::insertOrUpdate(uint64_t u) {
    removeFromQueue(u);
    Key k = calculateKey(u);
    queue_.insert({k, u});
    inQueueKey_[u] = k;
}

void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != start_) {
        double best = INF;
        for (const Transition* t : problem_->validIncoming(u)) {
            double cand = g(t->from) + edgeWeight(*t);
            if (cand < best) best = cand;
        }
        rhs_[u] = best;
    }
    removeFromQueue(u);
    if (g(u) != rhs(u)) insertOrUpdate(u);
}

void LPAStarPlanner::computeShortestPath() {
    while (!queue_.empty() &&
           (queue_.begin()->first < calculateKey(goal_) || rhs(goal_) != g(goal_))) {
        auto it = queue_.begin();
        Key kOld = it->first;
        uint64_t u = it->second;
        ++statesExplored_;

        Key kNew = calculateKey(u);
        if (kOld < kNew) {
            queue_.erase(it);
            inQueueKey_[u] = kNew;
            queue_.insert({kNew, u});
        } else if (g(u) > rhs(u)) {
            g_[u] = rhs(u);
            removeFromQueue(u);
            for (const Transition* t : problem_->validOutgoing(u)) updateVertex(t->to);
        } else {
            g_[u] = INF;
            updateVertex(u);
            for (const Transition* t : problem_->validOutgoing(u)) updateVertex(t->to);
        }
    }
}

PlanningResult LPAStarPlanner::extractPath() const {
    PlanningResult result;
    result.statesExplored = statesExplored_;

    if (std::isinf(g(goal_))) {
        result.success = false;
        return result;
    }

    // Walk backward from goal to start, at each step picking the
    // predecessor edge that actually realizes rhs(u) = g(pred) + w(pred,u).
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    uint64_t u = goal_;
    statePath.push_back(u);
    std::unordered_map<uint64_t, bool> visited;
    visited[u] = true;

    while (u != start_) {
        const Transition* best = nullptr;
        double bestVal = INF;
        for (const Transition* t : problem_->validIncoming(u)) {
            double cand = g(t->from) + edgeWeight(*t);
            if (cand < bestVal) {
                bestVal = cand;
                best = t;
            }
        }
        if (!best || visited.count(best->from)) {
            // No consistent predecessor found (shouldn't happen if g(goal) is finite),
            // or we detected a cycle -- bail out safely.
            result.success = false;
            return result;
        }
        transitionPath.push_back(best->id);
        u = best->from;
        statePath.push_back(u);
        visited[u] = true;
    }

    std::reverse(statePath.begin(), statePath.end());
    std::reverse(transitionPath.begin(), transitionPath.end());

    double totalCost = 0.0;
    double minSafety = INF;
    for (uint64_t tid : transitionPath) totalCost += problem_->transitions.at(tid).cost;
    for (uint64_t sid : statePath) minSafety = std::min(minSafety, problem_->minDistanceToBadState(sid));

    result.success = true;
    result.statePath = std::move(statePath);
    result.transitionPath = std::move(transitionPath);
    result.totalCost = totalCost;
    result.safetyScore = minSafety;
    return result;
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    problem_ = &problem;
    start_ = problem.initialState;
    goal_ = problem.goalState;

    g_.clear();
    rhs_.clear();
    queue_.clear();
    inQueueKey_.clear();
    statesExplored_ = 0;

    rhs_[start_] = 0.0;
    insertOrUpdate(start_);

    computeShortestPath();
    return extractPath();
}

void LPAStarPlanner::onTransitionChanged(uint64_t transitionId) {
    auto it = problem_->transitions.find(transitionId);
    if (it == problem_->transitions.end()) return;
    updateVertex(it->second.to);
}

void LPAStarPlanner::onStateBadnessChanged(uint64_t stateId) {
    updateVertex(stateId);
    auto oit = problem_->outgoing.find(stateId);
    if (oit != problem_->outgoing.end())
        for (uint64_t tid : oit->second) updateVertex(problem_->transitions.at(tid).to);
    auto iit = problem_->incoming.find(stateId);
    if (iit != problem_->incoming.end())
        for (uint64_t tid : iit->second) updateVertex(problem_->transitions.at(tid).from);
}

void LPAStarPlanner::onGoalChanged(uint64_t newGoalState) {
    // g/rhs are relative to start_, so they stay valid -- we only need to
    // switch which vertex ComputeShortestPath is aiming for.
    goal_ = newGoalState;
}

PlanningResult LPAStarPlanner::replan() {
    // Reset the counter so PlanningResult::statesExplored reflects the
    // cost of THIS replan only, not the cumulative total since the last
    // full plan() call -- that's the number that's actually comparable
    // to a fresh plan() on the same (modified) problem.
    statesExplored_ = 0;
    computeShortestPath();
    return extractPath();
}