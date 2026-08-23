#pragma once
#include <unordered_map>
#include <set>
#include <limits>
#include "Planner.hpp"

// LPA* (Lifelong Planning A*) planner.
//
// Why LPA* fits this assignment: g(u) and rhs(u) are computed relative to
// a FIXED start state, not the goal. That means when the environment
// changes (edge cost/availability changes, bad states change, or even the
// goal changes) most of g/rhs is still correct — we only need to touch the
// handful of vertices actually affected, then re-run ComputeShortestPath,
// which only re-expands the locally inconsistent frontier. That directly
// covers Test Cases 4-6 (dynamic transition, goal update, transition
// addition) with one algorithm.
//
// Multi-criteria objective: the spec's Score(P) = aG - bC + gD + dR is a
// weighted sum. We fold it into a single scalar edge weight so ordinary
// shortest-path machinery applies:
//     edgeWeight(u->v) = beta*cost(u,v) - gamma*safetyDist(v) - delta*reliability(u,v)
// (goal completion G is handled separately: G=1 iff a path exists at all).
// Weights beta/gamma/delta are constructor parameters so you can tune the
// cost/safety/reliability trade-off (Test Case 3) without touching the
// algorithm. Bad states are a HARD constraint (never traversed at all,
// see PlanningProblem::validOutgoing), not part of the soft score.
class LPAStarPlanner : public Planner {
public:
    // beta: weight on raw transition cost (minimize)
    // gamma: weight on safety distance to nearest bad state (maximize -> subtracted)
    // delta: weight on reliability (maximize -> subtracted)
    explicit LPAStarPlanner(double beta = 1.0, double gamma = 0.0, double delta = 0.0);

    // Full (re)initialization: clears g/rhs and solves from scratch.
    // Use this the first time, or if you don't care about incremental reuse.
    PlanningResult plan(const PlanningProblem& problem) override;

    // --- Incremental replanning API ---
    // Call one or more of these after mutating the PlanningProblem that
    // was passed to the last plan()/replan() call, then call replan().
    // These only mark vertices as "locally inconsistent"; the actual
    // recomputation is deferred to replan().
    void onTransitionChanged(uint64_t transitionId);
    void onStateBadnessChanged(uint64_t stateId);
    void onGoalChanged(uint64_t newGoalState);

    // Re-run ComputeShortestPath reusing existing g/rhs values, then
    // extract the path to the (possibly new) goal.
    PlanningResult replan();

private:
    struct Key {
        double k1, k2;
        bool operator<(const Key& o) const {
            if (k1 != o.k1) return k1 < o.k1;
            return k2 < o.k2;
        }
    };

    const PlanningProblem* problem_ = nullptr;
    uint64_t start_ = 0;
    uint64_t goal_ = 0;
    double beta_, gamma_, delta_;

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    std::unordered_map<uint64_t, Key> inQueueKey_; // vertices currently in the queue
    std::set<std::pair<Key, uint64_t>> queue_;

    size_t statesExplored_ = 0;

    static constexpr double INF = std::numeric_limits<double>::infinity();
    static constexpr double SAFETY_CAP = 1000.0; // caps the "no bad states -> infinite safety" case

    double g(uint64_t u) const;
    double rhs(uint64_t u) const;
    double heuristic(uint64_t u) const;               // Euclidean distance to current goal
    double edgeWeight(const Transition& t) const;      // folded cost/safety/reliability

    Key calculateKey(uint64_t u) const;
    void insertOrUpdate(uint64_t u);
    void removeFromQueue(uint64_t u);
    void updateVertex(uint64_t u);
    void computeShortestPath();
    PlanningResult extractPath() const;
};