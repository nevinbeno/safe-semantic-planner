#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <unordered_set>
#include "PlanningProblem.hpp"
#include "LPAStarPlanner.hpp"

// Dumps a problem + the chosen path to two CSVs so Python/matplotlib can
// draw a diagram of it (nodes, edges, bad states, highlighted path).
// Only used for the small illustrative test-case graphs, not benchmarks.
static void exportForPlot(const std::string& name, const PlanningProblem& p,
                           const PlanningResult& r) {
    std::filesystem::create_directories("results/plots"); // no-op if it already exists
    std::unordered_set<uint64_t> onPath(r.statePath.begin(), r.statePath.end());
    std::ofstream nodes("results/plots/" + name + "_nodes.csv");
    nodes << "id,x,y,bad,role\n";
    for (const auto& [id, s] : p.states) {
        double x = s.embedding.size() > 0 ? s.embedding[0] : 0.0;
        double y = s.embedding.size() > 1 ? s.embedding[1] : 0.0;
        std::string role = "normal";
        if (id == p.initialState) role = "start";
        else if (id == p.goalState) role = "goal";
        nodes << id << "," << x << "," << y << "," << (p.isBad(id) ? 1 : 0) << "," << role << "\n";
    }

    std::ofstream edges("results/plots/" + name + "_edges.csv");
    edges << "from,to,onPath\n";
    for (const auto& [tid, t] : p.transitions) {
        (void)tid;
        bool edgeOnPath = false;
        for (size_t i = 0; i + 1 < r.statePath.size(); ++i)
            if (r.statePath[i] == t.from && r.statePath[i + 1] == t.to) edgeOnPath = true;
        edges << t.from << "," << t.to << "," << (edgeOnPath ? 1 : 0) << "\n";
    }
}

static void printResult(const std::string& label, const PlanningResult& r) {
    std::cout << "-- " << label << " --\n";
    if (!r.success) {
        std::cout << "  FAILED: no path found\n\n";
        return;
    }
    std::cout << "  path: ";
    for (size_t i = 0; i < r.statePath.size(); ++i) {
        std::cout << r.statePath[i];
        if (i + 1 < r.statePath.size()) std::cout << " -> ";
    }
    std::cout << "\n  totalCost: " << r.totalCost
              << "  safetyScore(min dist to bad state): " << r.safetyScore
              << "  statesExplored: " << r.statesExplored << "\n\n";
}

// ---------------------------------------------------------------------
// Test Case 1: Basic Reachability   S -> A -> B -> G
// ---------------------------------------------------------------------
void testCase1() {
    PlanningProblem p;
    p.addState(State(1, {0, 0}));  // S
    p.addState(State(2, {1, 0}));  // A
    p.addState(State(3, {2, 0}));  // B
    p.addState(State(4, {3, 0}));  // G
    p.addTransition(Transition(1, 1, 2, 1.0, 1.0, 1.0));
    p.addTransition(Transition(2, 2, 3, 1.0, 1.0, 1.0));
    p.addTransition(Transition(3, 3, 4, 1.0, 1.0, 1.0));
    p.initialState = 1;
    p.goalState = 4;

    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    printResult("Test Case 1: Basic Reachability", r);
    exportForPlot("tc1", p, r);
}

// ---------------------------------------------------------------------
// Test Case 2: Bad State Avoidance
//   S -> A -> X -> G   (X is bad)
//   S -> C -> D -> G   (must be selected instead)
// ---------------------------------------------------------------------
void testCase2() {
    PlanningProblem p;
    p.addState(State(1, {0, 0}));   // S
    p.addState(State(2, {1, 1}));   // A
    p.addState(State(3, {2, 1}));   // X (bad)
    p.addState(State(4, {1, -1}));  // C
    p.addState(State(5, {2, -1}));  // D
    p.addState(State(6, {3, 0}));   // G

    p.addTransition(Transition(1, 1, 2, 1.0, 1.0, 1.0)); // S->A
    p.addTransition(Transition(2, 2, 3, 1.0, 1.0, 1.0)); // A->X
    p.addTransition(Transition(3, 3, 6, 1.0, 1.0, 1.0)); // X->G
    p.addTransition(Transition(4, 1, 4, 1.0, 1.0, 1.0)); // S->C
    p.addTransition(Transition(5, 4, 5, 1.0, 1.0, 1.0)); // C->D
    p.addTransition(Transition(6, 5, 6, 1.0, 1.0, 1.0)); // D->G

    p.initialState = 1;
    p.goalState = 6;
    p.setBadState(3, true); // X is bad -> S->A->X->G is fully excluded

    LPAStarPlanner planner;
    PlanningResult r = planner.plan(p);
    printResult("Test Case 2: Bad State Avoidance (expect S->C->D->G)", r);
    exportForPlot("tc2", p, r);
}

// ---------------------------------------------------------------------
// Test Case 3: Safety Margin
//   Path 1 (S->A->G): lower cost, passes close to a bad state.
//   Path 2 (S->B->G): higher cost, stays far from the bad state.
//   We run the SAME graph twice with different gamma (safety weight) to
//   show how the trade-off shifts the chosen path.
// ---------------------------------------------------------------------
void testCase3() {
    PlanningProblem p;
    p.addState(State(1, {-2, 0}));   // S
    p.addState(State(2, {0.1, 0}));  // A - very close to bad state X
    p.addState(State(3, {5, 0}));    // B - far from bad state X
    p.addState(State(4, {0, 0.5}));  // G
    p.addState(State(5, {0, 0}));    // X - bad state

    p.addTransition(Transition(1, 1, 2, 1.0, 1.0, 1.0)); // S->A cheap
    p.addTransition(Transition(2, 2, 4, 1.0, 1.0, 1.0)); // A->G
    p.addTransition(Transition(3, 1, 3, 2.0, 1.0, 1.0)); // S->B pricier
    p.addTransition(Transition(4, 3, 4, 1.0, 1.0, 1.0)); // B->G

    p.initialState = 1;
    p.goalState = 4;
    p.setBadState(5, true);

    std::cout << "-- Test Case 3: Safety Margin --\n";
    {
        LPAStarPlanner cheapFirst(1.0, 0.0, 0.0); // gamma=0: cost-only, ignores safety
        PlanningResult r = cheapFirst.plan(p);
        printResult("  gamma=0 (cost only, expect S->A->G, cost=2, dist-to-bad=0.1)", r);
        exportForPlot("tc3_gamma0", p, r);
    }
    {
        LPAStarPlanner safetyFirst(1.0, 1.0, 0.0); // gamma>0: rewards distance from bad state
        PlanningResult r = safetyFirst.plan(p);
        printResult("  gamma=1 (safety-weighted, expect S->B->G, cost=3, dist-to-bad=0.5)", r);
        exportForPlot("tc3_gamma1", p, r);
    }
}

// ---------------------------------------------------------------------
// Test Case 4: Dynamic Transition
//   Initially S->A->G. Later (A,G) becomes unavailable -> need alternate.
// ---------------------------------------------------------------------
void testCase4() {
    PlanningProblem p;
    p.addState(State(1, {0, 0})); // S
    p.addState(State(2, {1, 0})); // A
    p.addState(State(3, {2, 0})); // G
    p.addState(State(4, {1, -2})); // detour node
    p.addTransition(Transition(1, 1, 2, 1.0, 1.0, 1.0)); // S->A
    p.addTransition(Transition(2, 2, 3, 1.0, 1.0, 1.0)); // A->G (will be cut)
    p.addTransition(Transition(3, 1, 4, 2.0, 1.0, 1.0)); // S->detour
    p.addTransition(Transition(4, 4, 3, 2.0, 1.0, 1.0)); // detour->G
    p.initialState = 1;
    p.goalState = 3;

    LPAStarPlanner planner;
    PlanningResult r4a = planner.plan(p);
    printResult("Test Case 4a: before edge cut (expect S->A->G)", r4a);
    exportForPlot("tc4a", p, r4a);

    p.setTransitionAvailable(2, false);
    planner.onTransitionChanged(2);
    PlanningResult r4b = planner.replan();
    printResult("Test Case 4b: after (A,G) cut (expect S->detour->G)", r4b);
    exportForPlot("tc4b", p, r4b);
}

// ---------------------------------------------------------------------
// Test Case 5: Goal Update
//   Goal changes mid-run; g/rhs relative to start are reused, only the
//   search target changes -> cheap replan, no rebuild.
// ---------------------------------------------------------------------
void testCase5() {
    PlanningProblem p;
    p.addState(State(1, {0, 0})); // S
    p.addState(State(2, {1, 0})); // A
    p.addState(State(3, {2, 0})); // old goal G1
    p.addState(State(4, {1, 3})); // new goal G2
    p.addTransition(Transition(1, 1, 2, 1.0, 1.0, 1.0)); // S->A
    p.addTransition(Transition(2, 2, 3, 1.0, 1.0, 1.0)); // A->G1
    p.addTransition(Transition(3, 2, 4, 4.0, 1.0, 1.0)); // A->G2
    p.initialState = 1;
    p.goalState = 3;

    LPAStarPlanner planner;
    PlanningResult r5a = planner.plan(p);
    printResult("Test Case 5a: original goal G1 (expect S->A->G1)", r5a);
    exportForPlot("tc5a", p, r5a);

    p.goalState = 4;
    planner.onGoalChanged(4);
    PlanningResult r5b = planner.replan();
    printResult("Test Case 5b: goal switched to G2 (expect S->A->G2, statesExplored small)", r5b);
    exportForPlot("tc5b", p, r5b);
}

// ---------------------------------------------------------------------
// Test Case 6: Transition Addition
//   A new shortcut is inserted; planner should find the improved solution.
// ---------------------------------------------------------------------
void testCase6() {
    PlanningProblem p;
    p.addState(State(1, {0, 0})); // S
    p.addState(State(2, {1, 0})); // A
    p.addState(State(3, {2, 0})); // B
    p.addState(State(4, {3, 0})); // G
    p.addTransition(Transition(1, 1, 2, 1.0, 1.0, 1.0)); // S->A
    p.addTransition(Transition(2, 2, 3, 1.0, 1.0, 1.0)); // A->B
    p.addTransition(Transition(3, 3, 4, 1.0, 1.0, 1.0)); // B->G
    p.initialState = 1;
    p.goalState = 4;

    LPAStarPlanner planner;
    PlanningResult r6a = planner.plan(p);
    printResult("Test Case 6a: before shortcut (expect S->A->B->G, cost=3)", r6a);
    exportForPlot("tc6a", p, r6a);

    p.addTransition(Transition(4, 1, 4, 0.5, 1.0, 1.0)); // new direct shortcut S->G
    planner.onTransitionChanged(4);
    PlanningResult r6b = planner.replan();
    printResult("Test Case 6b: after shortcut added (expect S->G, cost=0.5)", r6b);
    exportForPlot("tc6b", p, r6b);
}

int main() {
    std::cout << std::fixed << std::setprecision(3);
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();
    return 0;
}