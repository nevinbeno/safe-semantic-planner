# Safe Semantic Planner (LPA*)

C++17 implementation of the planner described in the PCCST503 Assignment 1
spec, using **LPA\* (Lifelong Planning A\*)**.

## Build & run

Everything under `results/` **and** `design_report.docx` are generated —
nothing is checked in or required to already exist. From a bare checkout
(just `include/`, `src/`, `Makefile`, `make_report.js`, the two `plot_*.py`
scripts, this README):

```
make run                # builds ./planner, runs it, writes results/plots/*.csv
make bench               # builds ./benchmark, runs it, writes results/results.csv
python3 plot_results.py  # reads results/results.csv -> results/charts/*.png
python3 plot_paths.py    # reads results/plots/*.csv -> results/charts/paths/*.png
make report              # reads results/charts/*.png -> design_report.docx
```

Run them in that order — each step reads what the previous one wrote. The
first four create their own output folders automatically
(`std::filesystem::create_directories` in C++, `os.makedirs(...,
exist_ok=True)` in Python) — no manual `mkdir` needed.

`make report` needs **Node.js with the `docx` npm package**
(`npm install docx`) — a heavier dependency than the rest of the project,
which is why it's a separate explicit step rather than bundled into `run`
or `bench`. If you don't have Node set up, everything else still works;
you just won't be able to regenerate the .docx (the charts and CSVs it
would embed are already sitting in `results/charts/` either way).

`make clean` removes both binaries (`planner`, `benchmark`), the generated
`results/` folder, **and** `design_report.docx` — a true full reset.
Re-running the five commands above regenerates everything identically
(the benchmark's random graphs use fixed seeds; timings will vary
slightly run to run depending on your machine's load, but states-explored
and cost figures will match exactly).

Python dependencies: `pandas`, `matplotlib` (`pip install pandas matplotlib`
if missing).

## Files

- `include/State.hpp` — a point in R^d, plus Euclidean distance helper.
- `include/Transition.hpp` — directed edge with cost/safety/reliability/available.
- `include/PlanningProblem.hpp` — owns states/transitions, adjacency maps for
  O(deg) successor/predecessor lookup, bad-state set, and `minDistanceToBadState`.
- `include/PlanningResult.hpp` / `include/Planner.hpp` — per spec.
- `include/LPAStarPlanner.hpp` / `src/LPAStarPlanner.cpp` — the algorithm.
- `src/benchmark.cpp` — scalability benchmark (see Experimental Results below).
- `src/main.cpp` — Test Cases 1–6 from the assignment, runnable directly.
- `plot_results.py` — turns `results/results.csv` into the 3 performance charts.
- `plot_paths.py` — turns each test case's node/edge dump into a graph diagram.
- `make_report.js` — builds `design_report.docx` from the charts (needs Node + `docx` npm package).

## Design notes (for the report)

### 1. State representation
Each `State` is an id + `vector<double>` embedding, so the planner works for
any dimensionality `d` without recompiling — the spec only fixes "finite
Cartesian space", not `d`.

### 2. Data structures
- States/transitions are stored in `unordered_map`s keyed by id (O(1) lookup).
- Two adjacency maps (`outgoing`, `incoming`) give successors/predecessors in
  O(out-degree)/O(in-degree) instead of scanning all transitions.
- Bad states are excluded **structurally**: `validOutgoing`/`validIncoming`
  filter out any transition whose endpoint is a bad state or that is marked
  unavailable, so the search graph never contains a bad state as a
  traversable node — this directly satisfies Optimization Objective #2
  ("never visit a bad state") as a hard constraint rather than a penalty.
- The open list is a `std::set<pair<Key,id>>` (balanced BST) plus a
  parallel `unordered_map<id,Key>` recording each vertex's current queue
  key, so "is u in the queue" and "remove u" are both O(log n) — needed
  for LPA*'s repeated decrease/insert/remove operations.

### 3. Why LPA\* (not D\* Lite)
Both algorithms handle dynamic edge-cost changes efficiently, but D\* Lite is
built around a robot physically moving through the graph (it searches
backward from the goal to the *agent's current position*, which changes
every step). This assignment's planner computes a path once and replans
when the *environment* changes — goal, bad states, transitions — not when
an agent moves. LPA\*'s `g`/`rhs` values are defined relative to a **fixed
start**, which has a convenient consequence: they stay valid across a goal
change, so `onGoalChanged()` is O(1) and `replan()` only re-expands the part
of the frontier that was never settled toward the old goal (Test Case 5).

### 4. Heuristic function
`h(u) = beta * EuclideanDistance(u, goal)`, used only for **key
prioritization**, not correctness. Important caveat: this is only an
admissible heuristic when `gamma = delta = 0` (pure cost minimization). Once
the safety/reliability terms subtract from edge weight, actual remaining
cost can be far below `beta * distance` (even clamped to 0), so a
distance-based heuristic would overestimate and could return a suboptimal
path. The implementation detects this and falls back to `h = 0` whenever
`gamma != 0 || delta != 0`, which is always admissible (degrades to
Dijkstra-style search — correct, just less guided).

### 5. Safety computation
`PlanningProblem::minDistanceToBadState(u)` computes the Euclidean distance
from `u` to the nearest bad state (O(k) per call, k = |bad states|; cached
per query rather than precomputed since bad states can change at runtime).
This directly implements Optimization Objective #4 (maximize minimum
distance to bad states) and feeds the `D` term of `Score(P)`.

### 6. Multi-objective scoring
`Score(P) = alpha*G - beta*C + gamma*D + delta*R` is folded into a single
scalar edge weight so standard shortest-path search applies:

```
edgeWeight(u->v) = max(0, beta*cost(u,v) - gamma*safetyDist(v) - delta*reliability(u,v))
```

`alpha`/`G` (goal completion) isn't part of the per-edge weight — it's
binary and handled by whether `g(goal)` is finite after search. The `max(0,
...)` clamp is necessary because LPA\*'s correctness proof assumes
non-negative edge weights; see Test Case 3 in `main.cpp` for a worked
example of how `beta/gamma` trade off cost against safety margin, including
where clamping saturates the safety bonus.

### 7. Time complexity
Each `updateVertex` call does O(in-degree) work. In the worst case (e.g. an
edge-cost decrease propagating widely), LPA\* re-expands O(V) vertices, each
doing O(E/V) work on average and O(log V) for queue operations →
**O(E log V)** per replan, same asymptotic bound as one Dijkstra/A\* run,
but in practice touching far fewer vertices than a full from-scratch
re-search after a small change (this locality is exactly what "efficient
replanning" in the spec's Dynamic Environment section means).

### 8. Space complexity
O(V + E): adjacency maps store each transition twice (once in `outgoing`,
once in `incoming`), plus O(V) for `g`, `rhs`, and the queue's key index.

## Extending this

- **Multi-goal planning (bonus)**: run `onGoalChanged` + `replan()` per
  candidate goal and keep the best `PlanningResult`; since `g`/`rhs` are
  reused across calls this is much cheaper than independent searches.
- **Incremental replanning (bonus)**: already the core design — see
  `onTransitionChanged`, `onStateBadnessChanged`, `onGoalChanged` +
  `replan()` in `LPAStarPlanner`.
- **Time-dependent availability**: extend `Transition` with a validity
  window and call `onTransitionChanged` when it flips, exactly like
  Test Case 4.