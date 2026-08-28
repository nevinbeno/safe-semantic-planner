# Safe Semantic Planner (LPA*)

**PCCST503 — Machine Learning, Assignment 1**

**Name:** Nevin Beno  
**University Register No.:** TCR24CS052

C++17 implementation of a planner that computes a safe, low-cost path
through a finite Cartesian state space, avoiding designated bad states
and replanning efficiently when the environment changes — using
**LPA* (Lifelong Planning A*)**.

## Deliverables in this repo

| # | Deliverable | Where |
|---|---|---|
| 1 | C++ source code | `include/`, `src/` |
| 2 | Design report | [Design Report](design_report.pdf) |
| 3 | Experimental results | `results/` |
| 4 | User manual | [USER MANUAL](USER_MANUAL.md) |
| 5 | Demonstration | [Demo Video](https://drive.google.com/file/d/1xhDQGUUWSoWwd7hZTWwjtevNR9DBPk1n/view?usp=sharing) |
| Bonus | Incremental replanning | `LPAStarPlanner::onTransitionChanged` / `onStateBadnessChanged` / `onGoalChanged` + `replan()` — benchmarked in `results/charts/` |

## Dependencies:
- C++17 compiler (g++/clang++)
- Python 3 with `pandas` and
`matplotlib` (`pip install -r pip_requirements.txt`) for the charts. 
- Node.js with the `docx` package (`npm install`, reading `package.json`) only if you want to regenerate the `.docx` report — everything else works without Node.

Full usage details (what each command does, how to read the output, how
to troubleshoot) are in [USER_MANUAL.md](USER_MANUAL.md)

## Quick start

```
make run      # build + run the 6 illustrative test cases from the spec
```

That's the fastest way to see the planner work. For the full experimental
pipeline (benchmarks, charts, report), see **"Regenerating everything from
scratch"** below.

## Project structure
```
.
├── benchmark               (binary file generated)
├── design_report.docx      (generated)
├── design_report.pdf       (generated)
├── include
│   ├── LPAStarPlanner.hpp
│   ├── Planner.hpp
│   ├── PlanningProblem.hpp
│   ├── PlanningResult.hpp
│   ├── State.hpp
│   └── Transition.hpp
├── Makefile
├── make_report.js
├── node_modules/
├── package.json
├── package-lock.json
├── PCCST503_Assignment_1.pdf
├── pip_requirements.txt
├── planner                 (binary file generated)
├── plot_paths.py
├── plot_results.py
├── README.md
├── results                  (generated)
│   ├── charts
│   │   ├── memory_usage.png
│   │   ├── paths
│   │   │   ├── tc1.png
│   │   │   ├── tc2.png
│   │   │   ├── tc3_gamma0.png
│   │   │   ├── tc3_gamma1.png
│   │   │   ├── tc4a.png
│   │   │   ├── tc4b.png
│   │   │   ├── tc5a.png
│   │   │   ├── tc5b.png
│   │   │   ├── tc6a.png
│   │   │   └── tc6b.png
│   │   ├── planning_time.png
│   │   └── states_explored.png
│   ├── plots
│   │   ├── tc1_edges.csv
│   │   ├── tc1_nodes.csv
│   │   ├── tc2_edges.csv
│   │   ├── tc2_nodes.csv
│   │   ├── tc3_gamma0_edges.csv
│   │   ├── tc3_gamma0_nodes.csv
│   │   ├── tc3_gamma1_edges.csv
│   │   ├── tc3_gamma1_nodes.csv
│   │   ├── tc4a_edges.csv
│   │   ├── tc4a_nodes.csv
│   │   ├── tc4b_edges.csv
│   │   ├── tc4b_nodes.csv
│   │   ├── tc5a_edges.csv
│   │   ├── tc5a_nodes.csv
│   │   ├── tc5b_edges.csv
│   │   ├── tc5b_nodes.csv
│   │   ├── tc6a_edges.csv
│   │   ├── tc6a_nodes.csv
│   │   ├── tc6b_edges.csv
│   │   └── tc6b_nodes.csv
│   └── results.csv
├── src
│   ├── benchmark.cpp
│   ├── LPAStarPlanner.cpp
│   └── main.cpp
├── USER_MANUAL.md
└── venv/               (Python Virtual Environment)

```

## Regenerating everything from scratch

```
make clean                # wipe binaries, results/, and generated reports
make run                  # -> results/plots/*.csv
make bench                # -> results/results.csv
python3 plot_paths.py     # -> results/charts/paths/*.png
python3 plot_results.py   # -> results/charts/*.png
make report               # -> design_report.docx (+ .pdf if LibreOffice is installed)
```

Run in that order — each step reads what the previous one wrote. The
benchmark uses fixed random seeds, so `statesExplored`/cost figures
reproduce exactly; wall-clock timings will vary slightly with your
machine's load.

## Design highlights

Full design rationale, complexity analysis, and experimental results are in
`design_report.docx`. Summary:

- **Why LPA\*, not D\* Lite**: D\* Lite is built around a robot physically
  moving through the graph. This planner computes a path once and replans
  when the *environment* changes (goal, bad states, transitions) — not when
  an agent moves. LPA\*'s `g`/`rhs` values are relative to a fixed start, so
  they stay valid across a goal change; `onGoalChanged()` is O(1).
- **Bad states are a hard constraint**: excluded structurally from the
  search graph (`validOutgoing`/`validIncoming`), never scored as a penalty.
- **Multi-objective scoring**: `Score(P) = alpha*G - beta*C + gamma*D + delta*R`
  is folded into one edge weight, `max(0, beta*cost - gamma*safetyDist -
  delta*reliability)`, so ordinary shortest-path search applies.
- **Heuristic correctness caveat**: `h(u) = beta * distance(u, goal)` is only
  admissible when `gamma = delta = 0`; the implementation falls back to
  `h = 0` otherwise to preserve optimality (see `design_report.docx` §9 for
  the bug this caught during development).
- **Measured replanning advantage**: incremental replanning explored 34-42%
  of the states a fresh search needed, across graph sizes from 100 to 4,900
  states (`results/charts/states_explored.png`).

## Test cases

All 6 illustrative test cases from the assignment spec are implemented in
`src/main.cpp` and verified against their expected outcome:

| Test Case | Scenario | Expected |
|---|---|---|
| 1 | Basic Reachability | `S -> A -> B -> G` |
| 2 | Bad State Avoidance | Detour around bad state |
| 3 | Safety Margin | Cost-only vs. safety-weighted path choice |
| 4 | Dynamic Transition | Replan after an edge is cut |
| 5 | Goal Update | Cheap replan after goal switch |
| 6 | Transition Addition | Shortcut discovered after insertion |