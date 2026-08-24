# User Manual — Safe Semantic Planner

This is a step-by-step guide to building, running, and interpreting the
planner. For *why* it's built this way, see `design_report.docx`. For a
quick project overview, see [README.md](README.md)

## 1. Prerequisites

| Tool | Needed for | Check with |
|---|---|---|
| C++17 compiler (g++ or clang++) | Everything | `g++ --version` |
| `make` | Everything | `make --version` |
| Python 3 + `pandas`, `matplotlib` | Charts | `python3 -c "import pandas, matplotlib"` |
| Node.js + `docx` npm package | Regenerating the design report only | `node --version` |

If Python packages are missing: `pip install -r pip_requirements.txt`
If Node's `docx` package is missing: `npm install` (reads `package.json`)

You do **not** need Node/npm to build, run, or benchmark the planner —
only to regenerate `design_report.docx`.

## 2. Building and running the planner

From the project root:

```
make run
```

This compiles `src/LPAStarPlanner.cpp` and `src/main.cpp` into an
executable called `planner`, then runs it immediately. You'll see output
for all 6 illustrative test cases from the assignment spec, one after
another, e.g.:

```
-- Test Case 1: Basic Reachability --
  path: 1 -> 2 -> 3 -> 4
  totalCost: 3.000  safetyScore(min dist to bad state): inf  statesExplored: 4
```

**How to read this:**

- `path`: the sequence of state IDs the planner chose, from start to goal.
- `totalCost`: sum of the raw `cost` field along the chosen transitions.
- `safetyScore`: the minimum distance from any state on the path to the
  nearest bad state (`inf` if there are no bad states in that test case).
- `statesExplored`: how many states LPA*'s search actually expanded.

Each test case's expected outcome is written in its section header (e.g.
"expect S->C->D->G") — compare the printed `path` against it. All 6 should
match; if any doesn't, something in your local copy has diverged from the
delivered source — see Troubleshooting (§6).

To rebuild without running: `make` (no `run`).
To just re-run an already-built binary: `./planner`.

## 3. Running the benchmark

```
make bench
```

Builds `benchmark` from `src/benchmark.cpp` and runs it. This generates
7 grid graphs of increasing size (100 to 4,900 states), and for each one
measures:

1. **Initial plan** — a full search from scratch.
2. **Fresh replan** — after cutting one transition, a brand-new search on
   the modified graph.
3. **Incremental replan** — after the same cut, LPA*'s `replan()`, reusing
   previous search state.

Console output summarizes states explored per scenario; full data
(including planning time in ms and peak memory in KB) is written to
`results/results.csv`.

**What to look for:** `incremental replan explored` should be
*substantially lower* than `fresh replan explored` at every grid size —
that's the core claim of the report (LPA* replans more efficiently than
solving from scratch). If you ever see `incremental` explored *higher*
than `fresh`, your `LPAStarPlanner.cpp`/`.hpp` are out of date — see
Troubleshooting (§6).

## 4. Generating charts

```
python3 plot_results.py
python3 plot_paths.py
```

- `plot_results.py` reads `results/results.csv` and writes 3 PNGs to
  `results/charts/`: planning time vs. graph size, states explored
  (fresh vs. incremental), and peak memory vs. graph size.
- `plot_paths.py` reads the per-test-case node/edge dumps that `make run`
  wrote to `results/plots/`, and draws a diagram of each (nodes, edges,
  bad states in red, the chosen path highlighted in green) to
  `results/charts/paths/`.

Run `make run` and `make bench` first — both scripts need the CSVs those
steps produce.

## 5. Regenerating the design report

```
make report
```

Requires Node.js with `npm install` done once (reads `package.json`). Reads
the PNGs in `results/charts/` and rebuilds `design_report.docx` from
scratch, with a footer (name, registration number, page numbers) on every
page. Run this *after* steps 2-4 above, so the charts it embeds reflect
your current run.

If LibreOffice (`soffice`) is installed and on your PATH, this step also
automatically converts the result to `design_report.pdf` in the same run —
you'll see `wrote design_report.pdf` in the output. If `soffice` isn't
found, you'll see a warning and only the `.docx` is produced; the report
content itself is unaffected either way. To get a PDF without installing
LibreOffice, open `design_report.docx` in Word or LibreOffice and use
"Export as PDF" / "Save As PDF" manually.

## 6. Troubleshooting

**"No such file or directory" errors from Python scripts.**
You skipped a step — `plot_results.py` needs `results/results.csv`
(`make bench`), `plot_paths.py` needs `results/plots/*.csv` (`make run`).
Run steps in the order given above.

**`make report` fails with `Cannot find module 'docx'`.**
Run `npm install` in the project root first.

**`make report` fails with `Cannot find module '.../make_report.js'`.**
The file isn't in your project root — check it's there; if not, it needs
to be copied in alongside `Makefile`.

**Benchmark shows incremental replan exploring *more* states than a fresh
replan.**
Your `src/LPAStarPlanner.cpp` (and/or `include/LPAStarPlanner.hpp`) is an
older version missing a bug fix — the `statesExplored` counter must be
reset at the start of `replan()`. Replace both files with the current
versions and rebuild (`make clean && make bench`).

**Full reset.**
```
make clean
```
Removes `planner`, `benchmark`, the entire `results/` folder, and
`design_report.docx`. Re-run steps 2-5 above in order to rebuild
everything from source.

## 7. Full rebuild sequence (reference)

```
make clean
make run
make bench
python3 plot_results.py
python3 plot_paths.py
make report
```