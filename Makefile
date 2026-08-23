CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Iinclude
SRC := src/LPAStarPlanner.cpp src/main.cpp
BENCH_SRC := src/LPAStarPlanner.cpp src/benchmark.cpp
BIN := planner
BENCH_BIN := benchmark
REPORT := design_report.docx

.PHONY: all clean run bench report

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(BIN) $(SRC)

$(BENCH_BIN): $(BENCH_SRC)
	$(CXX) $(CXXFLAGS) -o $(BENCH_BIN) $(BENCH_SRC)

run: $(BIN)
	./$(BIN)

bench: $(BENCH_BIN)
	./$(BENCH_BIN)

# Regenerates design_report.docx from the current chart PNGs.
# Needs Node.js with the `docx` npm package installed (npm install docx) --
# a heavier dependency than the rest of this project, so it's a separate,
# explicit target rather than part of `all`/`run`/`bench`.
# Run `make run`, `make bench`, and the two plot_*.py scripts FIRST so the
# charts this reads actually reflect your current code.
report:
	node make_report.js

clean:
	rm -f $(BIN) $(BENCH_BIN) $(REPORT)
	rm -rf results