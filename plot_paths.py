"""
Plots each test-case graph: states as nodes, transitions as edges, bad
states in red, start/goal marked, and the chosen path highlighted.
Run from the project root: python3 plot_paths.py
"""
import pandas as pd
import matplotlib.pyplot as plt
import os

CASES = ["tc1", "tc2", "tc3_gamma0", "tc3_gamma1", "tc4a", "tc4b",
         "tc5a", "tc5b", "tc6a", "tc6b"]

TITLES = {
    "tc1": "Test Case 1: Basic Reachability",
    "tc2": "Test Case 2: Bad State Avoidance",
    "tc3_gamma0": "Test Case 3a: Safety Margin (gamma=0, cost only)",
    "tc3_gamma1": "Test Case 3b: Safety Margin (gamma=1, safety-weighted)",
    "tc4a": "Test Case 4a: Before Edge Cut",
    "tc4b": "Test Case 4b: After Edge Cut (Replanned)",
    "tc5a": "Test Case 5a: Original Goal G1",
    "tc5b": "Test Case 5b: Goal Switched to G2 (Replanned)",
    "tc6a": "Test Case 6a: Before Shortcut Added",
    "tc6b": "Test Case 6b: After Shortcut Added (Replanned)",
}

os.makedirs("results/charts/paths", exist_ok=True)

for case in CASES:
    nodes = pd.read_csv(f"results/plots/{case}_nodes.csv")
    edges = pd.read_csv(f"results/plots/{case}_edges.csv")
    pos = {row.id: (row.x, row.y) for row in nodes.itertuples()}

    plt.figure(figsize=(6, 5))

    # Draw edges first (so nodes sit on top)
    for e in edges.itertuples(index=False):
        x1, y1 = pos[e[0]]  # 'from' column (renamed by pandas since it's a keyword)
        x2, y2 = pos[e.to]
        if e.onPath:
            plt.annotate("", xy=(x2, y2), xytext=(x1, y1),
                         arrowprops=dict(arrowstyle="-|>", color="tab:green", lw=2.5,
                                          shrinkA=12, shrinkB=12))
        else:
            plt.annotate("", xy=(x2, y2), xytext=(x1, y1),
                         arrowprops=dict(arrowstyle="-|>", color="lightgray", lw=1,
                                          shrinkA=12, shrinkB=12))

    # Draw nodes
    for n in nodes.itertuples():
        if n.bad:
            color = "tab:red"
        elif n.role == "start":
            color = "tab:blue"
        elif n.role == "goal":
            color = "tab:purple"
        else:
            color = "lightsteelblue"
        plt.scatter(n.x, n.y, s=500, color=color, zorder=3, edgecolors="black")
        plt.text(n.x, n.y, str(n.id), ha="center", va="center", zorder=4,
                  fontsize=10, fontweight="bold",
                  color="white" if color != "lightsteelblue" else "black")

    plt.title(TITLES.get(case, case))
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(f"results/charts/paths/{case}.png", dpi=150)
    plt.close()

print("Plotting Paths: Done")
