import pandas as pd
import matplotlib.pyplot as plt
import os

os.makedirs("results/charts", exist_ok=True)

df = pd.read_csv("results/results.csv")
initial = df[df["mode"] == "initial_plan"]
incr = df[df["mode"] == "incremental_replan"]
fresh = df[df["mode"] == "fresh_replan"]

# Chart 1: planning time vs graph size
plt.figure(figsize=(7, 5))
plt.plot(initial["n"], initial["planningTimeMs"], marker="o", label="Initial plan (from scratch)")
plt.plot(fresh["n"], fresh["planningTimeMs"], marker="s", label="Fresh replan (after edge cut)")
plt.plot(incr["n"], incr["planningTimeMs"], marker="^", label="Incremental replan (LPA*, after edge cut)")
plt.xlabel("Number of states (n)")
plt.ylabel("Planning time (ms)")
plt.title("Planning Time vs Graph Size")
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("results/charts/planning_time.png", dpi=150)
plt.close()

# Chart 2: states explored, fresh vs incremental replan
plt.figure(figsize=(7, 5))
width = 0.35
x = range(len(fresh))
plt.bar([i - width/2 for i in x], fresh["statesExplored"], width, label="Fresh replan")
plt.bar([i + width/2 for i in x], incr["statesExplored"], width, label="Incremental replan (LPA*)")
plt.xticks(list(x), [f"{n}" for n in fresh["n"]])
plt.xlabel("Number of states (n)")
plt.ylabel("States explored")
plt.title("States Explored After a Transition Change:\nFresh Search vs LPA* Incremental Replan")
plt.legend()
plt.grid(alpha=0.3, axis="y")
plt.tight_layout()
plt.savefig("results/charts/states_explored.png", dpi=150)
plt.close()

# Chart 3: memory usage vs graph size
plt.figure(figsize=(7, 5))
plt.plot(initial["n"], initial["memoryKb"], marker="o", color="tab:green")
plt.xlabel("Number of states (n)")
plt.ylabel("Peak memory (KB)")
plt.title("Peak Memory Usage vs Graph Size")
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("results/charts/memory_usage.png", dpi=150)
plt.close()

print("Plotting Results: Done")