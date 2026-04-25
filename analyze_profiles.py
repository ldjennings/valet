import pstats
import matplotlib.pyplot as plt
import numpy as np

PROFILES = {
    "unoptimized": "profile_unoptimized.prof",
    "optimized":   "profile_optimized.prof",
}

# (display name, substring to match against "file:lineno(funcname)" key)
FUNCTIONS = [
    ("rotate",                  "affinity.py:126(rotate)"),
    ("translate",               "affinity.py:247(translate)"),
    ("_affine_coords",          "affinity.py:72(_affine_coords)"),
    ("_has_possible_collision", "obstacle.py:102(_has_possible_collision)"),
    ("intersects_any",          "obstacle.py:24(intersects_any)"),
    ("is_valid_state",          "obstacle.py:171(is_valid_state)"),
    ("propagate",               "bots.py:392(propagate)"),
]


def extract(prof_path):
    p = pstats.Stats(prof_path)
    # key: (file, lineno, funcname), value: (cc, nc, tt, ct, callers)
    calls = {}
    time  = {}
    total_time = sum(tt for (cc, nc, tt, ct, _) in p.stats.values())
    for (fname, lineno, funcname), (cc, nc, tt, ct, _) in p.stats.items():
        key = f"{fname}:{lineno}({funcname})"
        for label, needle in FUNCTIONS:
            if needle in key:
                calls[label] = calls.get(label, 0) + nc
                time[label]  = time.get(label, 0.0) + tt
    return calls, time, total_time


data = {name: extract(path) for name, path in PROFILES.items()}

labels   = [f[0] for f in FUNCTIONS]
x        = np.arange(len(labels))
width    = 0.35

FONT_TITLE  = 16
FONT_LABEL  = 14
FONT_TICK   = 13
FONT_LEGEND = 13

fig, axes = plt.subplots(2, 1, figsize=(14, 11))

total_un  = data["unoptimized"][2]
total_opt = data["optimized"][2]

for ax, metric, ylabel in [
    (axes[0], 0, "Call count"),
    (axes[1], 1, "Exclusive time (s)"),
]:
    vals_un  = [data["unoptimized"][metric].get(l, 0) for l in labels]
    vals_opt = [data["optimized"][metric].get(l, 0)   for l in labels]

    ax.bar(x - width/2, vals_un,  width, label="Unoptimized", color="#d62728")
    ax.bar(x + width/2, vals_opt, width, label="Optimized",   color="#2ca02c")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha="right", fontsize=FONT_TICK)
    ax.set_ylabel(ylabel, fontsize=FONT_LABEL)
    ax.legend(fontsize=FONT_LEGEND)
    fmt = (lambda v, _: f"{v:,.0f}") if metric == 0 else (lambda v, _: f"{v:.2f}")
    ax.yaxis.set_major_formatter(plt.FuncFormatter(fmt))
    ax.tick_params(axis="y", labelsize=FONT_TICK)

fig.suptitle(
    f"Collision Checker Optimization — Call Counts and Exclusive Time\n"
    f"Total runtime:  unoptimized {total_un:.2f} s  |  optimized {total_opt:.2f} s  "
    f"({total_un/total_opt:.2f}× speedup)",
    fontsize=FONT_TITLE,
)
plt.tight_layout()
plt.savefig("profile_comparison.png", dpi=150)
print("Saved profile_comparison.png")
plt.show()

# ── text summary ──────────────────────────────────────────────────────────────
tracked_un  = sum(data["unoptimized"][1].get(l, 0.0) for l in labels)
tracked_opt = sum(data["optimized"][1].get(l, 0.0)   for l in labels)

print(f"\nTotal runtime:  unoptimized={total_un:.3f}s   optimized={total_opt:.3f}s   "
      f"speedup={total_un/total_opt:.2f}x   saved={total_un-total_opt:.3f}s")
print(f"Tracked fns:    unoptimized={tracked_un:.3f}s ({tracked_un/total_un*100:.1f}%)   "
      f"optimized={tracked_opt:.3f}s ({tracked_opt/total_opt*100:.1f}%)\n")

print(f"\n{'Function':<28} {'calls (unopt)':>14} {'calls (opt)':>12} {'Δcalls':>10}  "
      f"{'time (unopt)':>13} {'time (opt)':>11} {'Δtime':>9}")
print("-" * 105)
for label in labels:
    cu  = data["unoptimized"][0].get(label, 0)
    co  = data["optimized"][0].get(label, 0)
    tu  = data["unoptimized"][1].get(label, 0.0)
    to_ = data["optimized"][1].get(label, 0.0)
    print(f"{label:<28} {cu:>14,} {co:>12,} {co-cu:>+10,}  "
          f"{tu:>13.3f} {to_:>11.3f} {to_-tu:>+9.3f}")
