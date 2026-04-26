"""Generate collision_opt_comparison.svg from two cProfile .prof files.

Expected files (relative to this script):
    profiles/unoptimized.prof
    profiles/optimized.prof

Run from any directory:
    python report/media/scripts/analyze_profiles.py
"""

import pstats
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager

_font_dir = Path(__file__).parents[2] / "newcm-8.0.0" / "otf"
for _variant in ("Regular", "Bold", "Italic", "BoldItalic"):
    font_manager.fontManager.addfont(str(_font_dir / f"NewCM10-{_variant}.otf"))
for _variant in ("Regular", "Italic"):
    font_manager.fontManager.addfont(str(_font_dir / f"NewCM08-{_variant}.otf"))
matplotlib.rcParams["font.family"] = "NewComputerModern10"
matplotlib.rcParams["font.size"] = 14        # tick labels, ylabel, legend
matplotlib.rcParams["axes.titlesize"] = 12   # per-axes titles (unused here)
matplotlib.rcParams["figure.titlesize"] = 16 # suptitle

import matplotlib.pyplot as plt
import numpy as np

HERE     = Path(__file__).parent
OUT      = HERE.parent / "photos" / "collision_opt_comparison.svg"
PROF_DIR = HERE / "profiles"

PROFILES = {
    "unoptimized": PROF_DIR / "unoptimized.prof",
    "optimized":   PROF_DIR / "optimized.prof",
}

# Match on filename + funcname only (no line number — robust to code changes)
FUNCTIONS = [
    ("rotate",                  ("affinity.py",  "rotate")),
    ("translate",               ("affinity.py",  "translate")),
    ("_affine_coords",          ("affinity.py",  "_affine_coords")),
    ("_has_possible_collision", ("obstacle.py",  "_has_possible_collision")),
    ("intersects_any",          ("obstacle.py",  "intersects_any")),
    ("is_valid_state",          ("obstacle.py",  "is_valid_state")),
    ("propagate",               ("bots.py",      "propagate")),
]


def extract(prof_path):
    p = pstats.Stats(str(prof_path))
    calls = {}
    time  = {}
    total_time = sum(tt for (cc, nc, tt, ct, _) in p.stats.values())
    for (fname, lineno, funcname), (cc, nc, tt, ct, _) in p.stats.items():
        for label, (file_needle, func_needle) in FUNCTIONS:
            if file_needle in fname and funcname == func_needle:
                calls[label] = calls.get(label, 0) + nc
                time[label]  = time.get(label, 0.0) + tt
    return calls, time, total_time


data = {name: extract(path) for name, path in PROFILES.items()}

labels = [f[0] for f in FUNCTIONS]
x      = np.arange(len(labels))
width  = 0.35

total_un  = data["unoptimized"][2]
total_opt = data["optimized"][2]

fig, axes = plt.subplots(2, 1, figsize=(12, 8))

for ax, metric, ylabel in [
    (axes[0], 0, "Call count"),
    (axes[1], 1, "Exclusive time (s)"),
]:
    vals_un  = [data["unoptimized"][metric].get(l, 0) for l in labels]
    vals_opt = [data["optimized"][metric].get(l, 0)   for l in labels]

    ax.bar(x - width/2, vals_un,  width, label="Unoptimized", color="#d62728")
    ax.bar(x + width/2, vals_opt, width, label="Optimized",   color="#2ca02c")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.set_ylabel(ylabel)
    ax.legend()
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:,.2f}"))

fig.suptitle(
    f"Collision checker optimisation — call counts and exclusive time\n"
    f"Total runtime: unoptimized {total_un:.2f} s  |  optimized {total_opt:.2f} s  "
    f"({total_un/total_opt:.2f}× speedup)"
)
fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")

# ── text summary ───────────────────────────────────────────────────────────────

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
