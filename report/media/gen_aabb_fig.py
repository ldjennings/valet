"""Generate aabb_check.svg — figure illustrating the AABB broad-phase collision check.

Shows a rotated car footprint on a grid, with its axis-aligned bounding box (AABB)
drawn as a dashed rectangle, and the grid cells covered by the AABB highlighted.
One obstacle cell falls inside the AABB but outside the true footprint, illustrating
that the broad phase is conservative (may produce false positives, never false negatives).

Run from any directory; output is written next to this script.
"""

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
_font_dir = Path(__file__).parents[1] / "newcm-8.0.0" / "otf"
for _variant in ("Regular", "Bold", "Italic", "BoldItalic"):
    font_manager.fontManager.addfont(str(_font_dir / f"NewCM10-{_variant}.otf"))
for _variant in ("Regular", "Italic"):
    font_manager.fontManager.addfont(str(_font_dir / f"NewCM08-{_variant}.otf"))
matplotlib.rcParams["font.family"] = "NewComputerModern10"
matplotlib.rcParams["font.size"] = 11

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon, FancyArrowPatch
from shapely.affinity import rotate, translate
from shapely.geometry import box

OUT = Path(__file__).with_name("photos") / "aabb_check.svg"

# ── Car geometry (mirrors geometry.py / config.py) ────────────────────────────

WHEELBASE  = 2.8
CAR_LENGTH = 5.2
CAR_WIDTH  = 1.8
CELL_SIZE  = 3.0   # meters per grid cell
GRID_COLS  = 6
GRID_ROWS  = 5

def make_car_base():
    rect = box(-CAR_LENGTH / 2, -CAR_WIDTH / 2, CAR_LENGTH / 2, CAR_WIDTH / 2)
    rear_overhang = CAR_LENGTH - WHEELBASE
    offset = (CAR_LENGTH / 2) - rear_overhang
    return translate(rect, xoff=offset)

# ── Place car ─────────────────────────────────────────────────────────────────

CAR_X     = 9.5     # rear axle world position
CAR_Y     = 7.5
HEADING   = 35.0    # degrees

base      = make_car_base()
rotated   = rotate(base, HEADING, origin=(0, 0))
car_shape = translate(rotated, xoff=CAR_X, yoff=CAR_Y)

# AABB of translated shape
minx, miny, maxx, maxy = car_shape.bounds

# Grid cells covered by AABB (mirrors _has_possible_collision indexing)
min_i = int(minx // CELL_SIZE)
max_i = int(maxx // CELL_SIZE)
min_j = int(miny // CELL_SIZE)
max_j = int(maxy // CELL_SIZE)

# ── Obstacle cells ────────────────────────────────────────────────────────────
# (col, row) — one outside AABB, one inside AABB but not touching car, one touching car

obstacle_cells = [
    (0, 4),   # far away — outside AABB, clearly free
    (4, 1),   # inside AABB, but car shape doesn't touch it → broad-phase false positive
    (1, 0),   # outside AABB
]

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(7, 6))

# Draw grid
for col in range(GRID_COLS + 1):
    ax.axvline(col * CELL_SIZE, color="#cccccc", linewidth=0.7, zorder=1)
for row in range(GRID_ROWS + 1):
    ax.axhline(row * CELL_SIZE, color="#cccccc", linewidth=0.7, zorder=1)

# Shade AABB-covered cells
for ci in range(min_i, max_i + 1):
    for cj in range(min_j, max_j + 1):
        rect = mpatches.Rectangle(
            (ci * CELL_SIZE, cj * CELL_SIZE), CELL_SIZE, CELL_SIZE,
            facecolor="#aec6e8", edgecolor="none", alpha=0.5, zorder=2
        )
        ax.add_patch(rect)

# Draw obstacle cells
for (ci, cj) in obstacle_cells:
    in_aabb = (min_i <= ci <= max_i) and (min_j <= cj <= max_j)
    color = "#8b008b" if in_aabb else "#555555"
    rect = mpatches.Rectangle(
        (ci * CELL_SIZE, cj * CELL_SIZE), CELL_SIZE, CELL_SIZE,
        facecolor=color, edgecolor="none", alpha=0.75, zorder=3
    )
    ax.add_patch(rect)

# Draw car footprint
xs, ys = car_shape.exterior.xy
car_patch = MplPolygon(list(zip(xs, ys)),
                        facecolor="#2a5caa", edgecolor="#1a3c7a",
                        alpha=0.7, linewidth=1.2, zorder=4)
ax.add_patch(car_patch)

# Draw AABB
aabb_patch = mpatches.Rectangle(
    (minx, miny), maxx - minx, maxy - miny,
    facecolor="none", edgecolor="#d4600a",
    linewidth=1.5, linestyle="--", zorder=5
)
ax.add_patch(aabb_patch)

# ── Labels ────────────────────────────────────────────────────────────────────

# Label AABB
ax.text(minx + (maxx - minx) / 2, maxy + 0.25, "AABB",
        color="#d4600a", fontsize=9, ha="center", va="bottom")

# ── Legend ────────────────────────────────────────────────────────────────────

legend_patches = [
    mpatches.Patch(facecolor="#2a5caa", edgecolor="#1a3c7a", alpha=0.7, label="Car footprint"),
    mpatches.Patch(facecolor="none", edgecolor="#d4600a", linestyle="--", linewidth=1.5, label="AABB"),
    mpatches.Patch(facecolor="#aec6e8", alpha=0.5, label="Cells checked (inside AABB)"),
    mpatches.Patch(facecolor="#555555", alpha=0.75, label="Obstacle (outside AABB — skipped)"),
    mpatches.Patch(facecolor="#8b008b", alpha=0.75, label="Obstacle (inside AABB — exact check triggered)"),
]
ax.legend(handles=legend_patches, loc="upper left",
          prop=font_manager.FontProperties(family="NewComputerModern08", size=11),
          framealpha=0.9)

# ── Axes ──────────────────────────────────────────────────────────────────────

ax.set_xlim(0, GRID_COLS * CELL_SIZE)
ax.set_ylim(0, GRID_ROWS * CELL_SIZE)
ax.set_aspect("equal")
ax.set_xticks([])
ax.set_yticks([])
ax.set_axis_off()

# ── Save ──────────────────────────────────────────────────────────────────────

fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")
