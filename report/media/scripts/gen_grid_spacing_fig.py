"""Generate grid_spacing.svg — shows the A* grid cell size relative to the car footprint.

The key insight: the 1 m × 1 m duplicate-detection grid is much finer than the
5.2 m × 1.8 m car body, so any two nodes that hash to the same cell have nearly
identical footprints and one can safely be skipped.

Run from any directory; output is written next to this script.
"""

import math
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
matplotlib.rcParams["font.size"] = 11

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon, FancyArrowPatch
from shapely.affinity import rotate, translate
from shapely.geometry import box

OUT = Path(__file__).parents[1] / "photos" / "grid_spacing.svg"

# ── Constants (mirrors config.py / main.py) ───────────────────────────────────

CAR_LENGTH  = 5.2
CAR_WIDTH   = 1.8
WHEELBASE   = 2.8
SPACING     = 1.0       # A* grid cell size (metres)
HEADING_DEG = 20.0      # slight angle so the grid relationship is clear

# ── Car footprint ─────────────────────────────────────────────────────────────

def make_car_base():
    rect = box(-CAR_LENGTH / 2, -CAR_WIDTH / 2, CAR_LENGTH / 2, CAR_WIDTH / 2)
    rear_overhang = CAR_LENGTH - WHEELBASE
    offset = (CAR_LENGTH / 2) - rear_overhang
    return translate(rect, xoff=offset)

base = make_car_base()

# Place rear axle at a non-round position inside a cell to make the snapping visible
AX_X, AX_Y = 3.3, 2.6
shape = rotate(base, HEADING_DEG, origin=(0, 0))
shape = translate(shape, xoff=AX_X, yoff=AX_Y)

# Grid cell the axle falls in (floor so cell edges align with grid lines)
cell_col = int(AX_X / SPACING)
cell_row = int(AX_Y / SPACING)

# ── Plot ──────────────────────────────────────────────────────────────────────

GRID_W, GRID_H = 9, 6
fig, ax = plt.subplots(figsize=(8, 5.5))

# Grid lines
for col in range(GRID_W + 1):
    ax.axvline(col * SPACING, color="#cccccc", linewidth=0.7, zorder=1)
for row in range(GRID_H + 1):
    ax.axhline(row * SPACING, color="#cccccc", linewidth=0.7, zorder=1)

# Highlight the occupied cell
highlight = mpatches.Rectangle(
    (cell_col * SPACING, cell_row * SPACING),
    SPACING, SPACING,
    facecolor="#f4a636", edgecolor="#c07800", linewidth=1.2,
    alpha=0.6, zorder=2,
)
ax.add_patch(highlight)

# Car footprint
xs, ys = shape.exterior.xy
car_patch = MplPolygon(list(zip(xs, ys)),
                        facecolor="#2a5caa", edgecolor="#1a3c7a",
                        alpha=0.65, linewidth=1.2, zorder=3)
ax.add_patch(car_patch)

# Rear axle position dot
ax.scatter([AX_X], [AX_Y], color="#1a3c7a", s=30, zorder=5)

# Cell centre dot
snap_x = (cell_col + 0.5) * SPACING
snap_y = (cell_row + 0.5) * SPACING
ax.scatter([snap_x], [snap_y], color="#c07800", s=30, zorder=5, marker="x")

# ── Dimension annotations ─────────────────────────────────────────────────────

# Car length arrow (along the car's axis)
rad = math.radians(HEADING_DEG)
fwd = (math.cos(rad), math.sin(rad))
perp = (-math.sin(rad), math.cos(rad))

# rear and front axle positions (approximate visual ends of the car along its axis)
rear_end  = (AX_X - (CAR_LENGTH - WHEELBASE) * fwd[0],
             AX_Y - (CAR_LENGTH - WHEELBASE) * fwd[1])
front_end = (AX_X + WHEELBASE * fwd[0],
             AX_Y + WHEELBASE * fwd[1])

offset_perp = 1.35  # how far below the car to draw the length arrow
p0 = (rear_end[0]  - offset_perp * perp[0], rear_end[1]  - offset_perp * perp[1])
p1 = (front_end[0] - offset_perp * perp[0], front_end[1] - offset_perp * perp[1])
ax.annotate("", xy=p1, xytext=p0,
            arrowprops=dict(arrowstyle="<->", color="#333", lw=1.0), zorder=6)
mid = ((p0[0] + p1[0]) / 2 - 0.15 * perp[0],
       (p0[1] + p1[1]) / 2 - 0.15 * perp[1])
ax.text(mid[0], mid[1], f"{CAR_LENGTH} m", ha="center", va="top",
        fontsize=9, color="#333", rotation=HEADING_DEG)

# Grid cell size arrow (horizontal, below the grid)
cx0 = cell_col * SPACING
cx1 = (cell_col + 1) * SPACING
cy  = -0.55
ax.annotate("", xy=(cx1, cy), xytext=(cx0, cy),
            arrowprops=dict(arrowstyle="<->", color="#c07800", lw=1.0), zorder=6)
ax.text((cx0 + cx1) / 2, cy - 0.12, f"{SPACING} m cell",
        ha="center", va="top", fontsize=9, color="#c07800")

# ── Legend ────────────────────────────────────────────────────────────────────

legend_patches = [
    mpatches.Patch(facecolor="#2a5caa", edgecolor="#1a3c7a", alpha=0.65,
                   label=f"Car footprint ({CAR_LENGTH} m × {CAR_WIDTH} m)"),
    mpatches.Patch(facecolor="#f4a636", edgecolor="#c07800", alpha=0.6,
                   label=f"Occupied grid cell ({SPACING} m × {SPACING} m)"),
]
ax.legend(handles=legend_patches, loc="upper right",
          prop=font_manager.FontProperties(family="NewComputerModern08", size=10),
          framealpha=0.9)

# ── Axes ──────────────────────────────────────────────────────────────────────

ax.set_xlim(-0.3, GRID_W * SPACING + 0.3)
ax.set_ylim(-0.9, GRID_H * SPACING + 0.3)
ax.set_aspect("equal")
ax.set_axis_off()

fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")
