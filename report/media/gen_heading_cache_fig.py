"""Generate heading_cache.svg — two-panel figure illustrating the heading cache.

Left panel:  12 pre-rotated car shapes (every 30°) overlaid at the origin,
             showing what the cache conceptually contains.
Right panel: Worst-case approximation error — true shape at 22.5° vs the
             nearest cached shape at 20°, with the discrepancy region shaded.

Run from any directory; output is written next to this script.
"""

import math
import sys
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
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
from shapely.affinity import rotate, translate
from shapely.geometry import box
from shapely.ops import unary_union

OUT = Path(__file__).with_name("photos") / "heading_cache.svg"

# ── Car geometry (mirrors geometry.py / config.py) ───────────────────────────

WHEELBASE   = 2.8   # m
CAR_LENGTH  = 5.2   # m
CAR_WIDTH   = 1.8   # m

def make_car_base():
    rect = box(-CAR_LENGTH / 2, -CAR_WIDTH / 2, CAR_LENGTH / 2, CAR_WIDTH / 2)
    rear_overhang = CAR_LENGTH - WHEELBASE
    offset = (CAR_LENGTH / 2) - rear_overhang
    return translate(rect, xoff=offset)

def rotated(base, deg):
    return rotate(base, deg, origin=(0, 0))

def shapely_to_patch(geom, **kwargs):
    xs, ys = geom.exterior.xy
    return MplPolygon(list(zip(xs, ys)), **kwargs)

# ── Build shapes ──────────────────────────────────────────────────────────────

base = make_car_base()

# Left panel: every 120 deg (3 shapes)
left_shapes = [rotated(base, i * 120) for i in range(3)]

# Right panel: worst-case is 2.5° off; pick 22.5° (between cached 20° and 25°)
TRUE_DEG   = 22.5
CACHED_DEG = 20.0   # nearest 5° increment (round(22.5/5)*5 = 20)

true_shape   = rotated(base, TRUE_DEG)
cached_shape = rotated(base, CACHED_DEG)

# Discrepancy regions
only_true   = true_shape.difference(cached_shape)    # missed by cache (false negative risk)
only_cached = cached_shape.difference(true_shape)    # extra coverage (false positive)
overlap     = true_shape.intersection(cached_shape)

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7, 3.5))

# ── Left panel ───────────────────────────────────────────────────────────────

ALPHA = 0.18
EDGE  = "#2a5caa"

for shape in left_shapes:
    patch = shapely_to_patch(shape, facecolor=EDGE, edgecolor=EDGE,
                              alpha=ALPHA, linewidth=0.6)
    ax1.add_patch(patch)

# Highlight one example shape (0°)
highlight = shapely_to_patch(left_shapes[0], facecolor="none",
                              edgecolor=EDGE, linewidth=1.8, linestyle="--")
ax1.add_patch(highlight)

# Heading arrows — point in the forward direction of each cached shape
ARROW_LEN = 2.0
COLORS = ["#e05c00", "#2a5caa", "#228b22"]  # one per shape
for i, deg in enumerate([i * 120 for i in range(len(left_shapes))]):
    rad = math.radians(deg)
    dx, dy = ARROW_LEN * math.cos(rad), ARROW_LEN * math.sin(rad)
    ax1.annotate("", xy=(dx, dy), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=COLORS[i], lw=1.6))
    ax1.text(dx * 1.18, dy * 1.18, f"{deg}°", color=COLORS[i],
             ha="center", va="center")


ax1.set_aspect("equal")
ax1.autoscale()
_x0, _x1 = ax1.get_xlim()
_xpad = (_x1 - _x0) * 0.1
ax1.set_xlim(_x0 - _xpad, _x1 + _xpad)
ax1.set_title("Cached Rotations (every 120° shown)\n", fontsize=12)
ax1.set_xlabel("")
ax1.set_ylabel("")
ax1.set_xticks([])
ax1.set_yticks([])
ax1.grid(False)
ax1.set_axis_off()

# ── Right panel ───────────────────────────────────────────────────────────────

def add_shapely(ax, geom, **kwargs):
    if geom.is_empty:
        return
    geoms = list(geom.geoms) if geom.geom_type.startswith("Multi") else [geom]
    for g in geoms:
        if g.geom_type == "Polygon" and not g.is_empty:
            patch = shapely_to_patch(g, **kwargs)
            ax.add_patch(patch)

# Overlap region — transparent

# Extra coverage (cached but not true) — solid blue
add_shapely(ax2, only_cached,
            facecolor="#2a5caa", edgecolor="none", alpha=0.85)

# Missed region (true but not cached) — solid orange
add_shapely(ax2, only_true,
            facecolor="#d4600a", edgecolor="none", alpha=0.85)

# Outlines only — drawn on top so they're visible over the fills
add_shapely(ax2, cached_shape,
            facecolor="none", edgecolor="#2a5caa", linewidth=1.0, linestyle="-")

add_shapely(ax2, true_shape,
            facecolor="none", edgecolor="#d4600a", linewidth=1.0, linestyle="-")


ax2.set_aspect("equal")
ax2.set_xlim(ax1.get_xlim())
ax2.set_ylim(ax1.get_ylim())
ax2.set_title(f"Worst-case error: \nTrue Shape at {TRUE_DEG}° vs Cached Shape at {CACHED_DEG:.0f}°", fontsize=12)
ax2.set_xlabel("")
ax2.set_ylabel("")
ax2.set_xticks([])
ax2.set_yticks([])
ax2.grid(False)
ax2.set_axis_off()

legend_patches = [
    mpatches.Patch(facecolor="none", edgecolor="#555", linewidth=1.2, label="Overlap — always correct"),
    mpatches.Patch(facecolor="#2a5caa", alpha=0.85, label="Falsely blocked — cached only\n(obstacle here → false collision)"),
    mpatches.Patch(facecolor="#d4600a", alpha=0.85, label="Falsely cleared — true only\n(obstacle here → missed collision)"),
]
ax2.legend(handles=legend_patches, loc="upper left",
           prop=font_manager.FontProperties(family="NewComputerModern08", size=7),
           title_fontsize=7)

# ── Save ─────────────────────────────────────────────────────────────────────

fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")
