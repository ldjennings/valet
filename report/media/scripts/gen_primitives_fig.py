"""Generate primitives_fig.svg — motion primitives emanating from a car state.

Two panels:
  Left:  all primitives (turning + straight)
  Right: straight (delta=0) primitives only, zoomed to show their length

Calls the real CarBot.propagate() so the figure exactly matches the planner.

Run from the repo root:
    python report/media/scripts/gen_primitives_fig.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[3] / "src"))

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager

_font_dir = Path(__file__).parents[2] / "newcm-8.0.0" / "otf"
for _variant in ("Regular", "Bold", "Italic", "BoldItalic"):
    font_manager.fontManager.addfont(str(_font_dir / f"NewCM10-{_variant}.otf"))
for _variant in ("Regular", "Italic"):
    font_manager.fontManager.addfont(str(_font_dir / f"NewCM08-{_variant}.otf"))
matplotlib.rcParams["font.family"] = "NewComputerModern10"
matplotlib.rcParams["font.size"] = 16

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry.polygon import Polygon as ShapelyPolygon

from bots.bots import CarBot
from bots.state import CarState
from bots.geometry import place
from planner.astar import HybridConfig

OUT = Path(__file__).parents[1] / "photos" / "primitives_fig.svg"

# ── Config ────────────────────────────────────────────────────────────────────

cfg   = HybridConfig()
bot   = CarBot()
state = CarState(rear_axle_x=0.0, rear_axle_y=0.0, heading_rad=math.radians(30))

primitives = bot.propagate(state, cfg.spacing, cfg.angular_spacing, cfg.steering_granularity)

FWD_COLOR = "#2a5caa"
REV_COLOR = "#d4600a"

def is_straight(traj):
    return abs(traj[-1].heading_rad - traj[0].heading_rad) < 0.01

def is_fwd(traj):
    xs = [s.rear_axle_x for s in traj]
    ys = [s.rear_axle_y for s in traj]
    return (xs[1] - xs[0]) * math.cos(traj[0].heading_rad) + \
           (ys[1] - ys[0]) * math.sin(traj[0].heading_rad) > 0

def draw_car(ax):
    car_geom = place(bot._base, state.rear_axle_x, state.rear_axle_y, state.heading_rad)
    xs, ys = car_geom.exterior.xy
    ax.add_patch(MplPolygon(list(zip(xs, ys)),
                             facecolor=FWD_COLOR, edgecolor="#1a3c7a",
                             alpha=0.4, linewidth=1.2))

def draw_primitives(ax, trajs):
    for traj in trajs:
        xs = [s.rear_axle_x for s in traj]
        ys = [s.rear_axle_y for s in traj]
        color = FWD_COLOR if is_fwd(traj) else REV_COLOR
        ax.plot(xs, ys, color=color, linewidth=1.4, alpha=0.85)
        ax.plot(xs[-1], ys[-1], "o", color=color, markersize=4)
    ax.plot(state.rear_axle_x, state.rear_axle_y, "k+", markersize=8, markeredgewidth=1.2)

def finish_ax(ax, title):
    ax.set_aspect("equal")
    ax.autoscale()
    _x0, _x1 = ax.get_xlim(); _xp = (_x1 - _x0) * 0.12
    _y0, _y1 = ax.get_ylim(); _yp = (_y1 - _y0) * 0.12
    ax.set_xlim(_x0 - _xp, _x1 + _xp)
    ax.set_ylim(_y0 - _yp, _y1 + _yp)
    ax.set_title(title, fontsize=14)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_axis_off()

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

all_trajs      = [traj for traj, _ in primitives]
straight_trajs = [t for t in all_trajs if is_straight(t)]

# Left: all primitives
draw_car(ax1)
draw_primitives(ax1, all_trajs)
finish_ax(ax1, "All primitives")

# Right: straight only
draw_car(ax2)
draw_primitives(ax2, straight_trajs)
finish_ax(ax2, "Straight primitives (δ = 0)")

# Shared legend
legend_patches = [
    mpatches.Patch(facecolor=FWD_COLOR, label="Forward"),
    mpatches.Patch(facecolor=REV_COLOR, label="Reverse"),
]
ax1.legend(handles=legend_patches,
           prop=font_manager.FontProperties(family="NewComputerModern08", size=11),
           loc="upper left")

fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")
