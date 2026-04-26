"""Generate primitives_fig.svg — motion primitives emanating from a car state.

Calls the real CarBot.propagate() so the figure exactly matches the planner.

Run from the repo root:
    python report/media/scripts/gen_primitives_fig.py
"""

import math
import sys
from pathlib import Path

# ── add src to path so we can import bot/planner code ─────────────────────────
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
matplotlib.rcParams["font.size"] = 11

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon

from bots.bots import CarBot
from bots.state import CarState
from bots.geometry import place
from planner.astar import HybridConfig

OUT = Path(__file__).parents[1] / "photos" / "primitives_fig.svg"

# ── Config (matches planner defaults) ─────────────────────────────────────────

cfg    = HybridConfig()
bot    = CarBot()
state  = CarState(x=0.0, y=0.0, heading_rad=math.radians(30))

# ── Generate primitives ────────────────────────────────────────────────────────

primitives = bot.propagate(state, cfg.spacing, cfg.angular_spacing, cfg.steering_granularity)

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(6, 6))

FWD_COLOR = "#2a5caa"
REV_COLOR = "#d4600a"
SPEED     = bot.SPEED

for traj, _ in primitives:
    xs = [s.rear_axle_x for s in traj]
    ys = [s.rear_axle_y for s in traj]
    # determine forward vs reverse from first step displacement
    fwd = (xs[1] - xs[0]) * math.cos(traj[0].heading_rad) + \
          (ys[1] - ys[0]) * math.sin(traj[0].heading_rad) > 0
    color = FWD_COLOR if fwd else REV_COLOR
    ax.plot(xs, ys, color=color, linewidth=1.4, alpha=0.85)
    # dot at endpoint
    ax.plot(xs[-1], ys[-1], "o", color=color, markersize=4)

# Draw car footprint at start state
car_geom = place(bot._base, state.rear_axle_x, state.rear_axle_y, state.heading_rad)
xs, ys = car_geom.exterior.xy
car_patch = MplPolygon(list(zip(xs, ys)),
                        facecolor="#2a5caa", edgecolor="#1a3c7a",
                        alpha=0.5, linewidth=1.2)
ax.add_patch(car_patch)

# Mark rear axle origin
ax.plot(state.rear_axle_x, state.rear_axle_y, "k+", markersize=8, markeredgewidth=1.2)

# ── Legend / labels ───────────────────────────────────────────────────────────

legend_patches = [
    mpatches.Patch(facecolor=FWD_COLOR, label="Forward"),
    mpatches.Patch(facecolor=REV_COLOR, label="Reverse"),
]
ax.legend(handles=legend_patches,
          prop=font_manager.FontProperties(family="NewComputerModern08", size=8))

ax.set_aspect("equal")
ax.autoscale()
_x0, _x1 = ax.get_xlim(); _xp = (_x1 - _x0) * 0.1
_y0, _y1 = ax.get_ylim(); _yp = (_y1 - _y0) * 0.1
ax.set_xlim(_x0 - _xp, _x1 + _xp)
ax.set_ylim(_y0 - _yp, _y1 + _yp)
ax.set_xticks([])
ax.set_yticks([])
ax.set_axis_off()

fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")
