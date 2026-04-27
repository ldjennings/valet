"""Generate path_overlap.svg — illustrates why checking every Nth state suffices.

Shows car footprints along a short arc: all states drawn faintly, coarse-checked
states (every 4th) drawn with bold outlines. The dense overlap makes it visually
clear that consecutive footprints are nearly identical.

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
from matplotlib.patches import Polygon as MplPolygon
from shapely.affinity import rotate, translate
from shapely.geometry import box

OUT = Path(__file__).parents[1] / "photos" / "path_overlap.svg"

# ── Car geometry (mirrors config.py) ─────────────────────────────────────────

CAR_LENGTH  = 5.2
CAR_WIDTH   = 1.8
WHEELBASE   = 2.8
V           = 5.0       # m/s
DT          = 1 / 30    # seconds per timestep
COARSE_STEP = 4

def make_car_base():
    rect = box(-CAR_LENGTH / 2, -CAR_WIDTH / 2, CAR_LENGTH / 2, CAR_WIDTH / 2)
    rear_overhang = CAR_LENGTH - WHEELBASE
    offset = (CAR_LENGTH / 2) - rear_overhang
    return translate(rect, xoff=offset)

def arc_trajectory(x0, y0, h0, v, omega, n, dt):
    """Return list of (x, y, heading_deg) for n steps."""
    states = []
    if abs(omega) < 1e-9:
        for i in range(n + 1):
            t = i * dt
            states.append((x0 + v * t * math.cos(h0),
                            y0 + v * t * math.sin(h0),
                            math.degrees(h0)))
    else:
        R = v / omega
        for i in range(n + 1):
            h = h0 + omega * i * dt
            x = x0 + R * (math.sin(h) - math.sin(h0))
            y = y0 - R * (math.cos(h) - math.cos(h0))
            states.append((x, y, math.degrees(h)))
    return states

def footprint_patch(base, x, y, heading_deg, **kwargs):
    shape = rotate(base, heading_deg, origin=(0, 0))
    shape = translate(shape, xoff=x, yoff=y)
    xs, ys = shape.exterior.xy
    return MplPolygon(list(zip(xs, ys)), **kwargs)

# ── Generate trajectory ───────────────────────────────────────────────────────

# Moderate turn: 20° steer → omega = v * tan(delta) / wheelbase
delta = math.radians(20)
omega = V * math.tan(delta) / WHEELBASE   # ~0.65 rad/s, R ~7.7 m

N_STEPS = 20
states = arc_trajectory(0.0, 0.0, 0.0, V, omega, N_STEPS, DT)

base = make_car_base()

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(9, 4.5))

FILL_COLOR    = "#2a5caa"
COARSE_COLOR  = "#d4600a"

for i, (x, y, hdeg) in enumerate(states):
    is_coarse = (i % COARSE_STEP == 0)
    if is_coarse:
        # Bold outline, no fill — checked states
        patch = footprint_patch(base, x, y, hdeg,
                                facecolor="none",
                                edgecolor=COARSE_COLOR,
                                linewidth=1.6,
                                linestyle="-",
                                zorder=3)
    else:
        # Faint fill — unchecked states
        patch = footprint_patch(base, x, y, hdeg,
                                facecolor=FILL_COLOR,
                                edgecolor=FILL_COLOR,
                                alpha=0.08,
                                linewidth=0.4,
                                zorder=2)
    ax.add_patch(patch)

# Draw path centerline
path_xs = [s[0] for s in states]
path_ys = [s[1] for s in states]
ax.plot(path_xs, path_ys, color="#888888", linewidth=0.8,
        linestyle="--", zorder=1, label="_nolegend_")

# Mark checked state positions
coarse_xs = [states[i][0] for i in range(0, len(states), COARSE_STEP)]
coarse_ys = [states[i][1] for i in range(0, len(states), COARSE_STEP)]
ax.scatter(coarse_xs, coarse_ys, color=COARSE_COLOR, s=18, zorder=4,
           label="_nolegend_")

# ── Step-size annotation ──────────────────────────────────────────────────────

step_dist = V * DT  # meters per timestep
check_dist = step_dist * COARSE_STEP
ax.annotate("", xy=(states[COARSE_STEP][0], states[COARSE_STEP][1] - 1.6),
            xytext=(states[0][0], states[0][1] - 1.6),
            arrowprops=dict(arrowstyle="<->", color="#555", lw=1.0))
ax.text((states[0][0] + states[COARSE_STEP][0]) / 2,
        states[0][1] - 2.1,
        f"{check_dist:.2f} m between checks",
        ha="center", va="top", fontsize=9, color="#555")

# ── Legend ────────────────────────────────────────────────────────────────────

legend_patches = [
    mpatches.Patch(facecolor=FILL_COLOR, alpha=0.25, edgecolor=FILL_COLOR,
                   linewidth=0.4, label="Unchecked states"),
    mpatches.Patch(facecolor="none", edgecolor=COARSE_COLOR, linewidth=1.6,
                   label=f"Checked states (every {COARSE_STEP}th)"),
]
ax.legend(handles=legend_patches, loc="upper left",
          prop=font_manager.FontProperties(family="NewComputerModern08", size=10),
          framealpha=0.9)

# ── Axes ──────────────────────────────────────────────────────────────────────

ax.autoscale()
pad = 1.5
x0, x1 = ax.get_xlim()
y0, y1 = ax.get_ylim()
ax.set_xlim(x0 - pad, x1 + pad)
ax.set_ylim(y0 - pad * 1.5, y1 + pad)
ax.set_aspect("equal")
ax.set_axis_off()

# ── Save ──────────────────────────────────────────────────────────────────────

fig.tight_layout()
fig.savefig(OUT, format="svg", bbox_inches="tight")
print(f"Saved: {OUT}")
print(f"Step displacement: {step_dist:.3f} m  |  Check spacing: {check_dist:.3f} m  |  Car length: {CAR_LENGTH} m")
