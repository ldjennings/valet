#import "template.typ"

#show link: underline


#set text(font: "New Computer Modern", size: 11pt)
#set math.equation(numbering: "(1)")

#set par(
  justify: true,
  leading: 0.65em,
)


#let code-block(body) = rect(
  fill: luma(245), // light grey background
  stroke: 0.5pt + luma(200), // subtle border
  radius: 4pt, // rounded corners
  inset: 8pt, // padding inside the box
  body,
)

= Valet Homework Assignment
== RBE 550 - Liam Jennings


=== Introduction
This assignment implements a motion planning simulator for four vehicle types of increasing kinematic complexity: a holonomic point robot, a differential drive, an Ackermann car, and a car towing a trailer. Each vehicle navigates a randomly generated obstacle field from a fixed start pose to a goal pose.

The planner uses a python implementation of the Hybrid A\* search algorithm, a variant of A\* that searches over a continuous $(x, y, theta)$ state space while using a discretised grid for duplicate detection.

Collision detection is built on #link("https://shapely.readthedocs.io/en/stable/")[shapely] polygon geometry with a broad-phase occupancy-grid filter and a pre-rotated heading cache to keep query cost low. After successfully finding a raw path to the goal pose, it is post-processed by probabilistic shortcutting and then resampled to a uniform velocity for smooth playback.


==== Usage

Dependencies are managed via `pyproject.toml`. A `Makefile` is provided for convenience:

```
make install   # create .venv and install all dependencies
make venv      # create .venv if needed, then print the activation command
make clean     # remove .venv, caches, build artifacts, and any recorded mp4s
```

After activating the environment, the simulator is launched with the `runsim` entry point:

```
runsim [bot_type] [options]
```

`bot_type` is one of `point`, `diff`, `car`, or `trailer` (default: `diff`). Available options:

```
-m / --manual    drive the bot manually with arrow keys instead of planning
-r / --record    save the simulation to recording.mp4
-s / --seed N    fix the RNG seed for a reproducible obstacle layout
```

For example, to run the trailer bot with a fixed seed:
```
runsim trailer -s 42
```

When no seed is provided, a random one is chosen and printed to the terminal at startup, allowing any run to be reproduced exactly with `-s`. The terminal also prints planner progress during the search — node expansion count, open set size, current cost, and position — as well as a summary on completion including the number of expansions, shortcuts applied during smoothing, and final path length after resampling.


==== Dependencies

The project uses the following notable external libraries:

- *#link("https://www.pygame.org/")[Pygame]* — provides the real-time 2-D rendering window, keyboard event handling for manual drive mode, and the simulation loop.

- *#link("https://shapely.readthedocs.io/en/stable/")[Shapely]* — computational geometry library used to represent robot footprints as polygons and query them against obstacle geometry via an STRtree spatial index.

- *#link("https://imageio.readthedocs.io/")[imageio] / imageio-ffmpeg* — used together to encode simulation recordings to MP4 when the `--record` flag is passed; imageio-ffmpeg supplies the FFmpeg backend.

- *reeds_shepp (pyReedsShepp)* @liespace_pyreedsshepp — Python bindings to a C implementation of Reeds-Shepp path length and curve computations. The original library #cite(<ghliu_pyreedsshepp>) targets an older Python version; a personal fork (stored as a git submodule at `deps/pyReedsShepp`) was created to update the Cython build configuration for compatibility with the current Python version.

=== Design Goals

Development was incremental: start with a working holonomic point robot (not required, but useful for isolating planner bugs), then add each vehicle type in order of complexity. Shapely was used for collision detection from the start as a correct-if-slow baseline, with optimisation deferred until the planner was working. Performance work was largely driven by profiling with `cProfile` and `snakeviz`.
// cool icicle plot

The code was also an experiment with modern Python's type system — `typing.Protocol` and generics were used to keep vehicle types interchangeable without inheritance, inspired by Rust traits. Interesting in practice, though probably not worth the overhead in the future.

The trailer was the most annoying part. There isn't closed-form path for an arbitrary truck-trailer start/goal the way Reeds-Shepp works for a car, so a Hybrid A\* approach with Euler-integrated trailer kinematics was used instead.



#pagebreak()
=== Approach


=== Obstacle Environment

// TODO: add a screenshot of the environment here

The environment is a fixed-size 2D grid with randomly placed axis-aligned obstacles, a fixed start and goal at opposite corners, and a printable RNG seed for reproducibility. Obstacles are stored as both a NumPy boolean grid for fast broad-phase rejection and a Shapely `STRtree` for exact intersection tests.


=== Collision Detection
// Demonstrate your collision checker implementation with annotated diagrams depicting overlap
// checks with a vehicle and obstacle region. Submit these diagrams individually, but also include
// them in your report.

The collision checker was designed to handle two geometry types: rotated rectangular footprints (for the robot body, truck, and trailer) and a line segment (the hitch bar connecting truck to trailer). Obstacles are axis-aligned grid cells, stored as both a numpy boolean array for fast grid lookups and a shapely `STRtree` of `box` polygons for exact intersection tests.

#link("https://shapely.readthedocs.io/en/stable/")[shapely] is designed for general topological geometry analysis and is not inherently optimised for repeated per-frame queries against a fixed obstacle set. Naively calling intersects on individual translated geometries at every state check proved too slow for the planner's inner loop. Three optimisations were applied to address this.

==== Heading cache.
Rotating or translating a Shapely geometry is expensive. To exchange strict accuracy for performance, each base shape is pre-rotated at 72 evenly-spaced headings (every $5 degree$) at construction time. Per-state footprint queries look up the nearest pre-rotated shape and record only the $(x, y)$ offset, deferring translation until an exact check is actually needed.


==== State validation.
The state validator applies checks in order of increasing cost, exiting as soon as any check fails.

#let validator = code-block()[
  ```
  function IS_VALID_STATE(geometries):
      for each geometry in geometries:
          if geometry is a line segment:
              if any endpoint lies outside the environment boundary:
                  return false
              if the segment intersects an obstacle:
                  return false
          else: // Geometry is a rotated rectangle
              compute translated axis-aligned bounding box
              if bounding box lies outside the environment boundary:
                  return false
              if no obstacle can possibly overlap the bounding box:
                  continue  // broad-phase rejection
              compute translated geometry
              if geometry intersects any obstacle:
                  return false
      return true
  ```
]

#figure(validator, caption: "Pseudocode describing state validation process.")

For rectangular footprints: the translated axis-aligned bounding box (AABB) is checked against the environment boundary first. If it lies outside, the state is immediately rejected without touching any geometry. If it lies inside, the AABB is mapped to grid cell indices and the corresponding subgrid slice is checked for any occupied cel. This eliminates the majority of valid states at negligible cost. The geometry is only translated and tested exactly against the `STRTree` if the broad phase reports a possible collision.

===== Line segment handling.
The trailer's hitch bar cannot be represented as a box, so it is stored as raw endpoints rather than a Shapely geometry. Containment is checked by testing both endpoints against the boundary. Obstacle intersection uses Bresenham's line algorithm against an occupancy grid, stepping cell-by-cell along the segment and returning on the first occupied cell.

==== Path-level validation.
During planning, entire primitive trajectories must be validated, not just individual states. A two-phase approach is used: first, the last state and every 4th intermediate state are checked using the approximate (cached, untranslated) footprints. Most invalid primitives are caught here. If that passes, and fine collision checks are enabled, the remaining states are checked with exact footprints.

According to cProfile, STRtree intersection calls account for approximately 5% of hybrid A\* runtime after these optimisations, with the dominant cost shifted to state expansion and heuristic evaluation.







=== Planner Implementation
==== Hybrid A\*

The planner uses Hybrid A, a variant of A that operates over a continuous state space rather than a discrete grid. States are represented as continuous $(x, y, theta)$ (and $phi$ for the trailer), but duplicate detection uses a discretised grid: XY position is rounded to the nearest spacing multiple, and headings are floor-divided into fixed angular bins of `angular_spacing` size. A node is only expanded once per discrete cell, preventing the search from cycling while still allowing the robot to reach any continuous pose within a cell.

Neighboring cells are generated from a node by sampling a subset of the possible control inputs, and then simulating the kinematics resulting from them.

The heuristic used is the Reeds-Shepp path length from the current state to the goal, ignoring obstacles. This is admissible and significantly tighter than Euclidean distance for non-holonomic robots, as it respects the turning radius constraint.

#let astar_block = code-block()[
```
function HYBRID_ASTAR(start, goal):
    push start onto open heap
    while open heap is not empty:
        node ← pop lowest f = g + h node
        if node already visited: continue
        mark node visited
        if node is within terminal range of goal:
            path ← LAST_SHOT_CONNECTION(node, goal)
            if path is valid and collision-free:
                return POSTPROCESS(path)
        for each primitive in PROPAGATE(node.state):
            if primitive is collision-free:
                push endpoint onto heap with updated g cost
    return failure
```
]


#figure(astar_block, caption: "hybrid A* pseudocode")

==== Primitive Generation
At each node, the bot generates a set of motion primitives by sampling control inputs. For car-like and trailer vehicles, this means sampling steering angles $delta$ evenly between $-delta_max$ and $+delta_max$, combined with forward and reverse speed. For differential drives, different angular velocities $omega$ are instead sampled. Each pair is integrated over $n$ timesteps using an analytic arc trajectory formula #footnote[For derivation, see #ref(<arc_derivation>, supplement: [])] to avoid computational cost and error accumulation:

$
theta (i) &= theta_0 + omega dot i dot d t \
x(i) &= x_0 + R dot (sin(theta (i)) - sin(theta_0)) \
y(i) &= y_0 - R dot (cos(theta (i)) - cos(theta_0))
$ <eq:arc_trajectory_formula>

where $R = nu / omega$, and $i$ is the $i$-th timestep of $d t$ during the turn. Although the Ackermann and differential drive models differ in how $omega$ is produced, both can be reduced to the same unicycle model. Therefore, both kinds of vehicles may be modeled using the above formula.

The step count $n$  is calculated to satisfy two constraints:

$ n_nu >= frac("spacing", |nu| dot  d t), quad n_omega >= frac("angular_spacing", |omega| dot d t) $

The final value of $n$ is simply $max(n_nu, space n_omega)$. Satisfying these constraints guarantees every primitive lands in different $(x,y)$ cell and heading bin compared to the initial cell.

The cost of each primitive (and therefore edge in the A\* graph) is the arc length of the primitive plus a weighted heading change, with an additive penalty for reversals to discourage the planner from exploring backwards.


Two vehicle-specific exceptions apply. The differential drive appends additional rotate-in-place primitives with $v = 0$. These primitives have zero arc length, their cost relying purely on heading change, allowing the planner to reason about in-place rotation as a distinct maneuver.

The trailer's primitive generation  is more involved. The truck follows the same analytic arc as any other Ackermann vehicle, but the trailer heading $phi$ is coupled to the truck via a nonlinear ODE:

$ dot(phi) = frac(v, M) sin(theta - phi) $


where $M$ is the hitch-to-trailer-axle distance. This cannot be integrated in closed form alongside the truck arc, so $phi$ is propagated sequentially: the truck positions and headings are computed analytically in one pass, then $phi$ is stepped forward through the resulting sequence using Euler's method.

Paths that result in the trailer jackknifing #footnote()[When a vehicle towing a trailer loses control, causing the trailer to swing out and form an acute angle with the towing vehicle, resembling a folding pocket knife.] are treated as invalid: each step is checked against a jackknife limit via the condition $|theta - phi| < 90 degree$. If the limit is exceeded, the primitive is discarded.

==== Last-Shot Connection

When a node falls within a euclidean distance of `terminal_radius` of the goal pose, a closed-form trajectory ignorant of obstacles is attempted directly to the goal rather than continuing to expand primitives. For car and trailer bots this is a Reeds-Shepp path; for the differential drive bot it is a rotate-drive-rotate sequence.

This avoids the difficulty of landing exactly on the goal pose through discrete primitives, and is the mechanism by which the planner achieves precise heading alignment at the goal.

The attempted path is validated for collisions before being accepted. For the trailer, the trailer heading is simultaneously integrated along the RS path and the attempt is rejected if jackknifing occurs. As the planner is unable to directly control the trailer heading, it rejects attempts if the trailer heading at the goal is within a set tolerance.


==== Post-Processing

The raw path from the search is passed through two steps before playback.

Smoothing applies probabilistic shortcutting: two random indices are chosen, a direct connection is attempted via `generate_trajectory`, and if collision-free it replaces the span. This is repeated for 100 iterations and typically removes significant detours introduced by the discrete primitive structure. This step is done by the function `smooth_path`.

Resampling converts the variable-density path into uniform arc-length samples so the animation plays back at a constant velocity. Pure-rotation segments (zero XY displacement) are detected separately and resampled at a fixed angular rate. This step is done by the function `resample_path`.

=== Results <results>

// TODO: screenshots of each vehicle type navigating (point, diff, car, trailer)

The planner works well across all four vehicle types. The trailer in particular produces some satisfying paths — watching it navigate tight spaces while keeping the trailer heading under control is a good demonstration that the coupled kinematics are being handled correctly. Performance ended up in a reasonable place after profiling, making iteration much faster than it was early on. The codebase also ended up in a good state structurally; having a clean interface between vehicle types made it easy to experiment with planner parameters and add features without things breaking unexpectedly.

==== Areas for Improvement

- *Obstacle-aware heuristic.* The Reeds-Shepp heuristic ignores obstacles, which can cause the planner to underestimate costs in cluttered environments. Augmenting it with a precomputed 2D Dijkstra cost-to-go map from the goal would give tighter estimates and likely reduce node expansions.

- *State representation boundary.* Throughout the planner, full typed state objects are used rather than raw float arrays or tuples. This made the logic clean and easy to reason about, but there is a real performance cost — NumPy operations on raw arrays would be substantially faster than constructing and passing around dataclass instances in the planner's inner loop. This was a conscious tradeoff in favour of correctness and clarity over speed.

- *Dataclass performance.* Switching from standard Python dataclasses to a library like `msgspec` could reduce the overhead of creating large numbers of short-lived state objects during search.

- *Full SAT-based collision.* Shapely is still used for the narrow-phase intersection checks. Replacing it entirely with raw NumPy checks using the Separating Axis Theorem would remove the last external geometry dependency and likely be faster. The current version is good enough, but it's a loose end.

- *Planner visualisation.* It would be nice to watch Hybrid A\* progress in real time — drawing expanded nodes, the open set, and the current best path as the search runs. The current sim only shows the final result. This wasn't a priority, but it would make debugging and tuning much more intuitive.

#pagebreak()


#bibliography("refs.bib", style: "ieee")

#template.appendix()[
    = Derivation of @eq:arc_trajectory_formula <arc_derivation>

    Both the differential drive and Ackermann kinematics can be simplified to the unicycle kinematic model @lav2006[Section 13.1.2.3]:

    $ dot(x) &= nu dot cos(theta) \  dot(y) &= nu dot sin(theta)  \ dot(theta)  &= omega $

    The differential drive model is directly equivalent, while Ackermann steering has a dependency between the velocity $nu$ and steering angle $delta$:

    $ omega = nu dot tan(delta) / "wheelbase" $ <eq:ackermann_omega>

   When $nu$ and $omega$ are held constant, the heading evolves linearly as $ theta(t)= theta_0 + omega$, reducing the position integrals to standard trigonometric forms with known closed-form solutions.

    Treating this as an initial value problem with state $(x_0, y_0, theta_0)$ at $t = 0$, the heading integrates directly, as seen above:

    $ theta(t)= theta_0 + omega dot t  $

    Substituting into the position equations and integrating:

    $

    x(t) &= x_0 + integral_0^t nu dot cos(theta_0 + omega dot tau) space d tau = x_0 + lr(frac(v, omega) dot sin(theta_0 + omega dot tau) |)_(tau=0)^(tau=t) \

    y(t) &= y_0 + integral_0^t nu dot sin(theta_0 + omega dot tau) space d tau = y_0 - lr(frac(v, omega) dot cos(theta_0 + omega dot tau) |)_(tau=0)^(tau=t)

    $

    The radius of the circle traced by this arc can be found through the equation $R = L dot tan(delta)$ @lav2006[Section 13.1.2.1]. If we look back at @eq:ackermann_omega, this allows us to substitute $R = nu / omega$:

    $
    vec(x(t) & = x_0 + R dot (sin(theta(t)) - sin(theta_0)) , y(t) & = y_0 - R dot (cos(theta(t)) - cos(theta_0)), delim: "[", gap: #1em)
    $

    This can be extended to include the case of $omega approx 0 => R = v / 0$, where the arc approaches a straight line as $R -> infinity$. Intuitively, the kinematics of this case can be easily formulated:
    $
      vec(x(t) & = x_0 + nu dot t dot cos(theta_0), y(t) & = y_0 + nu dot t dot sin(theta_0), theta(t) & = theta_0, delim: "[", gap: #1em)
    $

    However, besides $theta(t)$, $x(t)$ and $y(t)$ must be formally derived via L'Hopital's rule:
    $
      lim_(nu->0) (nu dot (sin(theta_0 + omega dot t) - sin(theta_0))) / omega = 0 / 0 = lim_(w->0) (nu dot cos(theta_0  + 0 dot t) dot t - 0) / 1 = v dot t dot cos(theta_0)
    $

    $
        lim_(nu->0) (nu dot (cos(theta_0 + omega dot t) - cos(theta_0))) / omega = 0 / 0 = lim_(nu->0) (nu dot -sin(theta_0 + 0 dot t) dot t - 0) / 1 = -v dot t dot sin(theta_0)
    $

    which can be substituted back into the arc equations to get the straight line equations. Stationary ($nu = 0, abs(omega) > 0$) turns do not suffer from this instability, however this would require an ackermann car to have a steering angle of $90 degree$, which would cause $tan(delta) -> infinity$ (@eq:ackermann_omega).
]
