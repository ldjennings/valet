#import "template.typ": code-block, appendix, template, title-page

#show: template

#title-page(
  title: "Assignment 4: Valet",
  course: "RBE 550 — Motion Planning",
  author: "Liam Jennings",
  date: "April 2026",
)


== Introduction
This project implements a motion planning simulator for four vehicle types of increasing kinematic complexity: a holonomic point robot, a differential drive, an Ackermann car, and a car towing a trailer. Each vehicle navigates a randomly generated obstacle field from a fixed start pose to a goal pose.

The planner uses a python implementation of the Hybrid A\* search algorithm, a variant of A\* that searches over a continuous $(x, y, theta)$ state space while using a discretised grid for duplicate detection. The planner explores this grid by generating circular arc motion primitives, verifying that they do not collide with any obstacles. When within a set range of the goal, the planner occasionally attempts to plan a direct path while exploring nodes. If this path is verified to be collision free, the planner reconstructs a kinematically correct path by traversing its tree back to the start pose.

Collision detection is built on #link("https://shapely.readthedocs.io/en/stable/")[shapely] polygon geometry. Shapely is slow for repeated small queries, so a broad-phase occupancy-grid filter and a pre-rotated heading cache were implemented to keep query cost low.
After successfully finding a raw path to the goal pose, it is post-processed by probabilistic shortcutting and then resampled to a uniform velocity for smooth playback.
#figure(
    image("media/photos/trailer_sim.png", width: 70%),
    caption: [Trailer simulation mid-playback. The traced path follows the center of the rear axle for Ackermann and trailer vehicles, and the geometric center for differential drive and point robots. Blue dots mark the $(x, y)$ position of every state expanded during the planning phase.]
)


== Setup
=== Installation

The project was originally built with *Python 3.12*. The `reeds_shepp` dependency (`deps/pyReedsShepp`) also includes a C++ extension that requires the Boost headers at build time. Install them before setting up the environment:

- *Ubuntu/Debian:* `sudo apt install libboost-dev`
- *Arch:* `sudo pacman -S boost`
- *macOS:* `brew install boost`

The recommended way to install dependencies is with #link("https://docs.astral.sh/uv/")[uv]:

```sh
# install uv (once, system-wide)
curl -LsSf https://astral.sh/uv/install.sh | sh

uv sync          # installs Python 3.12 + all dependencies
uv run runsim    # run the simulator
```

Alternatively, a `Makefile` is provided for manual setup if Python 3.12 is already available. It also provides additional tools for managing the project:

```sh
make pip-install   # create .venv and install all dependencies via pip
make venv          # create .venv if needed, then print the activation command
make clean         # remove .venv, caches, build artifacts, and any recorded mp4s
```
=== Usage

After activating the environment, the simulator is launched with the `runsim` entry point:

```sh
runsim [bot_type] [options]
```

`bot_type` is one of `point`, `diff`, `car`, or `trailer` (default: `diff`). Available options:

```
-m / --manual    drive the bot manually with arrow keys instead of planning
-r / --record    save the simulation to recording.mp4
-s / --seed N    fix the RNG seed for a reproducible obstacle layout
```

While the simulation is running, pressing `s` saves a screenshot to the current directory (distinct from the `-s` / `--seed` launch flag).

For example, to run the trailer bot with a fixed seed:
```
runsim trailer -s 42
```

When no seed is provided, a random one is chosen and printed to the terminal at startup, allowing any run to be reproduced with `-s`. The terminal also prints planner progress during the search - node expansion count, open set size, current cost and position - as well as a summary on completion including the total number of expansions, shortcuts applied during smoothing, and final path length after resampling.


=== Dependencies

The project uses the following notable external libraries:

- *#link("https://www.pygame.org/")[Pygame]* — provides the real-time 2-D rendering window, keyboard event handling for manual drive mode, and the simulation loop.

- *#link("https://shapely.readthedocs.io/en/stable/")[Shapely]* — computational geometry library used to represent robot footprints as polygons and query them against obstacle geometry via an STRtree #footnote[A spatial index structure for fast nearest-neighbour queries, in the same vein as an Octree] spatial index.

- *#link("https://imageio.readthedocs.io/")[imageio] / imageio-ffmpeg* — used together to encode simulation recordings to MP4 when the `--record` flag is passed; imageio-ffmpeg supplies the FFmpeg backend.

- *reeds_shepp (pyReedsShepp)* @liespace_pyreedsshepp — Python bindings to a C implementation of Reeds-Shepp path length and curve computations. The original library #cite(<ghliu_pyreedsshepp>) targets an older Python version; a personal fork (stored as a git submodule at `deps/pyReedsShepp`) was created to update the Cython build configuration for compatibility with the Python version used for the project.

== Design Goals

Development was incremental: start with a working holonomic point robot (not required, but useful for isolating planner bugs), then add each vehicle type in order of complexity. Shapely was used for collision detection from the start as a correct-if-slow baseline, with optimisation deferred until the planner was working. Performance work was largely driven by profiling with `cProfile` and `snakeviz`.
#figure(
    image("media/photos/icicle.svg", width: 115%),
    caption: "Icicle plot rendering of profiled trailer simulation. Profiling shows time is split almost equally between primitive generation (propagate, 10.6 s) and collision checking (validate_path, 10.3 s), with is_valid_state accounting for 7.2 s of that collision budget."
)

The code was also an experiment with modern Python's type system — `typing.Protocol` and generics were used to keep vehicle types interchangeable without inheritance, inspired by Rust traits. Interesting in practice, though probably not worth the overhead in the future.



#pagebreak()
= Approach


== Obstacle Environment Implementation

#figure(
    image("media/photos/obstacle_env.png",width: 80%),
    caption: [Obstacles generated given a 10% occupation requirement. Captured while driving the car in manual mode.]
)

The environment consists of a fixed-size 2D grid with randomly placed axis-aligned obstacles, a fixed start and goal at opposite corners, and a printable RNG seed for reproducibility. Obstacles are stored as both a NumPy boolean grid for fast broad-phase rejection and a Shapely `STRtree` for exact intersection tests.

Obstacles are generated by randomly placing tetrominoes throughout the grid, until a set proportion of the grid is occupied by obstacles. The standard proportion is 10%. The vehicles (green rectangle) start in the top left, and attempts to navigate to the goal pose (yellow rectangle) in the bottom right. The starting and ending positions are always cleared.

#figure(
    grid(
        columns: 2,
        gutter: 1em,
        image("media/photos/bot_goals/goal_pose_point.png"),
        image("media/photos/bot_goals/goal_pose_diff.png"),
        image("media/photos/bot_goals/goal_pose_car.png"),
        image("media/photos/bot_goals/goal_pose_trailer.png"),
    ),
    caption: [Goal poses for each vehicle type: point robot (top left), differential drive (top right), Ackermann car (bottom left), trailer (bottom right).]
)

=== Car Goal Offset

Of specific note, the car's goal pose is offset further from the bottom edge than the other vehicles. Where the other vehicle goal poses are set 1 cell from the bottom edge, the car's is instead set 1.425 from that edge #footnote[Goal positions are specified as an offset from the grid edge passed to `grid_to_coords`, which adds an additional 0.5 to center within the cell; the true geometric distance is therefore offset - 0.5 cells. Other vehicles use an offset of 1 (0.5 cells geometric distance); the car uses 1.425 (0.925 cells).]. This decision was made to allow the goal position to be reachable: without it, the planner was unable to reach the goal at the standard offset.

Initial coarse tests suggested the feasibility cutoff was around 1.2 grid cells from the bottom edge, but this was an artifact of the #link(<heading_cache>)[heading cache] rounding to 5° increments producing false positives. Systematic testing with fine collision checking found the true cutoff: an offset of 1.38 exhausts the search space without finding a path, while 1.39 succeeds. Above an offset of 1.44, the car no longer needs to make the parallel parking maneuver to reach the goal. The final offset of 1.425 was chosen to sit within this window, ensuring the goal is reliably reachable while preserving the parallel parking maneuver.

#figure(
    grid(
        columns: 2,
        gutter: 1em,
        image("media/photos/car_offset_tuning/1_38-offset-fail-30004.png"),
        image("media/photos/car_offset_tuning/1_39-offset-pass-5719.png"),
    ),
    caption: [Offset 1.38 (left) exhausts the search after 30,004 expansions without finding a path. Offset 1.39 (right) succeeds in 5,719 expansions, confirming the feasibility cutoff.]
)

#figure(
    image("media/photos/car_offset_tuning/1_44-offset-pass-2436-noparallel.png", width: 80%),
    caption: [At offset 1.44 the car reaches the goal in 2,436 expansions without a parallel parking maneuver, establishing the upper bound of the target window.]
)


== Collision Detection
// Demonstrate your collision checker implementation with annotated diagrams depicting overlap
// checks with a vehicle and obstacle region. Submit these diagrams individually, but also include
// them in your report.

The collision checker was designed to handle two geometry types: rotated rectangular footprints (for the robot body, truck, and trailer) and a line segment (the hitch bar connecting truck to trailer). Obstacles are axis-aligned grid cells, stored as both a numpy boolean array for fast grid lookups and a shapely `STRtree` of `box` polygons for exact intersection tests.

#link("https://shapely.readthedocs.io/en/stable/")[Shapely] is designed for general topological geometry analysis and is not inherently optimised for repeated per-frame queries against a fixed obstacle set. Naively calling `intersects` on individually translated geometries at every state check proved too slow for the planner's inner loop. Three optimisations were applied to address this, and their combined effect is quantified in @fig:collision_opt.

=== Heading cache <heading_cache>

Rotating a Shapely geometry is expensive: internally, rotation calls `_affine_coords`, which walks every vertex of the polygon through a matrix multiply. To eliminate this per-query cost, each base shape is pre-rotated at 72 evenly-spaced headings (every $5°$) at construction time. Per-state footprint queries look up the nearest pre-rotated shape and record only the $(x, y)$ offset, deferring translation until an exact check is actually needed.

#figure(
    image("media/photos/heading_cache.svg", width: 100%),
    caption: [
        *Left:* three of the 72 pre-rotated car footprints stored in the heading cache (shown at 120$degree$ intervals for clarity; the full cache covers every 5$degree$). Arrows indicate the forward direction of each cached shape. *Right:* worst-case approximation error — the true footprint at 22.5$degree$ versus the nearest cached shape at 20$degree$, the maximum possible snap error being 2.5$degree$. Errors only occur when an obstacle intersects exclusively one discrepancy region (\~3.5% of vehicle area each); obstacles within the shared overlap are correctly detected by both shapes.
    ],
) <fig:heading_cache>

As seen in @fig:collision_opt, this nearly eliminates `rotate` entirely (974,294 calls $->$ 3,622), and drives a #box[65$times$] reduction in `_affine_coords` calls (1,948,445 $->$ 29,917), representing the single largest source of time saved across both optimisations.

=== State validation

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

#figure(validator, caption: [Pseudocode describing state validation process.])

For rectangular footprints, the translated axis-aligned bounding box (AABB) is checked against the environment boundary first. If it lies outside, the state is immediately rejected without touching any geometry. If it lies inside, the AABB is mapped to grid cell indices and the corresponding subgrid slice is checked for any occupied cell via `_has_possible_collision`. This eliminates the majority of states at negligible cost as the geometry is only translated and tested exactly against the `STRtree` (via `intersects_any`) if the broad phase reports a possible collision.
#figure(
    image("media/photos/aabb_check.svg", width: 80%),
    caption: [
        Broad-phase AABB collision check. The car footprint (blue) is rotated at 35°; its axis-aligned bounding box (orange, dashed) is mapped to grid cell indices and only the highlighted cells are queried for obstacles. The purple obstacle falls inside the AABB and triggers an exact Shapely intersection check, but does not intersect the footprint, so the state is accepted as free. The grey obstacles are outside the AABB and are therefore skipped entirely. This filter is conservative, but it catches the vast majority of states considered without requiring more expensive tests.
    ],
) <fig:aabb_check>

The effect is visible in @fig:collision_opt: `_has_possible_collision` appears only in the optimised profile (935,415 calls), while `intersects_any` — the expensive Shapely intersection — drops #box[41$times$] from 938,273 to 23,006 calls. Crucially, `is_valid_state` is called roughly the same number of times in both profiles (487,071 vs 485,689), confirming that the AABB filter does not reduce collision accuracy, instead avoiding exact checks on states that are clearly free.

The combined effect of these two optimisations is a #box[3.98$times$] end-to-end speedup (81.5 s $->$ 20.5 s). For reference, `propagate` — which handles motion primitive generation and is unaffected by the collision optimisations — shows virtually identical call counts and cost in both profiles, confirming the speedup is attributable entirely to the collision checker.

#figure(
    image("media/photos/collision_opt_comparison.svg", width: 100%),
    caption: [Call counts (*above*) and exclusive CPU time (*below*) for key , comparing the unoptimised and optimised collision checker. Total runtime: 81.5 s (unoptimised) vs 20.5 s (optimised), a #box[3.98$times$] speedup. `propagate` is included as a control to show that primitive generation cost is largely unchanged between runs.]
) <fig:collision_opt>

=== Line segment handling
The trailer's hitch bar connects the truck rear to the trailer's front axle. In this project, this is simplified to a line from the center of the rear axle of the truck to the trailers front axle. This cannot be represented as a box, so it is stored as raw endpoints rather than a Shapely geometry. Containment is checked by testing both endpoints against the boundary. Obstacle intersection uses Amanatides and Woo's fast voxel traversal algorithm @10.2312:egtp.19871000 against an occupancy grid, stepping cell-by-cell along the segment and returning on the first occupied cell.

=== Path-level validation
During planning, entire primitive trajectories must be validated, not just individual states. A two-phase approach is used: first, the last state and every 4th intermediate state are checked using the approximate (cached, untranslated) footprints. Most invalid primitives are caught here. If that passes, and fine collision checks are enabled, the remaining states are checked with exact footprints.






== Planner Implementation
=== Hybrid A\*

The planner uses Hybrid A\*, a variant of A\* that operates over a continuous state space rather than a discrete grid. States are represented as continuous $(x, y, theta)$ (and $phi$ for the trailer), but duplicate detection uses a discretised grid: XY position is rounded to the nearest spacing multiple, and headings are floor-divided into fixed angular bins of `angular_spacing` size. A node is only expanded once per discrete cell, preventing the search from cycling while still allowing the robot to reach any continuous pose within a cell.

Neighboring cells are generated from a node by sampling a subset of the possible control inputs, and then simulating the kinematics resulting from them.

The heuristic used is the Reeds-Shepp path length from the current state to the goal, ignoring obstacles. This is admissible and generally tighter than Euclidean distance for non-holonomic robots, as it accounts for turning radius constraints that Euclidean distance ignores @kurzer_path_2016[Section 8.1] @dolgov[Page 2].

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


#figure(astar_block, caption: [Hybrid A\* pseudocode])

=== Primitive Generation
#figure(
    image("media/photos/primitives_fig.svg", width: 100%),
    caption: [Motion primitives generated from a single car state (heading 30 $degree$). Blue arcs are forward; orange are reverse. *Left:* all 10 primitives — five steering angles ( -45 $degree$, -22.5 $degree$, 0 $degree$, 22.5 $degree$, 45 $degree$ ) $times$ forward/reverse. *Right:* straight primitives ($delta$ = 0) only, shown at the same scale to illustrate their short length relative to the car body — each arc extends only until its endpoint lands in a new grid cell.]
)
At each node, the bot generates a set of motion primitives by sampling control inputs. For car-like and trailer vehicles, this means sampling steering angles $delta$ evenly between $-delta_max$ and $+delta_max$, combined with forward and reverse speed. For differential drives, different angular velocities $omega$ are instead sampled. Each pair is integrated over $n$ timesteps using an analytic arc trajectory formula #footnote[For derivation, see #ref(<arc_derivation>, supplement: [])] to avoid computational cost and error accumulation:

$
theta (i) &= theta_0 + omega dot i dot d t \
x(i) &= x_0 + R dot (sin(theta (i)) - sin(theta_0)) \
y(i) &= y_0 - R dot (cos(theta (i)) - cos(theta_0))
$ <eq:arc_trajectory_formula>

where $R = nu / omega$, and $i$ is the $i$-th timestep of $d t$ during the turn. Although the Ackermann and differential drive models differ in how $omega$ is produced, both can be reduced to the same unicycle model @lav2006[Section 13.1.2.3]. Therefore, both kinds of vehicles may be modeled using the above formula.

The step count $n$  is calculated to satisfy two constraints:

$ n_nu >= frac("spacing", |nu| dot  d t), quad n_omega >= frac("angular_spacing", |omega| dot d t) $

The final value of $n$ is simply $max(n_nu, space n_omega)$. Satisfying these constraints guarantees every primitive lands in different $(x,y)$ cell and heading bin compared to the initial cell.

The cost of each primitive (and therefore edge in the A\* graph) is the arc length of the primitive plus a weighted heading change, with an additive penalty for reversals to discourage the planner from exploring backwards.


Two vehicle-specific exceptions apply:
- The differential drive appends additional rotate-in-place primitives with $v = 0$. These primitives have zero arc length, their cost relying purely on heading change, allowing the planner to reason about in-place rotation as a distinct maneuver.

- The trailer's primitive generation is more involved. The truck follows the same analytic arc as any other Ackermann vehicle, but the trailer heading $phi$ is coupled to the truck via a nonlinear ODE:

  $ dot(phi) = frac(v, M) sin(theta - phi) $


  where $M$ is the hitch-to-trailer-axle distance @lav2006[Equation 13.19]. This cannot be integrated in closed form alongside the truck arc, so $phi$ is propagated sequentially: the truck positions and headings are computed analytically in one pass, then $phi$ is stepped forward through the resulting sequence using Euler's method.

  Paths that result in the trailer jackknifing #footnote()[When a vehicle towing a trailer loses control, causing the trailer to swing out and form an acute angle with the towing vehicle, resembling a folding pocket knife.] are treated as invalid: each step is checked against a jackknife limit via the condition $|theta - phi| < 90 degree$. If the limit is exceeded, the primitive is discarded.

=== Last-Shot Connection

When a node falls within a euclidean distance of `terminal_radius` of the goal pose, a closed-form trajectory ignorant of obstacles is attempted directly to the goal rather than continuing to expand primitives. For car and trailer bots this is a Reeds-Shepp path @reedsshepp; for the differential drive bot it is a rotate-drive-rotate sequence.

This avoids the difficulty of landing exactly on the goal pose through discrete primitives, and is the mechanism by which the planner achieves precise heading alignment at the goal.

The connection is not attempted on every node within the terminal radius. Since the trajectory must be validated for collisions, attempting it from every qualifying node is expensive and unlikely to pay off far from the goal where obstacles are more likely to block a direct path. Following @kurzer_path_2016[Section 6.1.2], a distance-proportional probability gate is applied: the attempt probability scales linearly from 0 at the edge of the terminal radius to 1 at the goal position itself. The exact calculation  is:

$ P = 1 - norm(bold(p) - bold(p)_g) / R_t $

where $bold(p) = (x, y)$ is the position component of the current state, $bold(p)_g$ is the goal position, and $R_t$ is the terminal radius.

The attempted path is validated for collisions before being accepted. For the trailer, the trailer heading is simultaneously integrated along the RS path and the attempt is rejected if jackknifing occurs. As the planner is unable to directly control the trailer heading, it rejects attempts if the trailer heading at the goal is outside a set tolerance.


=== Post-Processing

The raw path from the search is passed through two steps before playback:

- *Path smoothing* @geraerts_creating_2007[Section 3.2] (`smooth_path`): two random indices are chosen, a direct connection is attempted via `generate_trajectory`, and if collision-free it replaces the span between those two indices. After repeating for 100 iterations, this step reduces total path length by replacing indirect routes imposed by the search order with more direct connections.

#figure(
    grid(
        columns: 2,
        gutter: 1em,
        image("media/photos/path_smoothing_fig_unsmoothed.png"),
        image("media/photos/path_smoothing_fig_smoothed.png"),
    ),
    caption: [Car path before (*left*) and after (*right*) probabilistic shortcutting. The raw path takes a wide detour imposed by the order in which the search explored the space; smoothing finds direct Reeds-Shepp shortcuts that reduce total path length.]
)

- *Resampling* (`resample_path`): the variable-density path is converted to uniform arc-length samples so the animation plays back at constant velocity. Pure-rotation segments (zero XY displacement) are detected separately and resampled at a distinct angular rate.

= Results <results>

#figure(
    grid(
        columns: 2,
        gutter: 1em,
        image("media/photos/bot_nav_photos/point_nav.png"),
        image("media/photos/bot_nav_photos/diff_nav.png"),
        image("media/photos/bot_nav_photos/car_nav.png"),
        image("media/photos/bot_nav_photos/trailer_nav.png"),
    ),
    caption: [Navigation results for each vehicle type: point robot (*top left*), differential drive (*top right*), Ackermann car (*bottom left*), trailer (*bottom right*).]
)

The planner works well across all four vehicle types. The trailer in particular produces some satisfying paths — watching it navigate tight spaces while keeping the trailer heading under control is a good demonstration that the coupled kinematics are being handled correctly. Performance ended up in a reasonable place after profiling, making iteration much faster than it was early on. The codebase also ended up in a good state structurally; having a clean interface between vehicle types made it easy to experiment with planner parameters and add features without things breaking unexpectedly.

== Areas for Improvement

- *Obstacle-aware heuristic.* The Reeds-Shepp heuristic ignores obstacles, which can cause the planner to underestimate costs in cluttered environments. Augmenting it with a precomputed 2D Dijkstra cost-to-go map from the goal would give tighter estimates and likely reduce node expansions. Other implementations @dolgov @kurzer_path_2016 have tried this approach, either individually or combined with obstacle-free distance metrics.

- *State representation boundary.* Throughout the planner, fully typed state objects are used rather than raw float arrays or tuples. This made the logic clean and easy to reason about, but there is a performance cost. NumPy operations on raw arrays would be substantially faster than constructing and passing around dataclass instances in the planner's inner loop. This was a conscious tradeoff in favour of correctness and clarity over speed.

- *Dataclass performance.* Switching from standard Python dataclasses to a library like `msgspec` could reduce the overhead of creating large numbers of short-lived state objects during search. Learning how to have these interact with NumPy would be interesting.

- *Full SAT-based collision.* Shapely is still used for the narrow-phase intersection checks. Replacing it entirely with raw NumPy checks using the Separating Axis Theorem would remove the last external geometry dependency and likely be faster. The current version is good enough, but it's a loose end. Other approaches to collision detection like circular bounding boxes could be explored as well.

- *Planner visualisation.* It would be nice to watch Hybrid A\* progress in real time: drawing expanded nodes, edges, the open set, and the current best path as the search runs. The current sim only shows the final result. This wasn't a priority, but it would make debugging and tuning much more intuitive.

- *Full dynamics and controls simulation.* The current simulator only considers kinematics, with perfect awareness of its surroundings. A more realistic setup could add a simple tracking controller and model dynamics from actuators, or play with sensors and requiring the robot to explore before finding its way.

#pagebreak()


#bibliography("refs.bib", style: "ieee")

#appendix()[
    = Derivation of @eq:arc_trajectory_formula <arc_derivation>

    Both the differential drive and Ackermann kinematics can be simplified to the unicycle kinematic model @lav2006[Section 13.1.2.3]:

    $ dot(x) &= nu dot cos(theta) \  dot(y) &= nu dot sin(theta)  \ dot(theta)  &= omega $ <unicycle>

    The differential drive model is directly equivalent, while Ackermann steering has a dependency between the velocity $nu$ and steering angle $delta$:

    $ omega = nu dot tan(delta) / L $ <eq:ackermann_omega>

    where $L$ is the wheelbase (distance between front and rear axles).

   When $nu$ and $omega$ are held constant, the heading evolves linearly as $ theta(t)= theta_0 + omega$, reducing the position integrals to standard trigonometric forms with known closed-form solutions.

    Treating this as an initial value problem with state $(x_0, y_0, theta_0)$ at $t = 0$, the heading integrates directly, as seen above:

    $ theta(t)= theta_0 + omega dot t  $

    Substituting into the position equations and integrating:

    $

    x(t) &= x_0 + integral_0^t nu dot cos(theta_0 + omega dot tau) space d tau = x_0 + lr(frac(v, omega) dot sin(theta_0 + omega dot tau) |)_(tau=0)^(tau=t) \

    y(t) &= y_0 + integral_0^t nu dot sin(theta_0 + omega dot tau) space d tau = y_0 - lr(frac(v, omega) dot cos(theta_0 + omega dot tau) |)_(tau=0)^(tau=t)

    $

    The radius of the circle traced by this arc can be found through the equation $R = L / tan(delta)$ @lav2006[Section 13.1.2.1]. from
  @eq:ackermann_omega, $omega = nu dot tan(delta)/L$, so $R = L/tan(delta) = nu/omega$. Substituting $nu/omega$ for $R$:

    $
    vec(x(t) & = x_0 + R dot (sin(theta(t)) - sin(theta_0)) , y(t) & = y_0 - R dot (cos(theta(t)) - cos(theta_0)), delim: "[", gap: #1em)
    $

    This can be extended to include the case of $omega approx 0 => R = v / 0$, where the arc approaches a straight line as $R -> infinity$. As in this case the heading, and thus the components of the velocity remain constant, the kinematics of this case can be easily formulated from @unicycle:
    $
      vec(
        x(t)        & = x_0 + dot(x) dot t = x_0 + nu dot t dot cos(theta_0), 
        y(t)        & = y_0 + dot(y) dot t = y_0 + nu dot t dot sin(theta_0), 
        theta(t)    & = theta_0 + omega dot t = theta_0, 
      
      delim: "[", gap: #1em)
    $

    This result can be verified formally via L'Hôpital's rule applied to the arc equations as $omega -> 0$:
    $
      lim_(omega->0) (nu dot (sin(theta_0 + omega dot t) - sin(theta_0))) / omega = 0 / 0 = lim_(omega->0) (nu dot cos(theta_0  + 0 dot t) dot t - 0) / 1 = v dot t dot cos(theta_0)
    $

    $
        lim_(omega->0) (nu dot (cos(theta_0 + omega dot t) - cos(theta_0))) / omega = 0 / 0 = lim_(omega->0) (nu dot -sin(theta_0 + 0 dot t) dot t - 0) / 1 = -v dot t dot sin(theta_0)
    $

    which can be substituted back into the arc equations to get the straight line equations. Stationary ($nu = 0, abs(omega) > 0$) turns do not suffer from this instability, however this would require an ackermann car to have a steering angle of $90 degree$, which would cause $tan(delta) -> infinity$ (@eq:ackermann_omega).
]
