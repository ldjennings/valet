#show link: underline
#import "template.typ"

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
For this assignment, I simulated the kinematics of the robot, car, and trailer using a state lattice, and implemented a collision checker using #link("https://shapely.readthedocs.io/en/stable/")[shapely], a python library used to model and manipulate geometry.


==== Usage
Dependencies are managed through a `pyproject.toml` file. Create a virtual environment, then run `pip install .`.

To run the main simulator, now enter the command `runsim`, as an entry point was created by the config file. This is basic, as it only has the collision checking implemented, but you can maneuver and rotate it with the arrow and WASD keys respectively.

To specify the configuration to collision check, use `--work` to specify   `diff`, `car`, or `trailer`.


=== Approach

I chose to represent the state of the world using a numpy array, with different states (empty, wall, goal, hero, enemy) represented by different integers. I chose numpy arrays to store the grid state because it allows for fast access and easy copying of the grid for updates, while also being very simple.

The hero uses A\* with a Manhattan distance heuristic to decide its next move, with that being recalculated with every step taken. Using D\* would be more appropriate due to the changing environment, but I used A\* because it was simpler to implement.



=== Collision Detection
// Demonstrate your collision checker implementation with annotated diagrams depicting overlap
// checks with a vehicle and obstacle region. Submit these diagrams individually, but also include
// them in your report.

For collision detection, I initially chose to use #link("https://shapely.readthedocs.io/en/stable/")[shapely], a third-party python library used to model and manipulate geometry. The main reason for doing so was to develop the rest of the program without needing to take the time to figure out all the details of collision detection.

However, this approach soon ran into performance issues. Shapely is designed to be performant, but much of that performance lies in batching calls to the underlying C/C++ library. My approach, calling intersection on individual geometries, suffered from the overhead involved in repeatedly crossing that barrier.


// Additionally, shapely's features

//  overhead due to being a fully featured geometry library. It's designed to deal with general topology geometry analysis, while this assignment is limited to collisions between rectangles, either axis aligned or oriented.

// Shapely is designed around being a python interface for the #link("https://libgeos.org/")[C/C++ GEOS library]

To deal with this, I focused on three different approaches: caching approximate rotations of each geometry, reducing the number of states that need to be checked, and eliminating states that can be easily validated.

==== State validator
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

The state validator function accepts a list o representing the current state

The general approach for the state validator function focused around filtering the rectangle checks with cheap checks before calling expensive shapely functions.

The state validator, basically

I could likely replace this with an implementation of the Separating Axis Theorem, but I didn't believe it was worth the time to do so. According to #link("https://docs.python.org/3/library/profile.html")[Cprofile], calls to the STRtree function only take up 5% of the hybrid A\* runtime #footnote[Ran with a seed of 11, grid spacing of 1, angular spacing of $frac(pi, 3)$], so I decided to focus on improving other parts of the program.









As an example, the code for defining the trailer geometry can be seen below. It constructs two rectangles using the constants described in the assignment and the current state variables:

```python
def truck_trailer_geom(x, y, theta, phi):
    """
    Returns a Shapely geometry (truck + trailer + connection)
    given truck rear-axle center (x, y), truck heading theta, and trailer angle phi.
    The trailer is attached directly at the truck's rear axle.
    """
    # shortening the constants,
    TRUCK_LEN = cfg.TRUCK_LENGTH_METERS
    TRUCK_W = cfg.TRUCK_WIDTH_METERS
    WHEELBASE = cfg.TRUCK_WHEELBASE_METERS
    TRAILER_LEN = cfg.TRAILER_LENGTH_METERS
    TRAILER_W = cfg.TRAILER_WIDTH_METERS
    D1 = cfg.TRUCK_HITCH_TO_TRAILER_AXLE * 3

    # Trailer axle
    x_t = x + D1 * math.cos(math.radians(theta + phi))
    y_t = y + D1 * math.sin(math.radians(theta + phi))

    # truck rectangle with the rear axle at 0,0
    truck_rect = Polygon([
        [-WHEELBASE, -TRUCK_W/2],
        [TRUCK_LEN - WHEELBASE, -TRUCK_W/2],
        [TRUCK_LEN - WHEELBASE,  TRUCK_W/2],
        [-WHEELBASE,  TRUCK_W/2]
    ])
    truck_world = rotate(truck_rect, theta, origin=(0, 0))
    truck_world = translate(truck_world, xoff=x, yoff=y)

    # trailer rectangle
    trailer_rect = Polygon([
        [-TRAILER_LEN/2, -TRAILER_W/2],
        [ TRAILER_LEN/2, -TRAILER_W/2],
        [ TRAILER_LEN/2,  TRAILER_W/2],
        [-TRAILER_LEN/2,  TRAILER_W/2]
    ])
    trailer_world = rotate(trailer_rect, theta + phi, origin=(0, 0))
    trailer_world = translate(trailer_world, xoff=x_t, yoff=y_t)

    # line between them
    connection = LineString([(x, y), (x_t, y_t)])

    return unary_union([truck_world, trailer_world, connection])
```



#figure(
    grid(
        columns: 2,
        gutter: 2mm,
        box(
            stroke: 2pt + black,
            radius: 5pt,
            inset: 5pt,
            image("media/trailer_clear.png"),
        ),
        box(
            stroke: 2pt + black,
            radius: 5pt,
            inset: 5pt,
            image("media/trailer_collision.png"),
        )
    ),
    caption: "Example of collision detection on trailer. The connection length has been exaggerated."
)

An example of the collision detection can be seen above with the truck and trailer. When it has collided with the obstacles in the environment, the vehicle turns red, but is normally green when not colliding with anything.. As can be seen in the diagram, the line segment is also included in the collision check.

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

=== Results

==== Challenges

I had a lot of difficulty with getting a state lattice set up. This mainly revolved around properly simultating the kinematics of the system and making sure that the state lattice was properly aligned throughout the process. I was able to get a very rough version working, but it failed to regularly tile the state space, as I was just propogating out from headings rather than actually going to regular nodes that could easily be searched using A\*. It could get within a very rough radius, but not close enough to be consistent:

#figure(
    image("media/test_lattice.png"),
    caption: "results from experimenting with rudimentary state latticing."
)

I also tried to get a handle on Reed-Shepp or Dubin paths, but never got far enough with those.

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
