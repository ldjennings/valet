#show link: underline
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

For collision detection, I initially chose to use #link("https://shapely.readthedocs.io/en/stable/")[shapely], a third-party python library used to model and manipulate geometry. The main reason for doing so was to develop the rest of the program without needing to take the time to figure out all the details of collision detection.

However, this approach soon ran into performance issues. While shapely is fairly performant by default, it also has to deal with much more overhead due to being a fully featured geometry library. It encompasses concepts like [TODO], while this assignment mostly deals with collisions between boxes, most of them on an axis aligned regular grid. When the collision detector/state validation function can run tens, even hundreds of thousands of times at times, that overhead quickly adds up.

To deal with this, I focused on three different approaches: caching approximate rotations of each geometry, reducing the number of states that need to be checked, and eliminating states that can be easily validated.

==== State validator
```py
    def is_valid_state(self, bot_geoms: list[FootprintEntry]) -> bool:
        """Return True if none of the geometries intersect any obstacle and all lie within the boundary."""
        for ox, oy, g in bot_geoms:
            minx, miny, maxx, maxy = g.bounds
            bounds = (minx + ox, miny + oy, maxx + ox, maxy + oy)

            if not self._within_bounds(bounds):
                return False

            # Broad-phase rejection
            if not self.has_possible_collision(bounds):
                continue

            # Very rare, also very expensive. Not called in the majority of cases.
            # Only translate if the geometry isn't already placed.
            if ox != 0.0 or oy != 0.0:
                positioned = translate(g, xoff=ox, yoff=oy)
            else:
                positioned = g
            if intersects_any(self.obstacles, positioned):
                return False

        return True
```

At the fundamental level the state validator works with shapely: defining the vehicles and environment geometrically and then call the `intersects` method with the two. I did start using the C API for shapely, and stored the obstacles as an #link("https://shapely.readthedocs.io/en/stable/strtree.html#shapely.STRtree")[STRTree], but its still the same idea. I could likely replace this with an implementation of the Separating Axis Theorem, but I didn't believe it was worth the time to do so. According to #link("https://docs.python.org/3/library/profile.html")[Cprofile], calls to the STRtree function only take up 1% of the hybrid A\* runtime #footnote[Ran with a seed of 11, grid spacing of 1, angular spacing of $frac(pi, 3)$], so I decided to focus on improving other parts of the program.

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
            image("../media/trailer_clear.png"),
        ),
        box(
            stroke: 2pt + black,
            radius: 5pt,
            inset: 5pt,
            image("../media/trailer_collision.png"),
        )
    ),
    caption: "Example of collision detection on trailer. The connection length has been exaggerated."
)

An example of the collision detection can be seen above with the truck and trailer. When it has collided with the obstacles in the environment, the vehicle turns red, but is normally green when not colliding with anything.. As can be seen in the diagram, the line segment is also included in the collision check.

=== Results

==== Challenges

I had a lot of difficulty with getting a state lattice set up. This mainly revolved around properly simultating the kinematics of the system and making sure that the state lattice was properly aligned throughout the process. I was able to get a very rough version working, but it failed to regularly tile the state space, as I was just propogating out from headings rather than actually going to regular nodes that could easily be searched using A\*. It could get within a very rough radius, but not close enough to be consistent:

#figure(
    image("../media/test_lattice.png"),
    caption: "results from experimenting with rudimentary state latticing."
)

I also tried to get a handle on Reed-Shepp or Dubin paths, but never got far enough with those.