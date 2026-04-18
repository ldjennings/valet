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

For collision detection, I chose to use #link("https://shapely.readthedocs.io/en/stable/")[shapely], a third-party python library used to model and manipulate geometry.

This significantly reduced the difficulty of doing collision detection, and reduced the technical debt.

Rather than implementing a series of more granular checks, like checking points along the robot perimiter, then using hte separating axis theorem or similar, I only needed to define the vehicles and environment geometrically and then call the `intersects` method with the two. 

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
            image("photos/trailer_clear.png"),
        ),
        box(
            stroke: 2pt + black,
            radius: 5pt,
            inset: 5pt,
            image("photos/trailer_collision.png"),
        )
    ),
    caption: "Example of collision detection on trailer. The connection length has been exaggerated."
)

An example of the collision detection can be seen above with the truck and trailer. When it has collided with the obstacles in the environment, the vehicle turns red, but is normally green when not colliding with anything.. As can be seen in the diagram, the line segment is also included in the collision check. 

=== Results

==== Challenges

I had a lot of difficulty with getting a state lattice set up. This mainly revolved around properly simultating the kinematics of the system and making sure that the state lattice was properly aligned throughout the process. I was able to get a very rough version working, but it failed to regularly tile the state space, as I was just propogating out from headings rather than actually going to regular nodes that could easily be searched using A\*. It could get within a very rough radius, but not close enough to be consistent:

#figure(
    image("photos/test_lattice.png"),
    caption: "results from experimenting with rudimentary state latticing."
)

I also tried to get a handle on Reed-Shepp or Dubin paths, but never got far enough with those.  