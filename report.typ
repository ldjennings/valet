
= Flatland Homework Assignment
== RBE 550 - Liam Jennings


=== Introduction
For this assignment, I made a simple grid world for the 'hero' and 'enemy' robots to navigate and maneuver around each other. I chose to use python, pygame, and A-star to finish this project. 


=== Approach

I chose to represent the state of the world using a numpy array, with different states (empty, wall, goal, hero, enemy) represented by different integers. I chose numpy arrays to store the grid state because it allows for fast access and easy copying of the grid for updates, while also being very simple. 

The hero uses A\* with a Manhattan distance heuristic to decide its next move, with that being recalculated with every step taken. Using D\* would be more appropriate due to the changing environment, but I used A\* because it was simpler to implement.

Enemies just move step by step, ignorant of anything else but the hero. I decided to run the program using pygame, but I think that I really should have just done matplotlib, because it would make it easier to visualize other aspects of the problem.



=== Results

#figure(
  image("pygame_animation_3.gif", width: 50%)
)

When playing with the program, I mainly varied the number of enemies in the simulation, also changing whether or not the hero can teleport after being caught. For low numbers of enemies (\~10), the hero was almost never caught, even without being able to teleport. At higher numbers, the hero can bounce around.

At times, when an enemy and the hero meet head on, they will oscillate up and down continuously. This is probably a quirk with how I programmed it.


=== Conclusions

Overall, A\* works well for this kind of grid navigation. At higher numbers, the enemies add enough challenge to make it interesting, and the GIFs are helpful for seeing how the simulation plays out.