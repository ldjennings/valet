import numpy as np
import matplotlib.pyplot as plt
import heapq

from test2 import generate_primitives

L = 2.5  # wheelbase (m)
v = 2.0  # constant velocity
dt = 0.1
T = 3.0  # seconds per primitive
steering_angles = np.radians([-26.6, -17.5, 0.0, 17.5, 26.6])

def wrap_to_pi(ang: float):
    """Wraps radian angle to pi to negative pi"""

    return (ang + np.pi) % (2 * np.pi) - np.pi

def simulate_primitive(delta, vel):
    x, y, theta = 0, 0, 0
    path = [(x, y, theta)]
    for _ in np.arange(0, T, dt):
        x += vel * np.cos(theta) * dt
        y += vel * np.sin(theta) * dt
        theta += vel / L * np.tan(delta) * dt
        path.append((x, y, theta))
    return path

def generate_neighbors(node, primitives, grid, goal):
    rows, cols = grid.shape
    x0, y0, theta0 = node
    neighbors = []
    
    for primitive in primitives:
        transformed = []
        valid = True

        for p in primitive:
            x_rel, y_rel, theta_rel = p
            x = x0 + np.cos(theta0)*x_rel - np.sin(theta0)*y_rel
            y = y0 + np.sin(theta0)*x_rel + np.cos(theta0)*y_rel
            theta = theta0 + theta_rel

            # basic collision check
            ix, iy = int(round(x)), int(round(y))
            if not (0 <= ix < rows and 0 <= iy < cols) or grid[ix, iy] != 0:
                valid = False
                break
            transformed.append((x, y, theta))

        if valid:
            neighbors.append(transformed[-1])
    
    return neighbors



motion_primitives = []
for delta in steering_angles:
    motion_primitives.append(simulate_primitive(delta, v))     # forward
    motion_primitives.append(simulate_primitive(delta, -v))    # reverse
motion_primitives = np.array(motion_primitives, dtype=object)


# --- A* Path Planning ---
def astar(grid, start, goal):
    def heuristic(s1, s2) -> float:
        """Distance metric that combines xy distance and heading difference"""
        dx, dy = s2[0]-s1[0], s2[1]-s1[1]
        dtheta = np.arctan2(np.sin(s2[2]-s1[2]), np.cos(s2[2]-s1[2]))
        return np.sqrt(dx**2 + dy**2) + 0.2 * abs(dtheta)
    
    
    pq = [(0 + heuristic(start, goal), 0, start, None)]
    visited = {}
    
    while pq:
        f, g, node, parent = heapq.heappop(pq)
        if node in visited: continue
        visited[node] = parent
        if node == goal or heuristic(node, goal) < 3:
            path = []
            while node:
                path.append(node)
                node = visited[node]
            return path[::-1]
        for nbr in generate_neighbors(node, motion_primitives, grid, goal):
            heapq.heappush(pq, (g + 1 + heuristic(nbr, goal), g + 1, nbr, node))
    return None


grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1  # obstacle
start = (2, 2, 0)
goal = (17, 17, np.radians(90))
path = astar(grid, start, goal)
path = np.array(path)

x, y, t = path[-1]


print(f"Final Position: x: {x}, y: {y}, theta: {np.degrees(wrap_to_pi(t))}")


plt.figure(figsize=(6,6))
plt.imshow(grid.T, origin='lower', cmap='gray_r')
plt.plot(path[:,0], path[:,1], 'k--', label='A* path')
plt.legend()
plt.axis('equal')
plt.show()






