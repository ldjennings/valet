import numpy as np
import matplotlib.pyplot as plt
from typing import Sequence
from simulator.main import DifferentialDriveModel
from simulator.obstacle import ObstacleEnvironment

# this is me trying to make the primitives work for differential drives.
# I know that they can follow a grid based solver like raw A* exactly, but I wanted to use it as a proof of concept for the rest
# of the assignment

# class Primitive:
#     def __init__(self, trajectory: Sequence[[float, float, float]]) -> None:
#         self.trajectory = trajectory
#         origin


def wrap_to_pi(ang_rad: float):
    """Wraps arbitrary radian angle to pi to negative pi"""

    return (ang_rad + np.pi) % (2 * np.pi) - np.pi



def generate_trajectory(start, goal, w:float = 1.0, v:float = 1.0, dt=0.1):
        # make inputs nicer
        x_start, y_start, t_start = start
        x_goal, y_goal, t_goal = goal

        t_start = wrap_to_pi(t_start)
        t_goal = wrap_to_pi(t_goal)

        traj = []

        dx = x_goal - x_start
        dy = y_goal - y_start
        theta_to_goal = np.arctan2(dy, dx)
        dtheta1 = wrap_to_pi(theta_to_goal - t_start)

        # first turning stage
        steps_to_rotate = max(1, int(abs(dtheta1) / (w*dt)))

        for i in range(steps_to_rotate):
            theta = t_start + dtheta1 * (i+1)/steps_to_rotate
            traj.append((x_start, y_start, wrap_to_pi(theta)))

        # drive straignt to goal
        distance = np.hypot(dx, dy)
        n_steps_straight = max(1, int(distance / (v*dt)))

        for i in range(n_steps_straight):
            x = x_start + dx * (i+1)/n_steps_straight
            y = y_start + dy * (i+1)/n_steps_straight
            traj.append((x, y, theta_to_goal))

        # rotate to goal heading
        dtheta2 = wrap_to_pi(t_goal - theta_to_goal)
        steps_to_rotate2 = max(1, int(abs(dtheta2) / (w*dt)))

        for i in range(steps_to_rotate2):
            theta = theta_to_goal + dtheta2 * (i+1)/steps_to_rotate2
            traj.append((x_goal, y_goal, wrap_to_pi(theta)))

        return np.array(traj)


primitive_goal_poses = np.array([
    [-2, 1, 180],
    [-2, 1, 135],
    [-2, 1,  90],
    [-1, 1, 180],
    [-1, 1, 135],
    [-1, 1,  90],

    [0, 1,   90],

    [1, 1,  90],
    [1, 1,  45],
    [1, 1,   0],
    [2, 1,  90],
    [2, 1,  45],
    [2, 1,   0],
], dtype=float)

# primitive_goal_poses[:, 2] = np.radians(primitive_goal_poses[:, 2])


# start = [0,0,np.radians(0)]



# plt.figure(figsize=(6,6))

# for p in primitive_goal_poses:
#     # need to rotate into the frame of the robot
#     dtheta = wrap_to_pi(start[2]) - np.radians(90)

#     traj = generate_trajectory([0,0,np.radians(90)], p)
#     traj[:,2] = wrap_to_pi(traj[:,2] + dtheta)
#     #rotate points in the 0 and 1 columns to new positions with heading
#     traj_reverse = -traj

#     R = np.array([
#         [np.cos(dtheta), -np.sin(dtheta)],
#         [np.sin(dtheta),  np.cos(dtheta)]
#     ])

#     xy_traj  = (R @ traj[:, :2].T).T + np.array([start[0], start[1]])
#     xy_reverse = (R @ traj_reverse[:, :2].T).T + np.array([start[0], start[1]])


#     traj_rot = np.column_stack((xy_traj, traj[:,2]))
#     reverse_rot = np.column_stack((xy_reverse, -traj[:,2]))


#     # plt.plot(traj_rot[:,0],traj_rot[:,1])
#     # plt.plot(xy_reverse[:,0],xy_reverse[:,1])
#     # plt.plot(traj[-1,2])


# plt.axis('equal')
# # plt.show()
     


def generate_primitives(x, y, theta, environment:ObstacleEnvironment) -> list:
    primitives = []
    for p in primitive_goal_poses:
        # need to rotate into the frame of the robot
        dtheta = wrap_to_pi(theta) - np.radians(90)

        traj = generate_trajectory([0,0,np.radians(90)], p)
        traj[:,2] = wrap_to_pi(traj[:,2] + dtheta)
        #rotate points in the 0 and 1 columns to new positions with heading
        traj_reverse = -traj

        R = np.array([
            [np.cos(dtheta), -np.sin(dtheta)],
            [np.sin(dtheta),  np.cos(dtheta)]
        ])

        xy_traj  = (R @ traj[:, :2].T).T + np.array([x, y])
        xy_reverse = (R @ traj_reverse[:, :2].T).T + np.array([x, y])

        forward_primitive = np.column_stack((xy_traj, traj[:,2]))
        reverse_primitive = np.column_stack((xy_reverse, -traj[:,2]))
        
        # collision check
        valid = True
        for state in forward_primitive:
            if DifferentialDriveModel.check_collision(state, environment.world_geom) == False:
                valid = False
                break

            if valid:
                primitives.append((forward_primitive[-1], forward_primitive))

        valid = True
        for state in reverse_primitive:
            if DifferentialDriveModel.check_collision(state, environment.world_geom) == False:
                valid = False
                break

            if valid:
                primitives.append((reverse_primitive[-1], reverse_primitive))

    return primitives


prisms = generate_primitives(3, 3, 0, None)

for end_pose, path in prisms:
    print(end_pose)
    plt.plot(path[:,0], path[:,1])

plt.show()
             


