import heapq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mayavi import mlab
from tvtk.api import tvtk

def heuristic(point, goal):
    return np.linalg.norm(point - goal)
    # return np.sum(np.abs(goal - point))

def is_collision(point, inflated_obstacles):
    for obstacle in inflated_obstacles:
        if (obstacle[0] <= point[0] <= obstacle[3]) and (obstacle[1] <= point[1] <= obstacle[4]) and (obstacle[2] <= point[2] <= obstacle[5]):
            return True
    return False

def line_intersects_obstacle(p1, p2, obstacle):
    t = 0.0
    step = 0.01
    while t <= 1.0:
        point = p1 + t * (p2 - p1)
        if (obstacle[0] <= point[0] <= obstacle[3]) and (obstacle[1] <= point[1] <= obstacle[4]) and (obstacle[2] <= point[2] <= obstacle[5]):
            return True
        t += step
    return False

def get_neighbors(point, inflated_obstacles, bounds, step_size=0.5):
    neighbors = []
    for dx in [-step_size, 0, step_size]:
        for dy in [-step_size, 0, step_size]:
            for dz in [-step_size, 0, step_size]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                neighbor = point + np.array([dx, dy, dz])
                if (bounds[0] <= neighbor[0] <= bounds[3]) and (bounds[1] <= neighbor[1] <= bounds[4]) and (bounds[2] <= neighbor[2] <= bounds[5]) and not is_collision(neighbor, inflated_obstacles):
                    collision_free = True
                    for obstacle in inflated_obstacles:
                        if line_intersects_obstacle(point, neighbor, obstacle):
                            collision_free = False
                            break
                    if collision_free:
                        neighbors.append(neighbor)
    return neighbors

def a_star(start, goal, obstacles, bounds, drone_size):
    inflated_obstacles = obstacles + np.array([-drone_size, -drone_size, -drone_size, drone_size, drone_size, drone_size])
    start = np.round(start).astype(int)
    goal = np.round(goal).astype(int)
    start_key = tuple(start)
    goal_key = tuple(goal)
    came_from = {}
    g_score = {start_key: 0}
    f_score = {start_key: heuristic(start, goal)}
    open_set = [(f_score[start_key], start_key)]

    while open_set:
        _, current_key = heapq.heappop(open_set)
        current = np.array(current_key)
        if np.array_equal(current, goal):
            path = [current]
            while current_key in came_from:
                current = came_from[current_key]
                current_key = tuple(current)
                path.append(current)
            return path[::-1]

        for neighbor in get_neighbors(current, inflated_obstacles, bounds):
            neighbor_key = tuple(neighbor)
            tentative_g_score = g_score[current_key] + np.linalg.norm(neighbor - current)
            if neighbor_key not in g_score or tentative_g_score < g_score[neighbor_key]:
                came_from[neighbor_key] = current
                g_score[neighbor_key] = tentative_g_score
                f_score[neighbor_key] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor_key], neighbor_key))

    return []

def plot_path(obstacles, start, goal, path):
    fig = mlab.figure(bgcolor=(1, 1, 1))

    for obstacle in obstacles:
        x = [obstacle[0], obstacle[3]]
        y = [obstacle[1], obstacle[4]]
        z = [obstacle[2], obstacle[5]]
        cube = tvtk.CubeSource()
        cube.x_length = x[1] - x[0]
        cube.y_length = y[1] - y[0]
        cube.z_length = z[1] - z[0]
        cube.center = ((x[1] + x[0]) / 2, (y[1] + y[0]) / 2, (z[1] + z[0]) / 2)
        cube_mapper = tvtk.PolyDataMapper(input_connection=cube.output_port)
        cube_actor = tvtk.Actor(mapper=cube_mapper)
        cube_actor.property.color = (1, 0, 0)
        cube_actor.property.opacity = 0.3
        fig.scene.add_actor(cube_actor)

    if path:
        for i in range(len(path) - 1):
            mlab.plot3d(
                [path[i][0], path[i + 1][0]],
                [path[i][1], path[i + 1][1]],
                [path[i][2], path[i + 1][2]],
                color=(0, 1, 0),
                tube_radius=0.025,
            )

    mlab.points3d(start[0], start[1], start[2], color=(0, 0, 1), scale_factor=0.1)
    mlab.points3d(goal[0], goal[1], goal[2], color=(1, 1, 0), scale_factor=0.1)

    mlab.show()


if __name__ == '__main__':
    obstacles = np.loadtxt("obstacles.txt")

    start = np.array([1, -2, 0])
    goal = np.array([-1, 2, 0])
    bounds = np.array([min(obstacles[:, 0])-1, min(obstacles[:, 1])-1, min(obstacles[:, 2])-1, max(obstacles[:, 3])+1, max(obstacles[:, 4])+1, max(obstacles[:, 5])])
    drone_size = 0.01

    path = a_star(start, goal, obstacles, bounds, drone_size)

    if path:
        print("Waypoints:")
        for waypoint in path:
            print(waypoint)
    else:
        print("No path found.")

    plot_path(obstacles, start, goal, path)
