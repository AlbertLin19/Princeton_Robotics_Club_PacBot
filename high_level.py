import numpy as np
from simulator.game_engine.grid import grid
from simulator.game_engine.variables import *
from algorithms.astar import astar

# o = normal pellet, e = empty space, O = power pellet, c = cherry position
# I = wall, n = ghost chambers
WALLS = np.logical_or(np.array(grid) == I, np.array(grid) == n, dtype=bool)

ACTIONS = [(-1, 0), (0, -1), (0, 0), (0, 1), (1, 0)]

NT = 3

# helper method to astar to a ghost, which is technically a barrier in maze
def astar_ghost(maze, start, end):
    maze[end] = False
    path = astar(maze, start, end)
    maze[end] = True
    return path

# state is a dict with keys:
#    pellets:       height x width
#    power_pellets: height x width
#    pac:           (row, col)
#    r:             (row, col)
#    b:             (row, col)
#    o:             (row, col)
#    p:             (row, col)
#    rf:            bool
#    bf:            bool
#    of:            bool
#    pf:            bool
#    dt:            distance threshold (in cells)
def get_action(state):

    obstacles = WALLS.copy()
    g_positions = []
    f_positions = []
    # consider ghosts which are not frightened to be obstacles
    if state["rf"]:
        f_positions.append(state["r"])
    else:
        g_positions.append(state["r"])
        obstacles[state["r"]] = True
    if state["bf"]:
        f_positions.append(state["b"])
    else:
        g_positions.append(state["b"])
        obstacles[state["b"]] = True
    if state["of"]:
        f_positions.append(state["o"])
    else:
        g_positions.append(state["o"])
        obstacles[state["o"]] = True
    if state["pf"]:
        f_positions.append(state["p"])
    else:
        g_positions.append(state["p"])
        obstacles[state["p"]] = True

    print("phase: frightened ghosts")

    # target the closest frightened ghost not on pac
    # move to it if it exists and is within dt
    closest_d = None
    closest_path = None
    for f_position in f_positions:
        print("pathfinding to frightened ghost")
        path = astar_ghost(obstacles, state["pac"], f_position)
        if not path or len(path) < 2:
            continue
        if closest_d is None or closest_d > len(path) - 1:
            closest_d = len(path) - 1
            closest_path = path
            if closest_d <= 1:
                break
    if closest_d and closest_d <= state["dt"]:
        return ACTIONS.index(tuple(np.subtract(closest_path[1], closest_path[0])))

    print("phase: power pellets")

    # target the closest power pellet not on pac
    # move to it, if it exists and (is further than 1 cell away or a ghost is within NT)
    # wait at it, if it exists and is within 1 cell and a ghost is not within NT cells
    nearby = False
    for g_position in g_positions:
        if WALLS[g_position]:
            continue
        print("pathfinding to ghost")
        path = astar_ghost(obstacles, state["pac"], g_position)
        if not path or len(path) - 1 <= NT:
            nearby = True
    print("nearby:", nearby)
    positions = np.argwhere(state["power_pellets"])
    closest_d = None 
    closest_path = None
    for position in positions:
        print("pathfinding to power pellet")
        path = astar(obstacles, state["pac"], position)
        if not path or len(path) < 2:
            continue 
        if closest_d is None or closest_d > len(path) - 1:
            closest_d = len(path) - 1
            closest_path = path 
            if closest_d <= 1:
                break
    if closest_d:
        if closest_d > 1 or nearby:
            return ACTIONS.index(tuple(np.subtract(closest_path[1], closest_path[0])))
        else:
            return ACTIONS.index((0, 0))
        
    print("phase: pellets")
    # target the closest pellet not on pac
    # move to it if it exists
    positions = np.argwhere(state["pellets"])
    closest_d = None 
    closest_path = None
    for position in positions:
        path = astar(obstacles, state["pac"], position)
        if not path or len(path) < 2:
            continue 
        if closest_d is None or closest_d > len(path) - 1:
            closest_d = len(path) - 1
            closest_path = path 
            if closest_d <= 1:
                break
    if closest_d:
        return ACTIONS.index(tuple(np.subtract(closest_path[1], closest_path[0])))

    return ACTIONS.index((0, 0))