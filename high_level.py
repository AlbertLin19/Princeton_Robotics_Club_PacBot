import numpy as np
from simulator.game_engine.grid import grid
from simulator.game_engine.variables import *
from algorithms.astar import astar

# o = normal pellet, e = empty space, O = power pellet, c = cherry position
# I = wall, n = ghost chambers
WALLS = np.logical_or(np.array(grid) == I, np.array(grid) == n, dtype=bool)

ACTIONS = [(-1, 0), (0, -1), (0, 0), (0, 1), (1, 0)]

NT = 6

# helper method to astar to a ghost, which is technically a barrier in maze
def astar_ghost(maze, start, end):
    maze[tuple(end)] = False
    tup = astar(maze, start, end)
    maze[tuple(end)] = True
    return tup

# state is a dict with keys:
#    prev_pac:      (row, col)
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
    # first coordinate 0, 1, -1 depending on forward, left, right
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

    obstacles_without_back = obstacles.copy()

    # pass the first five things 
    # prevents pacbot from going backwards
    prev_pac = (0,0)
    if state["dir"] == (-1, 0):
      prev_pac = (state["pac"][0]+1, state["pac"][1])
    elif state["dir"] == (0, 1):
      prev_pac = (state["pac"][0], state["pac"][1]-1)
    elif state["dir"] == (1, 0):
      prev_pac = (state["pac"][0]-1, state["pac"][1])
    elif state["dir"] == (0, -1):
      prev_pac = (state["pac"][0], state["pac"][1]+1)
    obstacles[prev_pac] = True
    #obstacles[state["prev_pac"]] = True
    
    nearby = False
    nearby_actions = None
    for g_position in g_positions:
        if WALLS[tuple(g_position)]:
            continue
        print("pathfinding to ghost")
        tup = astar_ghost(obstacles_without_back, state["pac"], g_position)
        if tup:
            actions, pathlength = tup 
        else:
            continue
        if pathlength - 1 <= NT:
            nearby = True
            # find path from ghost to pac with back barrier
            tup = astar_ghost(obstacles, state["pac"], g_position)
            if tup:
                actions, pathlength = tup 
            nearby_actions = actions
            
    print("nearby:", nearby)

    print("phase: frightened ghosts")

    # target the closest frightened ghost not on pac
    # move to it if it exists and is within dt
    closest_d = None
    closest_actions = None
    for f_position in f_positions:
        print("pathfinding to frightened ghost")
        tup = astar_ghost(obstacles, state["pac"], f_position)
        if tup:
            actions, pathlength = tup 
        else:
            continue
        if pathlength < 2:
            continue
        if closest_d is None or closest_d > pathlength - 1:
            closest_d = pathlength - 1
            closest_actions = actions
            if closest_d <= 1:
                break
    if closest_d and closest_d <= state["dt"]:
        return closest_actions[:5]

    print("phase: power pellets")

    # target the closest power pellet not on pac
    # move to it, if it exists and (is further than 1 cell away or a ghost is within NT)
    # wait at it, if it exists and is within 1 cell and a ghost is not within NT cells
    positions = np.argwhere(state["power_pellets"])
    closest_d = None 
    closest_actions = None
    for position in positions:
        print("pathfinding to power pellet")
        tup = astar(obstacles, state["pac"], position)
        if tup:
            actions, pathlength = tup 
        else:
            continue
        if not actions:
            continue
        if closest_d is None or closest_d > pathlength - 1:
            closest_d = pathlength - 1
            closest_actions = actions 
            if closest_d <= 1:
                break
    if closest_d:
        if closest_d > 1:
            return closest_actions[:5]
        if nearby: # pathfind to nearest ghost
            print("nearby actions")
            print(nearby_actions)
            closest_actions[-1] = (closest_actions[-1][0], nearby_actions[0][1])
            closest_actions.extend(nearby_actions[1:])
            return closest_actions[:5]
        else:
            return [(0, 0)]
    # grid, algorithm, a star
    print("phase: pellets")
    # target the closest pellet not on pac
    # move to it if it exists
    positions = np.argwhere(state["pellets"])
    closest_d = None 
    closest_actions = None
    for position in positions:
        tup = astar_ghost(obstacles, state["pac"], position)
        if tup:
            actions, pathlength = tup 
        else:
            continue
        if not actions or pathlength < 2:
            continue 
        if closest_d is None or closest_d > pathlength - 1:
            closest_d = pathlength - 1
            closest_actions = actions 
            if closest_d <= 1:
                break
    if closest_d:
        return closest_actions[:5]

    return [(0, 0)]

F = 1124314
L = 13898
R = 13424
H = 238872
A = 54354
X = 82348
B = 2342348
actions1 = [[I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I], # 0
        [I,o,o,o,o,I,I,O,F,X,A,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,o,I,I,o,I,I,H,I,I,A,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,R,F,L,L,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,R,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,F,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I], # 5
        [I,o,I,I,R,R,F,L,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,L,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I],
        [I,o,I,I,I,I,I,B,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,B,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I], # 10
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,o,o,o,I,I,B,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,B,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,I,I,I,I,I,B,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,o,o,o,I,I,B,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I], # 15
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I], # 20
        [I,o,I,I,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I], # 25
        [I,o,o,o,o,I,I,O,o,o,o,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I]]
#        |         |         |         |         |         |         |   top right of pacman board
#        0         5        10        15       20         25       30
actions1 = np.array(actions1)
def get_hardcoded_action1(state):
    if actions1[tuple(state["pac"])] == B:
        return [(0, 8), (1, -1)]
    if actions1[tuple(state["pac"])] == F:
        return [(0, 2)]
    if actions1[tuple(state["pac"])] == L:
        return [(0, 1), (1, -1)]
    if actions1[tuple(state["pac"])] == R:
        return [(0, 1), (-1, -1)]
    if actions1[tuple(state["pac"])] == H:
        if (actions1[tuple(state["r"])] == A) or (actions1[tuple(state["o"])] == A) or (actions1[tuple(state["b"])] == A) or (actions1[tuple(state["p"])] == A) or (actions1[tuple(state["r"])] == X) or (actions1[tuple(state["o"])] == X) or (actions1[tuple(state["b"])] == X) or (actions1[tuple(state["p"])] == X):
            return [(0, 1), (-1, -1)]
        return [(0, 0)]
    if actions1[tuple(state["pac"])] == X:
        return [(0, 1), (-1, -1)]
    else:
        return [(0, 100)]
    
actions2 = [[I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I], # 0
        [I,o,o,o,o,I,I,O,F,X,A,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,o,I,I,o,I,I,H,I,I,A,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,F,L,L,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I], # 5
        [I,o,I,I,o,o,o,R,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,R,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I],
        [I,o,I,I,I,I,I,B,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,B,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I], # 10
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,o,o,o,I,I,B,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,B,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,I,I,I,I,I,B,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,o,o,o,I,I,B,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I], # 15
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,o,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I], # 20
        [I,o,I,I,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I], # 25
        [I,o,o,o,o,I,I,O,o,o,o,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I]]
#        |         |         |         |         |         |         |   top right of pacman board
#        0         5        10        15       20         25       30
actions2 = np.array(actions2)
def get_hardcoded_action2(state):
    if actions2[tuple(state["pac"])] == B:
        return [(0, 8), (1, -1)]
    if actions2[tuple(state["pac"])] == F:
        return [(0, 2)]
    if actions2[tuple(state["pac"])] == L:
        return [(0, 1), (1, -1)]
    if actions2[tuple(state["pac"])] == R:
        return [(0, 1), (-1, -1)]
    if actions2[tuple(state["pac"])] == H:
        if (actions2[tuple(state["r"])] == A) or (actions2[tuple(state["o"])] == A) or (actions2[tuple(state["b"])] == A) or (actions2[tuple(state["p"])] == A) or (actions2[tuple(state["r"])] == X) or (actions2[tuple(state["o"])] == X) or (actions2[tuple(state["b"])] == X) or (actions2[tuple(state["p"])] == X):
            return [(0, 1), (-1, -1)]
        return [(0, 0)]
    if actions2[tuple(state["pac"])] == X:
        return [(0, 1), (-1, -1)]
    else:
        return [(0, 100)]

actions3 = [[I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I], # 0
        [I,o,o,o,o,I,I,O,F,X,A,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,o,I,I,o,I,I,H,I,I,A,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,F,L,L,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I], # 5
        [I,o,I,I,o,o,o,R,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,R,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I],
        [I,o,I,I,I,I,I,B,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,B,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I], # 10
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,o,o,o,I,I,B,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I],
        [I,o,I,I,I,I,I,B,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,I,I,I,I,I,B,I,I,I,I,I,e,I,n,n,n,n,e,I,I,I,I,I,o,I,I,I,I,I],
        [I,o,o,o,o,I,I,B,o,o,o,I,I,e,I,n,n,n,I,e,e,e,o,I,I,o,o,o,o,o,I], # 15
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,n,n,n,I,e,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,B,I,I,o,I,I,e,I,I,I,I,I,e,I,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,B,I,I,o,e,e,e,e,e,e,e,e,e,I,I,o,o,o,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,B,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,e,I,o,I],
        [I,o,I,I,I,I,I,L,I,I,o,I,I,I,I,I,e,I,I,I,I,I,I,I,I,o,I,I,I,o,I], # 20
        [I,o,I,I,o,o,o,L,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,I,I,I,I,I,I,I,I,I,I,o,I,I,o,I,I,I,o,I],
        [I,o,I,I,o,I,I,I,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,o,o,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,e,I,o,I],
        [I,o,I,I,o,I,I,o,I,I,o,I,e,e,e,e,e,e,e,e,e,I,o,I,I,o,I,I,I,o,I], # 25
        [I,o,o,o,o,I,I,O,o,o,o,I,e,e,e,e,e,e,e,e,e,I,o,o,o,o,o,O,o,o,I],
        [I,I,I,I,I,I,I,I,I,I,I,I,e,e,e,e,e,e,e,e,e,I,I,I,I,I,I,I,I,I,I]]
#        |         |         |         |         |         |         |   top right of pacman board
#        0         5        10        15       20         25       30
actions3 = np.array(actions3)
def get_hardcoded_action3(state):
    if actions3[tuple(state["pac"])] == B:
        return [(0, 7), (1, -1)]
    if actions3[tuple(state["pac"])] == F:
        return [(0, 2)]
    if actions3[tuple(state["pac"])] == L:
        return [(0, 1), (1, -1)]
    if actions3[tuple(state["pac"])] == R:
        return [(0, 1), (-1, -1)]
    if actions3[tuple(state["pac"])] == H:
        if (actions3[tuple(state["r"])] == A) or (actions3[tuple(state["o"])] == A) or (actions3[tuple(state["b"])] == A) or (actions3[tuple(state["p"])] == A) or (actions3[tuple(state["r"])] == X) or (actions3[tuple(state["o"])] == X) or (actions3[tuple(state["b"])] == X) or (actions3[tuple(state["p"])] == X):
            return [(0, 1), (-1, -1)]
        return [(0, 0)]
    if actions3[tuple(state["pac"])] == X:
        return [(0, 1), (-1, -1)]
    else:
        return [(0, 100)]