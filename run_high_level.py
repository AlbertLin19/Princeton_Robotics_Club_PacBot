import numpy as np
from simulator.gym_wrappers import PacBotEnv
from simulator.visualizer import Visualizer
from high_level import get_action
import time

#           UP,      LEFT,    STAY,   RIGHT,  DOWN
ACTIONS = [(-1, 0), (0, -1), (0, 0), (0, 1), (1, 0)]
#           LEFT,    DOWN,    _,      UP,     RIGHT
LEFT =    [(0, -1), (1, 0), (0, 0), (-1, 0), (0, 1)]


if __name__ == "__main__":
    visualizer = Visualizer()
    env = PacBotEnv(speed=0.8)
    obs = env.reset()
    direction = (-1, 0)
    grid = env.render()
    visualizer.draw_grid(grid)

    done = False
    key = 0
    #key = visualizer.wait()
    while key != "q":
        pellets = np.zeros((env.GRID_HEIGHT, env.GRID_WIDTH))
        power_pellets = np.zeros((env.GRID_HEIGHT, env.GRID_WIDTH))
        pellet_exists = obs[np.array(env.STATE_VALUES) == "pellet"]
        for i in range(len(pellet_exists)):
            if pellet_exists[i]:
                pellets[env.PELLET_LOCATIONS == i+1] = 1

        power_pellet_exists = obs[np.array(env.STATE_VALUES) == "power_pellet"]
        for i in range(len(power_pellet_exists)):
            if power_pellet_exists[i]:
                power_pellets[env.POWER_PELLET_LOCATIONS == i+1] = 1
        
        state = {
            "pellets": pellets,
            "power_pellets": power_pellets,
            "pac": (int(obs[env.STATE_VALUES.index("pac_x")]), int(obs[env.STATE_VALUES.index("pac_y")])),
            "dir": direction,
            "r": (int(obs[env.STATE_VALUES.index("r_x")]), int(obs[env.STATE_VALUES.index("r_y")])),
            "b": (int(obs[env.STATE_VALUES.index("b_x")]), int(obs[env.STATE_VALUES.index("b_y")])),
            "o": (int(obs[env.STATE_VALUES.index("o_x")]), int(obs[env.STATE_VALUES.index("o_y")])),
            "p": (int(obs[env.STATE_VALUES.index("p_x")]), int(obs[env.STATE_VALUES.index("p_y")])),
            "rf": obs[env.STATE_VALUES.index("r_frightened")],
            "bf": obs[env.STATE_VALUES.index("b_frightened")],
            "of": obs[env.STATE_VALUES.index("o_frightened")],
            "pf": obs[env.STATE_VALUES.index("p_frightened")],
            "dt": obs[env.STATE_VALUES.index("frightened_timer")] / 2,
        }
        actions = get_action(state) # in the form of a list of tuples, [(action, x), ..., (action, x)]
        action = actions[0]
        if action[0] == 0: # move forward
            for _ in range(action[0]):
                obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
                grid = env.render()
                visualizer.draw_grid(grid)
        elif action[0] == 1: # turn left
            direction = LEFT[ACTIONS.index(tuple(direction))]
            obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
            grid = env.render()
            visualizer.draw_grid(grid)
        elif action[0] == -1: # turn right
            direction = ACTIONS[LEFT.index(tuple(direction))]
            obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
            grid = env.render()
            visualizer.draw_grid(grid)

            
        # for row in obs[11]:
        #     for cell in row:
        #         print('1' if cell else '0', end='')
        #     print()
        # print(reward, done)

        if done:
            env.reset()
            env.render()
        #key = visualizer.wait()
        time.sleep(0.1)