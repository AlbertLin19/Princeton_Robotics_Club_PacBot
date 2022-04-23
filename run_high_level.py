import numpy as np
from simulator.gym_wrappers import PacBotEnv
from simulator.visualizer import Visualizer
from high_level import get_action
import time

#           UP,      LEFT,    STAY,   RIGHT,  DOWN
ACTIONS = [(-1, 0), (0, -1), (0, 0), (0, 1), (1, 0)]
#           LEFT,    DOWN,    _,      UP,     RIGHT
LEFT =    [(0, -1), (1, 0), (0, 0), (-1, 0), (0, 1)]

SLEEP = .01

START_DIR = (-1, 0)

if __name__ == "__main__":
    visualizer = Visualizer()
    env = PacBotEnv(speed=2)
    obs = env.reset()
    direction = START_DIR
    grid = env.render()
    visualizer.draw_grid(grid)

    done = False
    key = 0
    # key = visualizer.wait()
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
        
        print("direction")
        print(direction)
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
        print("getting actions")
        actions = get_action(state) # in the form of a list of tuples, [(action, x), ..., (action, x)]
        print("got actions")
        print(actions)
        action = actions[0]
        if action[0] ==  0 and action[1] == 1:
            prev_lives = env.game_state.lives
            obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
            if env.game_state.lives != prev_lives:
                direction = START_DIR
            grid = env.render()
            visualizer.draw_grid(grid)
            time.sleep(SLEEP)
            if len(actions) == 1:
                continue
            action = actions[1]
        if action[0] == 0: # move forward
            if action[1] == 0:
                prev_lives = env.game_state.lives
                obs, reward, done, _ = env.step(ACTIONS.index((0, 0)))
                if env.game_state.lives != prev_lives:
                    direction = START_DIR
                grid = env.render()
                visualizer.draw_grid(grid)
                time.sleep(SLEEP)
            for _ in range(action[1] - 1):
                prev_lives = env.game_state.lives
                obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
                if env.game_state.lives != prev_lives:
                    direction = START_DIR
                grid = env.render()
                visualizer.draw_grid(grid)
                time.sleep(SLEEP)
        elif action[0] == 1: # turn left
            direction = LEFT[ACTIONS.index(tuple(direction))]
            prev_lives = env.game_state.lives
            obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
            if env.game_state.lives != prev_lives:
                direction = START_DIR
            grid = env.render()
            visualizer.draw_grid(grid)
            time.sleep(SLEEP)
        elif action[0] == -1: # turn right
            direction = ACTIONS[LEFT.index(tuple(direction))]
            prev_lives = env.game_state.lives
            obs, reward, done, _ = env.step(ACTIONS.index(tuple(direction)))
            if env.game_state.lives != prev_lives:
                direction = START_DIR
            grid = env.render()
            visualizer.draw_grid(grid)
            time.sleep(SLEEP)

            
        # for row in obs[11]:
        #     for cell in row:
        #         print('1' if cell else '0', end='')
        #     print()
        # print(reward, done)

        if done:
            env.reset()
            env.render()
        key = visualizer.wait()
        