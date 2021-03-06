from simulator.gym_wrappers import PacBotEnv
from simulator.visualizer import Visualizer

if __name__ == "__main__":
    visualizer = Visualizer()
    env = PacBotEnv(speed=1)
    obs = env.reset()
    grid = env.render()
    visualizer.draw_grid(grid)

    done = False
    key = visualizer.wait()
    while key != "q":
        action = 2
        if key == "w":
            action = 0
        elif key == "a":
            action = 1
        elif key == "d":
            action = 3
        elif key == "s":
            action = 4
        obs, reward, done, _ = env.step(action)
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
        key = visualizer.wait()