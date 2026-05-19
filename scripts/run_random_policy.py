import argparse
import time

import numpy as np

from robot_lab_rl import CircleRobotEnv


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the circular robot with random wheel speeds.")
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet visualizer.")
    parser.add_argument("--steps", type=int, default=600, help="Number of simulation steps.")
    args = parser.parse_args()

    env = CircleRobotEnv(render_mode="human" if args.gui else "direct", max_steps=args.steps)
    observation, _ = env.reset()
    print(f"Initial observation [p0_x, p0_y, p0_yaw, p1_x, p1_y, p1_yaw, ...]: {observation}")

    for step in range(args.steps):
        action = np.random.uniform(
            low=env.action_space.low,
            high=env.action_space.high,
        )
        observation, reward, terminated, truncated, _ = env.step(action)

        if step % 60 == 0:
            print(f"step={step:03d} action={action.round(3)} observation={observation.round(3)}")

        if args.gui:
            time.sleep(env.time_step)
        if terminated or truncated:
            break

    env.close()


if __name__ == "__main__":
    main()
