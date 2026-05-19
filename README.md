# Robot Lab RL

This project is a 2D robotics sandbox for abstract Pi-puck-like robots. The
robots are red circles with differential-drive behavior, moving inside a
2.0 m x 1.0 m arena. The manual viewer is pure 2D and includes movable blue
blocks and yellow balls that can be pushed around by the robots.

The robot action format is:

```python
[left_wheel_speed, right_wheel_speed]
```

The robot observation format is:

```python
[x_position, y_position, yaw]
```

Only `x`, `y`, and yaw change. Robots cannot tip, roll, bounce, or drift in
height.

## Requirements

Use Conda on macOS. `pip install pybullet` may fail because pip tries to compile
PyBullet from source.

Check that Conda is available:

```bash
conda --version
```

## Setup

From the repository root, create the local Conda environment:

```bash
conda env create -p "$PWD/.conda-env" -f environment.yml
```

Install this project into that environment in editable mode:

```bash
"$PWD/.conda-env/bin/python" -m pip install -e .
```

You can now run all project commands with:

```bash
"$PWD/.conda-env/bin/python" <script-or-command>
```

Optional: activate the environment instead:

```bash
conda activate "$PWD/.conda-env"
```

Then commands can be run as plain `python ...`.

## Run the 2D Manual Sandbox

Start the main sandbox:

```bash
"$PWD/.conda-env/bin/python" scripts/manual_drive.py
```

This opens a pure 2D canvas, not the PyBullet 3D viewer.

You should see:

- a grey 2.0 m x 1.0 m arena
- two red circular robots labeled `P0` and `P1`
- black arrows showing each robot's front direction
- blue movable blocks
- yellow movable balls
- a bottom-right coordinate key showing each robot's `x`, `y`, and yaw
- a speed slider below the canvas

Controls:

- `P0`: `I` / `K` forward/backward, `J` / `L` turn left/right
- `P1`: arrow up/down forward/backward, arrow left/right turn
- speed slider: drag left/right to change keyboard driving speed from 10% to 100%

The robots use differential drive. Driving straight sets both wheels to the
same speed. Turning gives the left and right wheels different speeds.

## Run the Headless Simulation

Run a short random-policy simulation without graphics:

```bash
"$PWD/.conda-env/bin/python" scripts/run_random_policy.py
```

This is useful for checking that the Gymnasium-style environment runs without
opening the 2D manual viewer.

## Optional PyBullet GUI Demo

The project still supports opening PyBullet's GUI for debugging:

```bash
"$PWD/.conda-env/bin/python" scripts/run_random_policy.py --gui
```

The main manual sandbox does not use this because PyBullet's GUI is a 3D viewer.

## Run Tests

Run the test suite:

```bash
"$PWD/.conda-env/bin/python" -m pytest
```

Current tests check:

- differential-drive math
- strict 2D robot motion
- 2.0 m x 1.0 m arena bounds
- boundary clipping
- keyboard control mapping
- creation of movable blocks and balls
- pushing a block with a robot

## Project Structure

- `environment.yml`: Conda environment definition.
- `requirements.txt`: pip dependency list for systems where PyBullet installs cleanly.
- `pyproject.toml`: local editable-package configuration.
- `src/robot_lab_rl/envs/circle_robot_env.py`: Gymnasium-style 2D robot environment.
- `scripts/manual_drive.py`: pure 2D manual-driving sandbox.
- `scripts/run_random_policy.py`: random-action simulation runner.
- `tests/test_differential_drive.py`: movement, arena, keyboard, and object-physics tests.

## Notes

The current object physics are lightweight 2D physics: objects have position,
velocity, mass, wall collisions, damping, and push interactions. Rectangular
blocks also have orientation and angular velocity, so off-center pushes rotate
them around their center of mass. This is still not a full rigid-body engine,
but it is sufficient for testing pushing behavior before the RL task is defined.

## Next Milestones

1. Add target zones.
2. Add simple robot distance sensors.
3. Add goals for moving blocks or balls to target zones.
4. Define a reward function.
5. Train a first reinforcement learning policy.
