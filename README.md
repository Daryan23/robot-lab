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

## Run the Box-to-Zone Task

Start the simple RL-ready task:

```bash
"$PWD/.conda-env/bin/python" scripts/manual_drive.py --task box-push
```

This mode contains:

- exactly 2 robots
- exactly 1 movable blue box
- exactly 1 green target zone
- no balls or extra random blocks

The task objective is to push the box into the green zone. This is the first
environment intended for reinforcement learning.

Box-push observation format:

```python
[
    robot0_x, robot0_y, robot0_yaw,
    robot1_x, robot1_y, robot1_yaw,
    box_x, box_y, box_angle,
    zone_x, zone_y,
]
```

Box-push action format:

```python
[
    [robot0_left_wheel_speed, robot0_right_wheel_speed],
    [robot1_left_wheel_speed, robot1_right_wheel_speed],
]
```

Reward:

- positive reward when the box moves closer to the zone
- positive reward when robots move toward useful push positions behind the box
- positive reward when robots reduce their distance to the box
- tiny reward when robots face useful approach or push directions
- tiny reward when robots are behind the box on the opposite side from the zone
- small contact reward only when contact is useful for pushing toward the zone
- penalty if the box is pushed away from the zone
- time and action penalties
- strong penalty when an episode times out without success
- large success bonus when the full box is inside the rectangular zone

The episode ends when the box reaches the zone, the max step limit is reached,
or the box has made no useful progress for too long.

## Run the Headless Simulation

Run a short random-policy simulation without graphics:

```bash
"$PWD/.conda-env/bin/python" scripts/run_random_policy.py
```

This is useful for checking that the Gymnasium-style environment runs without
opening the 2D manual viewer.

## Gated Prior and Pretraining

Do not start PPO from a random policy, and do not continue from a weak behavior
cloned model. The recommended flow is now gated:

1. verify the scripted expert can solve the easy task
2. collect successful expert states with small start-position randomization
3. behavior-clone the PPO policy
4. run DAgger-style correction, where the policy visits its own bad states and
   the expert labels what it should have done there
5. save `models/box_push_ppo/box_push_prior.zip` only after evaluation

Run the full prior builder:

```bash
"$PWD/.conda-env/bin/python" scripts/build_box_push_prior.py \
  --difficulty easy \
  --seed-demo-steps 30000 \
  --dagger-iterations 6 \
  --dagger-episodes 20 \
  --epochs-per-iteration 12 \
  --eval-episodes 20 \
  --min-success-rate 0.80
```

This command should print gates like:

```text
expert gate: success=...
seed BC eval: success=...
dagger rollout 1: success=...
prior eval 1: success=...
final prior gate: success=...
```

Only continue to PPO if `final prior gate` is at least `80%` on easy. If it
fails, increase `--dagger-iterations` first; do not train PPO from that model.

Watch the prior checkpoints while the prior builder runs:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_training_progress.py --source prior --difficulty easy
```

Evaluate the final prior:

```bash
"$PWD/.conda-env/bin/python" scripts/evaluate_box_push.py \
  --model models/box_push_ppo/box_push_prior.zip \
  --difficulty easy \
  --episodes 20
```

For debugging, you can still watch the hand-written expert directly:

Watch the expert directly:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_box_push_expert.py --difficulty easy
```

The older manual BC pieces are still available, but the gated prior builder is
preferred because plain BC can overfit a small repeated trajectory and fail
when the policy drifts away from expert states.

Collect expert demonstrations manually:

```bash
"$PWD/.conda-env/bin/python" scripts/collect_box_push_demos.py --steps 200000
```

By default this collects successful `easy` demonstrations only. That is
intentional: the current heuristic is a reliable warm-start expert for easy
pushing, while medium/full are left for PPO fine-tuning. To include non-success
progress trajectories for analysis, add `--difficulty all --include-progress-episodes`.

Optionally watch collection while also saving the demo file:

```bash
"$PWD/.conda-env/bin/python" scripts/collect_box_push_demos.py \
  --visual \
  --difficulty easy \
  --steps 2000
```

Pretrain the policy with behavior cloning:

```bash
"$PWD/.conda-env/bin/python" scripts/pretrain_box_push_bc.py \
  --demo-path data/box_push_demos.npz \
  --epochs 50 \
  --checkpoint-every 5
```

BC saves checkpoints in `models/box_push_ppo/`, including
`box_push_bc_pretrained.zip`. The prior builder also updates that path for
compatibility, but its canonical output is `box_push_prior.zip`.

Watch BC checkpoints while pretraining runs:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_training_progress.py --source bc --difficulty easy
```

## Train the Box-Push Agent

Train PPO with the gated prior:

```bash
"$PWD/.conda-env/bin/python" scripts/train_box_push_ppo.py \
  --pretrained-model models/box_push_ppo/box_push_prior.zip
```

The default is now `2,000,000` timesteps with 8 parallel environments. Training
is intentionally headless and fast; speed alone does not mean the policy is
good. PPO now checks the pretrained model before training; if the prior gets
0% on easy, training stops instead of wasting time.

For a serious first run, use:

```bash
"$PWD/.conda-env/bin/python" scripts/train_box_push_ppo.py \
  --pretrained-model models/box_push_ppo/box_push_prior.zip \
  --timesteps 5000000 \
  --num-envs 8 \
  --eval-every 100000 \
  --checkpoint-every 250000
```

Training saves:

- checkpoints in `models/box_push_ppo/`
- best full-task model as `models/box_push_ppo/box_push_ppo_best_full.zip`
- the final model as `models/box_push_ppo/box_push_ppo_final.zip`
- TensorBoard logs in `runs/box_push_ppo/`
- monitor logs in `runs/box_push_ppo/monitor_<difficulty>.csv`

Useful shorter command while testing code changes:

```bash
"$PWD/.conda-env/bin/python" scripts/train_box_push_ppo.py \
  --timesteps 10000 \
  --num-envs 2 \
  --n-steps 256 \
  --batch-size 128 \
  --eval-every 0
```

By default, training uses curriculum phases:

- `easy`: robots and box start close to the zone
- `medium`: robots start farther away
- `full`: the original full-distance task

To train only one phase:

```bash
"$PWD/.conda-env/bin/python" scripts/train_box_push_ppo.py \
  --pretrained-model models/box_push_ppo/box_push_prior.zip \
  --difficulty full \
  --timesteps 2000000
```

The RL wrapper gives Stable-Baselines3 normalized actions:

```python
[robot0_left, robot0_right, robot1_left, robot1_right]
```

Each value is in `[-1, 1]`.

Internally the environment still receives:

```python
[
    [robot0_left, robot0_right],
    [robot1_left, robot1_right],
]
```

## Watch Training Progress

Start TensorBoard in a second terminal:

```bash
"$PWD/.conda-env/bin/tensorboard" --logdir runs
```

Then open the printed local URL, usually `http://localhost:6006`.

Useful graphs:

- `rollout/ep_rew_mean`: average episode reward
- `rollout/ep_len_mean`: average episode length
- `box_push/success_rate_100`: recent completed-episode success rate for the current training phase
- `box_push/box_zone_distance_100`: recent distance from box to zone
- `box_push/push_setup_distance_100`: whether robots are learning to move behind the box
- `box_push/robot_box_distance_100`: whether robots are learning to reach the box
- `box_push/heading_alignment_100`: whether robots face useful movement directions
- `box_push/behind_box_quality_100`: whether robots are on the correct side of the box
- `box_push/contact_quality_100`: whether robots are making useful pushing contact
- `eval_current_<difficulty>/success_rate`: deterministic success rate on the phase currently being trained
- `eval_full/success_rate`: deterministic success rate on the full-distance task
- `eval_full/average_final_distance`: final box-zone distance on the full task

During curriculum training, full-task evaluation can stay at `0%` while the
current `easy` phase is working. Judge the current phase with
`eval_current_easy/*` until the run reaches the medium/full phases.

To visually watch the newest checkpoint while training continues, open another
terminal and run:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_training_progress.py
```

This opens the same pure 2D viewer and reloads the newest saved model every few
seconds. It shows deterministic evaluation behavior, not the noisy exploration
actions used internally during PPO training.

For early curriculum debugging, you can watch the easier stage:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_training_progress.py --difficulty easy
```

To watch PPO checkpoints explicitly:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_training_progress.py --source ppo --difficulty full
```

To watch the prior instead:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_training_progress.py --source prior --difficulty easy
```

## Evaluate a Trained Policy

After training, run deterministic evaluation:

```bash
"$PWD/.conda-env/bin/python" scripts/evaluate_box_push.py --episodes 20
```

To evaluate a specific checkpoint:

```bash
"$PWD/.conda-env/bin/python" scripts/evaluate_box_push.py \
  --model models/box_push_ppo/box_push_ppo_10000_steps.zip \
  --episodes 20
```

To evaluate easier curriculum stages:

```bash
"$PWD/.conda-env/bin/python" scripts/evaluate_box_push.py --difficulty easy --episodes 20
```

## Watch a Trained Policy

Open the pure 2D viewer and let the saved policy drive the robots:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_box_push_policy.py
```

To watch a specific checkpoint:

```bash
"$PWD/.conda-env/bin/python" scripts/watch_box_push_policy.py \
  --model models/box_push_ppo/box_push_ppo_10000_steps.zip
```

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
- box-push task spawn count, observation/action shapes, reward, success, and viewer rendering
- Stable-Baselines3 wrapper compatibility

## Project Structure

- `environment.yml`: Conda environment definition.
- `requirements.txt`: pip dependency list for systems where PyBullet installs cleanly.
- `pyproject.toml`: local editable-package configuration.
- `src/robot_lab_rl/envs/circle_robot_env.py`: Gymnasium-style 2D robot environment.
- `src/robot_lab_rl/envs/box_push_env.py`: simple 2-robot, 1-box, 1-zone RL task.
- `src/robot_lab_rl/expert.py`: scripted heuristic prior for demonstrations.
- `src/robot_lab_rl/rl.py`: Stable-Baselines3 wrappers and checkpoint helpers.
- `scripts/manual_drive.py`: pure 2D manual-driving sandbox.
- `scripts/run_random_policy.py`: random-action simulation runner.
- `scripts/collect_box_push_demos.py`: expert demonstration collection.
- `scripts/pretrain_box_push_bc.py`: behavior cloning pretraining.
- `scripts/build_box_push_prior.py`: gated expert, BC, and DAgger prior builder.
- `scripts/train_box_push_ppo.py`: PPO training entry point.
- `scripts/evaluate_box_push.py`: saved-policy evaluation.
- `scripts/watch_box_push_expert.py`: heuristic expert viewer.
- `scripts/watch_box_push_policy.py`: pure 2D saved-policy viewer.
- `scripts/watch_training_progress.py`: live checkpoint viewer while training runs.
- `tests/test_differential_drive.py`: movement, arena, keyboard, and object-physics tests.
- `tests/test_box_push_env.py`: box-push task tests.
- `tests/test_expert_and_bc.py`: expert, demo, BC, and checkpoint-source tests.
- `tests/test_rl_helpers.py`: RL wrapper and Stable-Baselines3 compatibility tests.

## Notes

The current object physics are lightweight 2D physics: objects have position,
velocity, mass, wall collisions, damping, and push interactions. Rectangular
blocks also have orientation and angular velocity, so off-center pushes rotate
them around their center of mass. This is still not a full rigid-body engine,
but it is sufficient for testing pushing behavior before the RL task is defined.

## Next Milestones

1. Train the curriculum policy until full-task evaluation success is repeatable.
2. Tune reward weights if the robots learn to spin, block each other, or ignore the box.
3. Add randomized box/zone spawn positions after the fixed task works.
4. Add a harder curriculum stage with randomized starts.
5. Add optional distance sensors later if needed.
