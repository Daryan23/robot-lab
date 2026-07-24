  # Robot Lab RL — Project Overview

  A handoff document describing what this project does, conceptually and
  technically. Intended to be pasted into another AI model as context.

  ## Conceptual Perspective

  **Goal:** A 2D robotics sandbox where two (or more) differential-drive robots
  (abstracted Pi-puck robots, drawn as red circles) learn to **cooperatively push
  a box into a target zone** — a classic multi-agent manipulation task. Arena is
  2.0 m × 1.0 m.

  ### Core learning idea — Residual Policy Learning

  Instead of learning a policy from scratch, there is a hand-written **geometric
  expert** (`BoxPushHeuristicPolicy`) that can already solve the task. The expert
  stays in the control loop. PPO (reinforcement learning) only learns a **bounded
  correction** on top of it:

  ```
  final_action = expert_action + residual_scale · policy_correction
  ```

  Consequences:
  - An untrained residual policy already performs at expert level (measured: 10/10
    solved on the `full` task with `--residual-scale 0.3`).
  - PPO only **refines** the controller and adds robustness to randomized spawns.
  - No behavior-cloning warm-start is needed. The BC / demo-collection scripts
    were deliberately removed (this is the `feature/residual-only-training`
    branch).
  - **Critical:** a residual model only behaves correctly when loaded with the
    **same `--residual-scale`** it was trained with.

  ### Decentralized parameter-sharing architecture

  Rather than one central policy that sees everything and controls both robots,
  there is **one shared network** called **once per robot** with that robot's own
  **egocentric** (local, relative) observation. The `n` robots are presented to
  Stable-Baselines3 as `n` parallel sub-environments of a single `VecEnv`, so one
  network learns from the pooled experience of all robots (shared team reward).
  Because the network only ever sees a single robot's local view, execution stays
  decentralized and the same network **scales to n > 2 robots unchanged**.

  ### Curriculum learning

  Training proceeds in stages (`easy` → `medium` → `full`) where robots and box
  start progressively farther from the zone; the model is carried forward across
  stages. During curriculum training, full-task eval can read 0% while the current
  easy phase is working — judge each phase by its own `eval_current_<phase>/*`.

  ## Technical Perspective

  ### Stack
  - Python, **Conda** (not pip — `pip install pybullet` tries to compile from
    source and fails on macOS), editable install (`pip install -e .`).
  - **Gymnasium**-style environment, **Stable-Baselines3 (PPO)**, TensorBoard.
  - Custom **lightweight 2D physics** (position, velocity, mass, wall collisions,
    damping, push interactions; rectangular blocks also carry orientation +
    angular velocity, so off-center pushes rotate them about their center of
    mass). This is **not** a full rigid-body engine.
  - Pure 2D viewer (not the PyBullet 3D viewer); optional PyBullet GUI debug mode.

  ### Interfaces
  - Robot action: `[left_wheel_speed, right_wheel_speed]`; normalized to `[-1, 1]`
    for SB3.
  - Robot observation: `[x, y, yaw]`. Only x, y, yaw change — robots cannot tip,
    roll, bounce, or move in height (strict 2D).

  **Box-push global observation (centralized view):**
  ```
  [r0_x, r0_y, r0_yaw, r1_x, r1_y, r1_yaw, box_x, box_y, box_angle, zone_x, zone_y]
  ```

  **Egocentric observation (decentralized view):** 11 self/box/zone features + 3
  numbers per neighbor (relative x, y, heading; default 2 nearest neighbors) + 3
  box-geometry numbers. World-frame vectors are rotated into the robot's heading
  frame (`_to_robot_frame`, +x = straight ahead) so the same policy rule works
  regardless of orientation.

  ### Reward shaping (dense)
  Positive: box moves closer to zone; robots move to useful push positions behind
  the box; robots reduce distance to box; useful heading; being behind the box on
  the side opposite the zone; useful pushing contact; large success bonus when the
  box is fully inside the zone. Negative: box pushed away from zone; time penalty;
  action penalty; strong timeout penalty. Episode ends on success, step limit, or
  no-useful-progress timeout.

  ### Key modules (`src/robot_lab_rl/`)
  - `envs/box_push_env.py` — the RL task (2 robots, 1 box, 1 zone).
  - `envs/circle_robot_env.py` — base 2D robot environment.
  - `expert.py` — geometric expert: distribute push points behind the box, greedy
    robot-to-point assignment.
  - `decentralized.py` — egocentric observation, parameter-sharing `VecEnv`, and
    residual policy learning.
  - `rl.py` — Stable-Baselines3 wrappers and checkpoint helpers.

  ### Key scripts (`scripts/`)
  - `train_shared_ppo.py` — **recommended**: decentralized + residual + curriculum.
  - `train_box_push_ppo.py` — centralized PPO (residual, or scratch baseline).
  - `watch_shared_policy.py`, `watch_box_push_policy.py`,
    `watch_box_push_expert.py`, `watch_training_progress.py` (live checkpoint
    viewer while training runs).
  - `evaluate_box_push.py`, `run_demos.py`, `plot_report_figures.py`.
  - `manual_drive.py` — pure 2D manual-driving sandbox (`--task box-push`).

  ### Representative commands
  Train the recommended decentralized residual policy:
  ```bash
  "$PWD/.conda-env/bin/python" scripts/train_shared_ppo.py \
    --difficulty full --timesteps 1000000 \
    --randomization 0.15 --residual-scale 0.3
  ```
  Watch it (residual-scale MUST match training):
  ```bash
  "$PWD/.conda-env/bin/python" scripts/watch_shared_policy.py \
    --model models/box_push_shared_ppo/box_push_shared_ppo_full_final.zip \
    --difficulty full --randomization 0.15 --residual-scale 0.3
  ```
  Tests: `"$PWD/.conda-env/bin/python" -m pytest`

  ### Gotchas
  - macOS `OMP: Error #15` (duplicate OpenMP runtimes) → prefix commands with
    `KMP_DUPLICATE_LIB_OK=TRUE`.
  - Models and TensorBoard logs are git-ignored; regenerate via the train
    commands.

  ### Open milestones
  1. Train the curriculum policy until full-task eval success is repeatable.
  2. Tune reward weights (avoid spinning, mutual blocking, ignoring the box).
  3. Randomized box/zone spawns after the fixed task works.
  4. Add a harder curriculum stage with randomized starts.
  5. Optional distance sensors later if needed.
