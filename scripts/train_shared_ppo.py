"""Train a decentralized, parameter-sharing PPO policy on the box-push task.

One shared network is called once per robot with that robot's egocentric
observation and trained on the pooled experience of all robots. Execution is
fully decentralized; the same network scales to ``n > 2`` robots unchanged.

Example:
    python scripts/train_shared_ppo.py --difficulty full --timesteps 1_000_000
"""

import argparse
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecMonitor

from robot_lab_rl.decentralized import make_shared_box_push_vec_env
from robot_lab_rl.envs.box_push_env import DIFFICULTIES

DEFAULT_MODEL_DIR = Path("models/box_push_shared_ppo")
DEFAULT_LOG_DIR = Path("runs/box_push_shared_ppo")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="full")
    parser.add_argument(
        "--curriculum",
        nargs="+",
        default=None,
        metavar="DIFFICULTY",
        help="Train through these difficulties in order, carrying the model "
        "forward (e.g. --curriculum easy medium full). Overrides --difficulty.",
    )
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--randomization", type=float, default=0.0)
    parser.add_argument(
        "--residual-scale",
        type=float,
        default=0.0,
        help="Residual Policy Learning: if > 0, the policy only adds a "
        "correction (this scale) on top of the geometric expert, starting at "
        "expert performance. Try 0.3. No BC warm-start needed in this mode.",
    )
    parser.add_argument("--learning-rate", type=float, default=2.5e-4)
    parser.add_argument("--ent-coef", type=float, default=0.01)
    parser.add_argument("--n-steps", type=int, default=2048)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument(
        "--pretrained-model",
        type=Path,
        default=None,
        help="Warm-start from a behavior-cloned model (see pretrain_shared_bc.py).",
    )
    args = parser.parse_args()

    args.model_dir.mkdir(parents=True, exist_ok=True)

    # TensorBoard logging is optional: only enable it if tensorboard is present.
    try:
        import tensorboard  # noqa: F401

        args.log_dir.mkdir(parents=True, exist_ok=True)
        tensorboard_log = str(args.log_dir)
    except ImportError:
        tensorboard_log = None
        print("tensorboard not installed; continuing without TensorBoard logging.")

    # Curriculum: a list of difficulties, the model is carried forward between
    # them. Without --curriculum it is a single phase on --difficulty.
    phases = args.curriculum if args.curriculum else [args.difficulty]
    timesteps_per_phase = max(1, args.timesteps // len(phases))

    def make_phase_env(difficulty: str) -> VecMonitor:
        return VecMonitor(
            make_shared_box_push_vec_env(
                difficulty=difficulty,
                max_steps=args.max_episode_steps,
                randomization=args.randomization,
                residual_scale=args.residual_scale,
            )
        )

    model: PPO | None = None
    vec_env: VecMonitor | None = None
    for difficulty in phases:
        print(f"\n=== Phase: {difficulty} for {timesteps_per_phase} timesteps ===")
        if vec_env is not None:
            vec_env.close()
        vec_env = make_phase_env(difficulty)

        if model is None:
            if args.pretrained_model is not None:
                # Warm-start: keep the behavior-cloned weights, continue with PPO.
                model = PPO.load(
                    args.pretrained_model,
                    env=vec_env,
                    tensorboard_log=tensorboard_log,
                )
                model.verbose = 1
                model.learning_rate = args.learning_rate
                model.ent_coef = args.ent_coef
                print(f"Warm-started from {args.pretrained_model}")
            else:
                model = PPO(
                    "MlpPolicy",
                    vec_env,
                    verbose=1,
                    tensorboard_log=tensorboard_log,
                    n_steps=args.n_steps,
                    batch_size=args.batch_size,
                    gamma=0.995,
                    gae_lambda=0.95,
                    ent_coef=args.ent_coef,
                    learning_rate=args.learning_rate,
                    clip_range=0.2,
                    max_grad_norm=0.5,
                    policy_kwargs={"net_arch": [128, 128]},
                )
        else:
            model.set_env(vec_env)

        model.learn(
            total_timesteps=timesteps_per_phase,
            tb_log_name="shared_ppo",
            reset_num_timesteps=False,
            progress_bar=False,
        )
        model.save(args.model_dir / f"box_push_shared_ppo_after_{difficulty}")

    assert model is not None
    final_path = args.model_dir / f"box_push_shared_ppo_{phases[-1]}_final"
    model.save(final_path)
    if vec_env is not None:
        vec_env.close()
    print(f"Saved shared-policy model to {final_path}.zip")


if __name__ == "__main__":
    main()
