"""Generate report-ready figures for the decentralized shared box-push policy.

Produces three PNGs under ``figures/``:

* fig1_residual_vs_scratch.png -- untrained success rate per difficulty,
  residual-expert policy vs. a plain (from-scratch) policy. Shows that the
  residual policy starts at expert level while the plain policy is at zero.
* fig2_robustness_vs_randomization.png -- untrained residual success as the
  spawn randomization grows, motivating why training/robustness is needed.
* fig3_learning_curve.png -- success rate over PPO training timesteps for the
  residual policy trained with randomization (learning closes the gap).

Example:
    python scripts/plot_report_figures.py --eval-episodes 20 \
        --curve-timesteps 200000 --curve-eval-every 25000
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

from robot_lab_rl import BoxPushEnv
from robot_lab_rl.decentralized import (
    SharedBoxPushVecEnv,
    egocentric_observation,
    make_shared_box_push_vec_env,
)


def evaluate_success(model, difficulty, randomization, residual_scale, episodes, max_steps=800):
    """Deterministic success rate of a model on one difficulty."""
    env = BoxPushEnv(
        render_mode="direct",
        difficulty=difficulty,
        max_steps=max_steps,
        randomization=randomization,
    )
    blender = SharedBoxPushVecEnv(env, residual_scale=residual_scale)
    successes = 0
    for _ in range(episodes):
        env.reset()
        info = {}
        for _ in range(env.max_steps):
            observations = np.stack(
                [egocentric_observation(env, i) for i in range(env.num_robots)]
            )
            actions, _ = model.predict(observations, deterministic=True)
            _, _, terminated, truncated, info = env.step(
                blender.actions_to_wheel_speeds(env, actions)
            )
            if terminated or truncated:
                break
        successes += int(info.get("box_in_zone", False))
    env.close()
    return successes / episodes


def fresh_model(difficulty, residual_scale):
    env = make_shared_box_push_vec_env(difficulty, max_steps=800, residual_scale=residual_scale)
    model = PPO("MlpPolicy", env, verbose=0, policy_kwargs={"net_arch": [128, 128]})
    return model, env


def figure_residual_vs_scratch(args, figures_dir):
    difficulties = ["easy", "medium", "full"]
    residual_rates, scratch_rates = [], []
    for difficulty in difficulties:
        model_r, env_r = fresh_model(difficulty, residual_scale=args.residual_scale)
        residual_rates.append(
            evaluate_success(model_r, difficulty, 0.0, args.residual_scale, args.eval_episodes)
        )
        env_r.close()
        model_s, env_s = fresh_model(difficulty, residual_scale=0.0)
        scratch_rates.append(evaluate_success(model_s, difficulty, 0.0, 0.0, args.eval_episodes))
        env_s.close()

    x = np.arange(len(difficulties))
    width = 0.38
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.bar(x - width / 2, scratch_rates, width, label="From scratch (random net)", color="#bbbbbb")
    ax.bar(
        x + width / 2,
        residual_rates,
        width,
        label=f"Residual expert (scale {args.residual_scale})",
        color="#2a7fff",
    )
    ax.set_xticks(x, difficulties)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Success rate")
    ax.set_title("Untrained policy: residual expert vs. from scratch")
    ax.legend()
    for i, value in enumerate(residual_rates):
        ax.text(i + width / 2, value + 0.02, f"{value:.0%}", ha="center", fontsize=9)
    fig.tight_layout()
    path = figures_dir / "fig1_residual_vs_scratch.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def figure_robustness(args, figures_dir):
    randomizations = [0.0, 0.05, 0.10, 0.15, 0.20]
    model, env = fresh_model("full", residual_scale=args.residual_scale)
    rates = [
        evaluate_success(model, "full", r, args.residual_scale, args.eval_episodes)
        for r in randomizations
    ]
    env.close()

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.plot(randomizations, rates, marker="o", color="#2a7fff")
    ax.set_xlabel("Spawn randomization")
    ax.set_ylabel("Success rate (full, untrained)")
    ax.set_ylim(0, 1.05)
    ax.set_title("Bare heuristic degrades as spawns become random")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = figures_dir / "fig2_robustness_vs_randomization.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


class SuccessCurveCallback(BaseCallback):
    """Periodically record deterministic success rate during training."""

    def __init__(self, difficulty, randomization, residual_scale, eval_every, episodes):
        super().__init__()
        self.difficulty = difficulty
        self.randomization = randomization
        self.residual_scale = residual_scale
        self.eval_every = eval_every
        self.episodes = episodes
        self.timesteps = []
        self.success_rates = []

    def _on_step(self) -> bool:
        if self.num_timesteps - getattr(self, "_last_eval", 0) >= self.eval_every:
            self._last_eval = self.num_timesteps
            rate = evaluate_success(
                self.model,
                self.difficulty,
                self.randomization,
                self.residual_scale,
                self.episodes,
            )
            self.timesteps.append(self.num_timesteps)
            self.success_rates.append(rate)
            print(f"  [curve] t={self.num_timesteps} success={rate:.0%}")
        return True


def figure_learning_curve(args, figures_dir):
    model, env = fresh_model("full", residual_scale=args.residual_scale)
    callback = SuccessCurveCallback(
        difficulty="full",
        randomization=args.curve_randomization,
        residual_scale=args.residual_scale,
        eval_every=args.curve_eval_every,
        episodes=args.eval_episodes,
    )
    # Record the starting point (untrained) too.
    start_rate = evaluate_success(
        model, "full", args.curve_randomization, args.residual_scale, args.eval_episodes
    )
    callback.timesteps.append(0)
    callback.success_rates.append(start_rate)
    print(f"  [curve] t=0 success={start_rate:.0%}")
    model.learn(total_timesteps=args.curve_timesteps, callback=callback, progress_bar=False)
    env.close()

    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.plot(callback.timesteps, callback.success_rates, marker="o", color="#2a7fff")
    ax.set_xlabel("PPO training timesteps")
    ax.set_ylabel(f"Success rate (full, rand {args.curve_randomization})")
    ax.set_ylim(0, 1.05)
    ax.set_title("Residual policy: training adds robustness to random spawns")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = figures_dir / "fig3_learning_curve.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--residual-scale", type=float, default=0.3)
    parser.add_argument("--eval-episodes", type=int, default=20)
    parser.add_argument("--curve-timesteps", type=int, default=200_000)
    parser.add_argument("--curve-eval-every", type=int, default=25_000)
    parser.add_argument("--curve-randomization", type=float, default=0.15)
    parser.add_argument("--figures-dir", type=Path, default=Path("figures"))
    parser.add_argument(
        "--only",
        choices=["all", "bars", "robustness", "curve"],
        default="all",
        help="Generate only one figure (the learning curve is the slow one).",
    )
    args = parser.parse_args()

    args.figures_dir.mkdir(parents=True, exist_ok=True)
    if args.only in ("all", "bars"):
        figure_residual_vs_scratch(args, args.figures_dir)
    if args.only in ("all", "robustness"):
        figure_robustness(args, args.figures_dir)
    if args.only in ("all", "curve"):
        figure_learning_curve(args, args.figures_dir)


if __name__ == "__main__":
    main()
