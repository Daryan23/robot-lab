"""Generate report-ready figures for the decentralized shared box-push policy.

Clean/modern matplotlib styling. Produces PNGs under ``figures/``:

* fig_trajectory.png          -- top-down arena with the box path to the zone.
* fig_success_vs_random.png   -- success vs spawn randomization, residual
                                 (untrained / optionally trained) vs scratch.
* fig_learning_curve.png      -- success rate over PPO timesteps.
* fig_reward_curve.png        -- mean episode reward over PPO timesteps
                                 (the same signal TensorBoard shows as
                                 rollout/ep_rew_mean), from the same run.

Examples:
    # fast, no training:
    python scripts/plot_report_figures.py --only trajectory
    python scripts/plot_report_figures.py --only success
    # report-quality learning + reward curve (slow):
    python scripts/plot_report_figures.py --only curve \
        --curve-timesteps 400000 --curve-eval-every 40000 --eval-episodes 30
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Rectangle
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.utils import safe_mean
from stable_baselines3.common.vec_env import VecMonitor

from robot_lab_rl import BoxPushEnv
from robot_lab_rl.decentralized import (
    SharedBoxPushVecEnv,
    egocentric_observation,
    make_shared_box_push_vec_env,
)

# --- clean/modern style -----------------------------------------------------
PRIMARY = "#2563eb"   # blue
SECONDARY = "#ef4444" # red
ACCENT = "#10b981"    # green
MUTED = "#9ca3af"     # gray


def save_table(path, header, rows) -> None:
    """Write a whitespace-separated data table for LaTeX/pgfplots.

    ``header`` is a list of column names, ``rows`` an iterable of equal-length
    rows. Read it in LaTeX with ``\\addplot table[x=..., y=...] {file.dat};``.
    """
    cols = list(header)
    with open(path, "w") as fh:
        fh.write(" ".join(cols) + "\n")
        for row in rows:
            fh.write(" ".join(_fmt(v) for v in row) + "\n")
    print(f"Saved {path}")


def _fmt(value) -> str:
    if isinstance(value, float):
        return "nan" if np.isnan(value) else f"{value:.6g}"
    return str(value)


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#444444",
            "axes.linewidth": 1.0,
            "axes.grid": True,
            "axes.axisbelow": True,
            "grid.color": "#e5e7eb",
            "grid.linewidth": 1.0,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.family": "sans-serif",
            "font.size": 12,
            "axes.titlesize": 14,
            "axes.titleweight": "bold",
            "axes.labelsize": 12,
            "legend.frameon": False,
            "lines.linewidth": 2.4,
            "lines.markersize": 7,
            "figure.dpi": 120,
            "savefig.dpi": 200,
            "savefig.bbox": "tight",
        }
    )


# --- evaluation helpers -----------------------------------------------------
def _new_eval_env(difficulty, randomization, residual_scale, max_steps=800):
    env = BoxPushEnv(
        render_mode="direct",
        difficulty=difficulty,
        max_steps=max_steps,
        randomization=randomization,
    )
    blender = SharedBoxPushVecEnv(env, residual_scale=residual_scale)
    return env, blender


def evaluate_success(model, difficulty, randomization, residual_scale, episodes, max_steps=800):
    env, blender = _new_eval_env(difficulty, randomization, residual_scale, max_steps)
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


def fresh_model(difficulty, residual_scale, monitor=False):
    vec = make_shared_box_push_vec_env(difficulty, max_steps=800, residual_scale=residual_scale)
    env = VecMonitor(vec) if monitor else vec
    model = PPO("MlpPolicy", env, verbose=0, policy_kwargs={"net_arch": [128, 128]})
    return model, env


def load_model(path):
    return PPO.load(path)


# --- figure: box trajectory -------------------------------------------------
def figure_trajectory(args, figures_dir):
    model = load_model(args.trained_model) if args.trained_model else fresh_model("full", args.residual_scale)[0]
    env, blender = _new_eval_env("full", args.trajectory_randomization, args.residual_scale)

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    half_w, half_h = env.arena_width / 2.0, env.arena_height / 2.0
    ax.add_patch(Rectangle((-half_w, -half_h), env.arena_width, env.arena_height,
                           fill=False, edgecolor="#444444", linewidth=1.5))

    cmap = plt.get_cmap("viridis")
    n_episodes = args.trajectory_episodes
    traj_rows = []  # (episode, step, x, y) for LaTeX export
    for ep in range(n_episodes):
        env.reset()
        zx, zy = env.zone_position
        # zone rectangle (drawn once, but positions may jitter -> draw per ep faint)
        if ep == 0:
            ax.add_patch(Rectangle((zx - env.zone_half_width, zy - env.zone_half_height),
                                   2 * env.zone_half_width, 2 * env.zone_half_height,
                                   facecolor=ACCENT, alpha=0.25, edgecolor=ACCENT, linewidth=1.5,
                                   label="target zone"))
        path = [(env.box.x, env.box.y)]
        for _ in range(env.max_steps):
            obs = np.stack([egocentric_observation(env, i) for i in range(env.num_robots)])
            a, _ = model.predict(obs, deterministic=True)
            env.step(blender.actions_to_wheel_speeds(env, a))
            path.append((env.box.x, env.box.y))
            if env.box_in_zone():
                break
        path = np.array(path)
        for step, (px, py) in enumerate(path):
            traj_rows.append((ep, step, px, py))
        color = cmap(ep / max(1, n_episodes - 1))
        ax.plot(path[:, 0], path[:, 1], color=color, alpha=0.9, linewidth=2.2,
                label="box path" if ep == 0 else None)
        ax.scatter(path[0, 0], path[0, 1], color=color, marker="o", s=45, zorder=5,
                   edgecolor="white", linewidth=1.0)
        ax.scatter(path[-1, 0], path[-1, 1], color=color, marker="*", s=160, zorder=5,
                   edgecolor="white", linewidth=1.0)
    env.close()

    ax.set_xlim(-half_w - 0.05, half_w + 0.05)
    ax.set_ylim(-half_h - 0.05, half_h + 0.05)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = "Box trajectories to the zone"
    title += " (trained)" if args.trained_model else " (untrained residual)"
    ax.set_title(title)
    ax.legend(loc="upper left")
    ax.text(0.0, -half_h - 0.02, "● start    ★ end", ha="center", va="top",
            fontsize=10, color="#444444")
    path_out = figures_dir / "fig_trajectory.png"
    fig.savefig(path_out)
    plt.close(fig)
    print(f"Saved {path_out}")
    save_table(figures_dir / "fig_trajectory.dat", ["episode", "step", "x", "y"], traj_rows)


# --- figure: success vs randomization ---------------------------------------
def figure_success_vs_random(args, figures_dir):
    randomizations = [0.0, 0.05, 0.10, 0.15, 0.20]

    residual_model = fresh_model("full", args.residual_scale)[0]
    scratch_model = fresh_model("full", 0.0)[0]
    series = {
        "Residual expert (untrained)": (
            [evaluate_success(residual_model, "full", r, args.residual_scale, args.eval_episodes)
             for r in randomizations], PRIMARY, "-"),
        "From scratch (untrained)": (
            [evaluate_success(scratch_model, "full", r, 0.0, args.eval_episodes)
             for r in randomizations], MUTED, "--"),
    }
    if args.trained_model:
        trained = load_model(args.trained_model)
        series["Residual expert (trained)"] = (
            [evaluate_success(trained, "full", r, args.residual_scale, args.eval_episodes)
             for r in randomizations], ACCENT, "-")

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    for label, (rates, color, ls) in series.items():
        ax.plot(randomizations, rates, marker="o", color=color, linestyle=ls, label=label)
    ax.set_xlabel("spawn randomization")
    ax.set_ylabel("success rate (full task)")
    ax.set_ylim(-0.03, 1.05)
    ax.set_title("Robustness to randomized spawns")
    ax.legend(loc="upper right")
    path_out = figures_dir / "fig_success_vs_random.png"
    fig.savefig(path_out)
    plt.close(fig)
    print(f"Saved {path_out}")

    labels = list(series.keys())
    rows = [
        [randomizations[i]] + [series[label][0][i] for label in labels]
        for i in range(len(randomizations))
    ]
    header = ["randomization"] + [
        label.replace(" ", "_").replace("(", "").replace(")", "") for label in labels
    ]
    save_table(figures_dir / "fig_success_vs_random.dat", header, rows)


# --- figure: learning curve + reward curve ----------------------------------
class CurveCallback(BaseCallback):
    def __init__(self, difficulty, randomization, residual_scale, eval_every, episodes):
        super().__init__()
        self.difficulty = difficulty
        self.randomization = randomization
        self.residual_scale = residual_scale
        self.eval_every = eval_every
        self.episodes = episodes
        self.timesteps, self.success, self.reward = [], [], []
        self._last_eval = 0

    def _record(self):
        rate = evaluate_success(self.model, self.difficulty, self.randomization,
                                self.residual_scale, self.episodes)
        rew = (safe_mean([ep["r"] for ep in self.model.ep_info_buffer])
               if len(self.model.ep_info_buffer) > 0 else float("nan"))
        self.timesteps.append(self.num_timesteps)
        self.success.append(rate)
        self.reward.append(rew)
        print(f"  [curve] t={self.num_timesteps} success={rate:.0%} reward={rew:.1f}")

    def _on_step(self) -> bool:
        if self.num_timesteps - self._last_eval >= self.eval_every:
            self._last_eval = self.num_timesteps
            self._record()
        return True


def figure_curves(args, figures_dir):
    model, env = fresh_model("full", args.residual_scale, monitor=True)
    cb = CurveCallback("full", args.curve_randomization, args.residual_scale,
                       args.curve_eval_every, args.eval_episodes)
    start = evaluate_success(model, "full", args.curve_randomization, args.residual_scale,
                             args.eval_episodes)
    cb.timesteps.append(0); cb.success.append(start); cb.reward.append(float("nan"))
    print(f"  [curve] t=0 success={start:.0%}")
    model.learn(total_timesteps=args.curve_timesteps, callback=cb, progress_bar=False)
    env.close()

    def smooth(values, k=5):
        # Edge-aware moving average (no convolution droop at the ends).
        v = np.array(values, dtype=float)
        if len(v) < 3:
            return v
        half = max(1, k // 2)
        return np.array([v[max(0, i - half): i + half + 1].mean() for i in range(len(v))])

    # learning curve
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.plot(cb.timesteps, cb.success, marker="o", color=MUTED, alpha=0.5, label="raw")
    ax.plot(cb.timesteps, smooth(cb.success), color=PRIMARY, label="smoothed")
    ax.set_xlabel("PPO training timesteps")
    ax.set_ylabel(f"success rate (full, rand {args.curve_randomization})")
    ax.set_ylim(-0.03, 1.05)
    ax.set_title("Learning curve: training improves robustness")
    ax.legend(loc="upper left")
    p1 = figures_dir / "fig_learning_curve.png"
    fig.savefig(p1); plt.close(fig); print(f"Saved {p1}")
    smoothed_success = smooth(cb.success)
    save_table(
        figures_dir / "fig_learning_curve.dat",
        ["timesteps", "success_raw", "success_smoothed"],
        zip(cb.timesteps, cb.success, smoothed_success),
    )

    # reward curve
    ts = [t for t, r in zip(cb.timesteps, cb.reward) if not np.isnan(r)]
    rw = [r for r in cb.reward if not np.isnan(r)]
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.plot(ts, rw, marker="o", color=MUTED, alpha=0.5, label="raw")
    ax.plot(ts, smooth(rw), color=SECONDARY, label="smoothed")
    ax.set_xlabel("PPO training timesteps")
    ax.set_ylabel("mean episode reward")
    ax.set_title("Mean episode reward (ep_rew_mean)")
    ax.legend(loc="lower right")
    p2 = figures_dir / "fig_reward_curve.png"
    fig.savefig(p2); plt.close(fig); print(f"Saved {p2}")
    save_table(
        figures_dir / "fig_reward_curve.dat",
        ["timesteps", "reward_raw", "reward_smoothed"],
        zip(ts, rw, smooth(rw)),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--residual-scale", type=float, default=0.3)
    parser.add_argument("--eval-episodes", type=int, default=20)
    parser.add_argument("--trained-model", type=Path, default=None,
                        help="Optional trained shared-policy .zip to add to plots.")
    parser.add_argument("--trajectory-episodes", type=int, default=6)
    parser.add_argument("--trajectory-randomization", type=float, default=0.12)
    parser.add_argument("--curve-timesteps", type=int, default=400_000)
    parser.add_argument("--curve-eval-every", type=int, default=40_000)
    parser.add_argument("--curve-randomization", type=float, default=0.15)
    parser.add_argument("--figures-dir", type=Path, default=Path("figures"))
    parser.add_argument("--only", choices=["all", "trajectory", "success", "curve"], default="all")
    args = parser.parse_args()

    apply_style()
    args.figures_dir.mkdir(parents=True, exist_ok=True)
    if args.only in ("all", "trajectory"):
        figure_trajectory(args, args.figures_dir)
    if args.only in ("all", "success"):
        figure_success_vs_random(args, args.figures_dir)
    if args.only in ("all", "curve"):
        figure_curves(args, args.figures_dir)


if __name__ == "__main__":
    main()
