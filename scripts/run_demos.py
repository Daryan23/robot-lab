"""Reproduce the box-push demos end-to-end with a single command.

For every requested difficulty this script:

1. trains a **Residual** policy (scripted expert stays in the loop, the network
   only learns a bounded correction),
2. trains a **PPO-from-scratch** policy (no expert, learns everything itself),
3. measures the **scripted expert's** own success rate (the baseline line), and
4. renders the report figures: the residual learning curves *and* a direct
   **PPO-vs-Residual** comparison figure -- one per difficulty.

A colleague who just pulled the repository can therefore reproduce all of our
demos, see the learning graphs, and see the PPO-vs-Residual difference for each
demo, without needing any of our (git-ignored) ``runs/`` or ``models/``.

Examples::

    # our default demos (the cooperative tasks), full length
    python scripts/run_demos.py

    # fast pipeline smoke test (few timesteps -- graphs are sparse but prove it runs)
    python scripts/run_demos.py --quick

    # pick difficulties explicitly
    python scripts/run_demos.py --difficulties coop_random coop_mixed full
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

# Make the sibling scripts and the installed package importable when run directly.
SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPTS_DIR.parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from evaluate_box_push import evaluate_one  # noqa: E402
from robot_lab_rl import BoxPushHeuristicPolicy  # noqa: E402

# The demos we actually produced figures for (see CHANGES.md). Any difficulty
# from src/robot_lab_rl/envs/box_push_env.py DIFFICULTIES also works.
DEFAULT_DEMOS = ["coop_random", "coop_mixed"]


class _ExpertModel:
    """Adapter so the scripted expert can be evaluated with ``evaluate_one``."""

    def __init__(self) -> None:
        self.expert = BoxPushHeuristicPolicy()

    def predict(self, observation, deterministic: bool = True):
        return self.expert.predict(observation), None


def expert_baseline_percent(difficulty: str, episodes: int, max_episode_steps: int) -> float:
    """Success rate (in %) of the hand-written expert alone on this difficulty."""
    result = evaluate_one(
        _ExpertModel(),
        difficulty,
        episodes,
        max_episode_steps,
        randomization=0.0,
        residual=False,
    )
    return result["success_rate"] * 100.0


def run(cmd: list[str]) -> None:
    print("\n$ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def train(
    difficulty: str,
    method: str,
    *,
    timesteps: int,
    eval_every: int,
    residual_scale: float,
    seed: int,
    runs_root: Path,
    models_root: Path,
) -> tuple[Path, Path]:
    """Train one policy (method = 'residual' or 'scratch'); return (run dir, model dir)."""
    run_dir = runs_root / f"demo_{difficulty}_{method}"
    model_dir = models_root / f"demo_{difficulty}_{method}"
    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "train_box_push_ppo.py"),
        "--difficulty", difficulty,
        "--timesteps", str(timesteps),
        "--eval-every", str(eval_every),
        "--no-full-eval",
        "--seed", str(seed),
        "--log-dir", str(run_dir),
        "--model-dir", str(model_dir),
    ]
    if method == "residual":
        cmd += ["--residual", "--residual-scale", str(residual_scale)]
    run(cmd)
    return run_dir, model_dir


def make_figures(
    difficulty: str,
    residual_run: Path,
    scratch_run: Path,
    baseline: float,
    output: Path,
    *,
    residual_model: Path,
    scratch_model: Path,
    residual_scale: float,
    episodes: int,
) -> None:
    # Residual learning curves (success / reward / correction / outcomes).
    run([
        sys.executable,
        str(SCRIPTS_DIR / "plot_training_progress.py"),
        "--logdir", str(residual_run),
        "--output", str(output),
        "--expert-baseline", f"{baseline:.0f}",
    ])
    # Learning-over-time comparison (context; training curves are noisy on random tasks).
    run([
        sys.executable,
        str(SCRIPTS_DIR / "plot_method_comparison.py"),
        "--run", f"Residual (Expert + Korrektur)={residual_run}",
        "--run", f"PPO von Null (ohne Expert)={scratch_run}",
        "--output", str(output),
        "--expert-baseline", f"{baseline:.0f}",
        "--title", f"PPO vs. Residual: {difficulty}",
    ])
    # Trustworthy headline: final deterministic success as a bar chart.
    run([
        sys.executable,
        str(SCRIPTS_DIR / "plot_final_comparison.py"),
        "--difficulty", difficulty,
        "--residual-model", str(residual_model),
        "--scratch-model", str(scratch_model),
        "--residual-scale", str(residual_scale),
        "--episodes", str(episodes),
        "--output", str(output),
    ])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--difficulties",
        nargs="+",
        default=DEFAULT_DEMOS,
        help=f"Which demos to reproduce (default: {' '.join(DEFAULT_DEMOS)}).",
    )
    parser.add_argument("--timesteps", type=int, default=250_000, help="PPO timesteps per training.")
    parser.add_argument("--eval-every", type=int, default=5_000, help="Evaluation/logging frequency.")
    parser.add_argument("--residual-scale", type=float, default=0.4, help="Bound on the learned correction.")
    parser.add_argument("--seed", type=int, default=0, help="Seed for every training (reproducibility).")
    parser.add_argument(
        "--episodes",
        type=int,
        default=100,
        help="Episodes for the expert-baseline measurement. Randomized tasks (coop_random) are "
        "high-variance, so keep this large (>=100) for a stable baseline line.",
    )
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--runs-root", type=Path, default=REPO_ROOT / "runs", help="Where training logs are written.")
    parser.add_argument("--models-root", type=Path, default=REPO_ROOT / "models", help="Where models are written.")
    parser.add_argument("--figures-root", type=Path, default=REPO_ROOT / "figures", help="Where figures are written.")
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Fast pipeline smoke test: 20k timesteps and few baseline episodes (graphs are sparse).",
    )
    args = parser.parse_args()

    if args.quick:
        args.timesteps = 20_000
        args.eval_every = 5_000
        args.episodes = 5

    print(f"Reproducing {len(args.difficulties)} demo(s): {', '.join(args.difficulties)}")
    print(f"  timesteps/training={args.timesteps}, residual_scale={args.residual_scale}, seed={args.seed}")
    print("  Each demo trains Residual + PPO-from-scratch, then renders the figures.\n")

    summary = []
    for difficulty in args.difficulties:
        print(f"\n{'=' * 70}\n### DEMO: {difficulty}\n{'=' * 70}")

        residual_run, residual_models = train(
            difficulty, "residual",
            timesteps=args.timesteps, eval_every=args.eval_every,
            residual_scale=args.residual_scale, seed=args.seed,
            runs_root=args.runs_root, models_root=args.models_root,
        )
        scratch_run, scratch_models = train(
            difficulty, "scratch",
            timesteps=args.timesteps, eval_every=args.eval_every,
            residual_scale=args.residual_scale, seed=args.seed,
            runs_root=args.runs_root, models_root=args.models_root,
        )

        print(f"\n--- Measuring scripted-expert baseline on {difficulty} ({args.episodes} episodes) ---")
        baseline = expert_baseline_percent(difficulty, args.episodes, args.max_episode_steps)
        print(f"Expert baseline: {baseline:.0f}% success")

        figures_dir = args.figures_root / f"demo_{difficulty}"
        make_figures(
            difficulty, residual_run, scratch_run, baseline, figures_dir,
            residual_model=residual_models / "box_push_ppo_final.zip",
            scratch_model=scratch_models / "box_push_ppo_final.zip",
            residual_scale=args.residual_scale, episodes=args.episodes,
        )
        summary.append((difficulty, baseline, figures_dir))

    print(f"\n{'=' * 70}\nDONE. Reproduced {len(summary)} demo(s):\n{'=' * 70}")
    for difficulty, baseline, figures_dir in summary:
        print(f"  {difficulty:14s} expert baseline {baseline:3.0f}%  ->  figures in {figures_dir}/")
        print(f"                 learning curves:  {figures_dir}/overview.png")
        print(f"                 PPO vs Residual (over time): {figures_dir}/method_comparison.png")
        print(f"                 FINAL result (headline):     {figures_dir}/final_comparison.png")


if __name__ == "__main__":
    main()
