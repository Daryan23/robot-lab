"""Bar chart of the *final* deterministic success rate: Expert vs Residual vs PPO.

Training-time curves on the randomized ``coop_random`` task are noisy (rolling
training success includes exploration; the periodic eval uses only a few
episodes). For a trustworthy report figure this script instead evaluates each
final policy **deterministically over many episodes** and draws one bar per
method with the number on top -- an honest, apples-to-apples comparison.

Example::

    python scripts/plot_final_comparison.py --difficulty coop_random \
        --residual-model models/demo_coop_random_residual/box_push_ppo_final.zip \
        --scratch-model  models/demo_coop_random_scratch/box_push_ppo_final.zip \
        --residual-scale 0.4 --episodes 200 --output figures/demo_coop_random
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from evaluate_box_push import evaluate_one  # noqa: E402
from robot_lab_rl import BoxPushHeuristicPolicy  # noqa: E402


class _ExpertModel:
    """Adapter so the scripted expert can be evaluated with ``evaluate_one``."""

    def __init__(self) -> None:
        self.expert = BoxPushHeuristicPolicy()

    def predict(self, observation, deterministic: bool = True):
        return self.expert.predict(observation), None


def success_percent(model, difficulty, episodes, max_steps, *, residual, residual_scale=0.4) -> float:
    result = evaluate_one(
        model, difficulty, episodes, max_steps,
        randomization=0.0, residual=residual, residual_scale=residual_scale,
    )
    return result["success_rate"] * 100.0


def main() -> None:
    parser = argparse.ArgumentParser(description="Bar chart of final deterministic success: Expert vs Residual vs PPO.")
    parser.add_argument("--difficulty", required=True)
    parser.add_argument("--residual-model", type=Path, required=True, help="Trained residual policy .zip.")
    parser.add_argument("--scratch-model", type=Path, required=True, help="Trained PPO-from-scratch policy .zip.")
    parser.add_argument("--residual-scale", type=float, default=0.4)
    parser.add_argument("--episodes", type=int, default=200, help="Deterministic evaluation episodes per method.")
    parser.add_argument("--max-episode-steps", type=int, default=800)
    parser.add_argument("--output", type=Path, default=Path("figures"))
    parser.add_argument("--format", choices=("png", "pdf"), default="png")
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    print(f"Evaluating final policies on {args.difficulty} ({args.episodes} deterministic episodes each)...")
    expert = success_percent(_ExpertModel(), args.difficulty, args.episodes, args.max_episode_steps, residual=False)
    print(f"  Expert (allein):          {expert:.1f}%")
    residual = success_percent(
        PPO.load(args.residual_model), args.difficulty, args.episodes, args.max_episode_steps,
        residual=True, residual_scale=args.residual_scale,
    )
    print(f"  Residual (Expert+Korr.):  {residual:.1f}%")
    scratch = success_percent(
        PPO.load(args.scratch_model), args.difficulty, args.episodes, args.max_episode_steps, residual=False,
    )
    print(f"  PPO von Null:             {scratch:.1f}%")

    labels = ["Expert\n(allein)", "Residual\n(Expert + Korrektur)", "PPO von Null\n(ohne Expert)"]
    values = [expert, residual, scratch]
    colors = ["gray", "tab:blue", "tab:red"]

    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    bars = ax.bar(labels, values, color=colors, width=0.6)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, value + 1.5, f"{value:.0f}%", ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax.set_ylabel("Erfolgsrate [%]")
    ax.set_ylim(0, 105)
    ax.set_title(f"Endergebnis: {args.difficulty}\n(deterministisch, {args.episodes} Episoden)")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()

    args.output.mkdir(parents=True, exist_ok=True)
    out = args.output / f"final_comparison.{args.format}"
    fig.savefig(out, dpi=args.dpi)
    print(f"saved {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
