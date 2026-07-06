"""Render report-quality figures of a box-push training run from TensorBoard logs.

This reads the scalar logs written during training (``runs/.../ppo_*``) and
produces PNG/PDF figures suitable for a report, focused on *how the robots
learn*: success rate over time, average reward, the learned correction on top
of the scripted expert (residual runs), and the episode-outcome breakdown.

Pass a single run to plot a single run, or **several seed runs** to plot the
mean curve with a shaded ``±1 std`` error band across seeds -- the honest way
to show that a result is reproducible and not a lucky seed::

    # single run
    ./.conda-env/bin/python scripts/plot_training_progress.py \
        --logdir runs/box_push_ppo --output figures

    # error bands over several seeds (each --logdir is one seed's run dir)
    ./.conda-env/bin/python scripts/plot_training_progress.py \
        --logdir runs/coop_random_seed0 runs/coop_random_seed1 runs/coop_random_seed2 \
        --output figures --expert-baseline 58
"""

from __future__ import annotations

import argparse
from functools import partial
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator  # noqa: E402


def resolve_run_dir(logdir: Path) -> Path:
    """Return the directory that actually holds tfevents files.

    Accepts either the directory passed to ``--logdir`` directly or its parent;
    if several ``ppo_*`` (or other) run subdirectories exist, the most recently
    modified one is used.
    """

    if list(logdir.glob("events.out.tfevents.*")):
        return logdir
    candidates = [path for path in logdir.glob("**/") if list(path.glob("events.out.tfevents.*"))]
    if not candidates:
        raise FileNotFoundError(
            f"No TensorBoard event files found under {logdir}. Run a training first."
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_scalars(run_dir: Path) -> tuple[dict[str, tuple[list[int], list[float]]], list[str]]:
    accumulator = EventAccumulator(str(run_dir), size_guidance={"scalars": 0})
    accumulator.Reload()
    tags = accumulator.Tags().get("scalars", [])
    series: dict[str, tuple[list[int], list[float]]] = {}
    for tag in tags:
        events = accumulator.Scalars(tag)
        series[tag] = ([event.step for event in events], [event.value for event in events])
    return series, sorted(tags)


def collect_seeds(run_dirs: list[Path]) -> tuple[list[dict], list[str]]:
    """Load every seed run and return (per-seed series dicts, union of tags)."""
    seeds = [load_scalars(run_dir)[0] for run_dir in run_dirs]
    tags = sorted({tag for series in seeds for tag in series})
    return seeds, tags


def aggregate(seeds: list[dict], tag: str, scale: float = 1.0):
    """Mean/std of one scalar tag across seeds on a shared step grid.

    Each seed may log the tag at slightly different steps, so every seed is
    linearly interpolated onto the union of steps *inside the range all seeds
    cover* (no extrapolation -- the band only spans where every seed has data).

    Returns ``(steps, mean, std, n_seeds)`` scaled by ``scale`` (e.g. 100 to
    turn a 0..1 rate into a percentage), or ``None`` if no seed has the tag.
    """
    present = [seed[tag] for seed in seeds if tag in seed and seed[tag][0]]
    if not present:
        return None
    if len(present) == 1:
        steps, values = present[0]
        grid = np.asarray(steps, dtype=float)
        mean = np.asarray(values, dtype=float) * scale
        return grid, mean, np.zeros_like(mean), 1
    low = max(min(steps) for steps, _ in present)
    high = min(max(steps) for steps, _ in present)
    if high <= low:  # seeds do not overlap in time -- fall back to the first
        steps, values = present[0]
        grid = np.asarray(steps, dtype=float)
        mean = np.asarray(values, dtype=float) * scale
        return grid, mean, np.zeros_like(mean), 1
    grid = np.asarray(
        sorted({step for steps, _ in present for step in steps if low <= step <= high}),
        dtype=float,
    )
    stacked = np.vstack([np.interp(grid, steps, values) for steps, values in present]) * scale
    return grid, stacked.mean(axis=0), stacked.std(axis=0), len(present)


def draw(ax, agg, label, color, band=True, **plot_kw) -> None:
    """Plot the mean line and, for >1 seed, a shaded ±1 std band."""
    grid, mean, std, n_seeds = agg
    line_label = f"{label} (Ø {n_seeds} Seeds)" if n_seeds > 1 else label
    ax.plot(grid, mean, color=color, label=line_label, **plot_kw)
    if band and n_seeds > 1 and np.any(std > 0):
        ax.fill_between(grid, mean - std, mean + std, color=color, alpha=0.18, linewidth=0)


def first_present(tags: list[str], *candidates: str) -> str | None:
    for candidate in candidates:
        if candidate in tags:
            return candidate
    return None


def matching(tags: list[str], prefix: str, suffix: str) -> str | None:
    for tag in tags:
        if tag.startswith(prefix) and tag.endswith(suffix):
            return tag
    return None


def plot_success(ax, seeds, tags, baseline=None, band=True) -> bool:
    plotted = False
    if baseline is not None:
        ax.axhline(baseline, ls="--", color="gray", linewidth=1.2, label=f"Expert-Start ({baseline:.0f} %)")
        plotted = True
    eval_current = matching(tags, "eval_current_", "/success_rate")
    eval_full = first_present(tags, "eval_full/success_rate")
    rolling = first_present(tags, "box_push/success_rate_100")
    if eval_current:
        agg = aggregate(seeds, eval_current, 100.0)
        if agg:
            difficulty = eval_current.split("/")[0].replace("eval_current_", "")
            draw(ax, agg, f"Eval ({difficulty})", "tab:blue", band, marker="o", markersize=3)
            plotted = True
    if eval_full and eval_full != eval_current:
        agg = aggregate(seeds, eval_full, 100.0)
        if agg:
            draw(ax, agg, "Eval (full task)", "tab:cyan", band, marker="s", markersize=3)
            plotted = True
    if rolling:
        agg = aggregate(seeds, rolling, 100.0)
        if agg:
            draw(ax, agg, "Training (rolling 100)", "tab:orange", band, alpha=0.9)
            plotted = True
    ax.set_title("Wie die Roboter lernen: Erfolgsrate")
    ax.set_xlabel("Trainings-Timesteps")
    ax.set_ylabel("Erfolgsrate [%]")
    ax.set_ylim(-2, 102)
    ax.grid(True, alpha=0.3)
    if plotted:
        ax.legend(fontsize=8)
    return plotted


def plot_reward(ax, seeds, tags, band=True) -> bool:
    tag = first_present(tags, "rollout/ep_rew_mean")
    if tag is None:
        return False
    agg = aggregate(seeds, tag)
    if agg is None:
        return False
    draw(ax, agg, "Ø Reward", "tab:green", band)
    ax.set_title("Durchschnittlicher Episoden-Reward")
    ax.set_xlabel("Trainings-Timesteps")
    ax.set_ylabel("Ø Reward")
    ax.grid(True, alpha=0.3)
    if agg[3] > 1:
        ax.legend(fontsize=8)
    return True


def plot_correction(ax, seeds, tags, band=True) -> bool:
    tag = first_present(tags, "residual/correction_magnitude_100")
    if tag is None:
        return False
    agg = aggregate(seeds, tag)
    if agg is None:
        return False
    draw(ax, agg, "|Korrektur|", "tab:purple", band)
    ax.set_title("Gelernte Korrektur über dem Expert (|Δ|)")
    ax.set_xlabel("Trainings-Timesteps")
    ax.set_ylabel("mittlere |Korrektur| [norm.]")
    ax.grid(True, alpha=0.3)
    if agg[3] > 1:
        ax.legend(fontsize=8)
    return True


def plot_outcomes(ax, seeds, tags, band=True) -> bool:
    keys = [
        ("box_push/outcome_success_rate_100", "Erfolg", "tab:green"),
        ("box_push/outcome_stall_rate_100", "Stillstand", "tab:orange"),
        ("box_push/outcome_timeout_rate_100", "Timeout", "tab:red"),
    ]
    present = [(tag, label, color) for tag, label, color in keys if tag in tags]
    if not present:
        return False
    for tag, label, color in present:
        agg = aggregate(seeds, tag, 100.0)
        if agg:
            draw(ax, agg, label, color, band)
    ax.set_title("Episoden-Ausgang")
    ax.set_xlabel("Trainings-Timesteps")
    ax.set_ylabel("Anteil [%]")
    ax.set_ylim(-2, 102)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot box-push training progress for a report.")
    parser.add_argument(
        "--logdir",
        type=Path,
        nargs="+",
        default=[Path("runs/box_push_ppo")],
        help="One or more training log directories. Pass several seed runs to draw mean ±std error bands.",
    )
    parser.add_argument("--output", type=Path, default=Path("figures"), help="Where to save the figures.")
    parser.add_argument("--format", choices=("png", "pdf"), default="png")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument(
        "--no-band",
        action="store_true",
        help="Plot only the mean line, without the shaded ±std error band, even with several seeds.",
    )
    parser.add_argument(
        "--expert-baseline",
        type=float,
        default=None,
        help="Draw a horizontal reference line at this success rate in %% (e.g. the scripted expert's success).",
    )
    args = parser.parse_args()

    run_dirs = [resolve_run_dir(logdir) for logdir in args.logdir]
    print(f"Reading TensorBoard logs from {len(run_dirs)} run(s):")
    for run_dir in run_dirs:
        print(f"  - {run_dir}")
    seeds, tags = collect_seeds(run_dirs)
    if not tags:
        raise SystemExit("No scalar tags found - has the training produced any logs yet?")
    if len(run_dirs) > 1:
        print(f"Aggregating {len(run_dirs)} seeds -> mean ±1 std error bands.")

    band = not args.no_band
    args.output.mkdir(parents=True, exist_ok=True)
    panels = [
        ("learning_success_rate", partial(plot_success, baseline=args.expert_baseline, band=band)),
        ("reward", partial(plot_reward, band=band)),
        ("correction", partial(plot_correction, band=band)),
        ("episode_outcomes", partial(plot_outcomes, band=band)),
    ]

    # Individual figures (each usable standalone in the report).
    active = []
    for name, plot_fn in panels:
        fig, ax = plt.subplots(figsize=(6.0, 4.0))
        if plot_fn(ax, seeds, tags):
            fig.tight_layout()
            out = args.output / f"{name}.{args.format}"
            fig.savefig(out, dpi=args.dpi)
            print(f"  saved {out}")
            active.append((name, plot_fn))
        plt.close(fig)

    # Combined overview figure (one image for the report).
    if active:
        columns = 2
        rows = (len(active) + columns - 1) // columns
        fig, axes = plt.subplots(rows, columns, figsize=(6.0 * columns, 4.0 * rows), squeeze=False)
        flat_axes = [ax for row in axes for ax in row]
        for (name, plot_fn), ax in zip(active, flat_axes):
            plot_fn(ax, seeds, tags)
        for ax in flat_axes[len(active):]:
            ax.axis("off")
        seed_note = f" (Ø {len(run_dirs)} Seeds ±1 std)" if len(run_dirs) > 1 else ""
        fig.suptitle(f"Box-Push: Trainingsfortschritt{seed_note}", fontsize=14)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        out = args.output / f"overview.{args.format}"
        fig.savefig(out, dpi=args.dpi)
        print(f"  saved {out}")
        plt.close(fig)

    print(f"Done. {len(active)} panel(s) written to {args.output}/")


if __name__ == "__main__":
    main()
